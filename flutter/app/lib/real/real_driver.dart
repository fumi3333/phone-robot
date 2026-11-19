import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:phone_robot_core/phone_robot_core.dart';

import '../platform/serial_link.dart';

/// 深度スキャンの受け渡し口。RealSession が DepthSource のストリームを
/// ここへ流し込み、RealDriver が「新しいスキャンが来たら地図へ登録」
/// というタイミング制御を行う。
class ScanFeed {
  Scan? latest;
  int seq = 0;
  DateTime? lastAt;

  void add(Scan s) {
    latest = s;
    seq++;
    lastAt = DateTime.now();
  }

  bool get isFresh =>
      lastAt != null &&
      DateTime.now().difference(lastAt!) < const Duration(milliseconds: 600);
}

/// ポーズの受け渡し口（ScanFeed と対）。RealSession が PoseSource の
/// ストリームをここへ流し込む。ポーズ本体に加えて「いつ届いたか」と
/// 「ARCore のトラッキング状態」を保持し、RealDriver の鮮度ゲート
/// （R1: 古いポーズ・トラッキング喪失では進まない）の入力になる。
class PoseFeed {
  Pose? latest;
  String trackingState = '未開始';
  DateTime? lastPoseAt;

  void addPose(Pose p) {
    latest = p;
    lastPoseAt = DateTime.now();
  }

  void setTrackingState(String s) {
    trackingState = s;
  }

  bool get isTracking => trackingState == 'TRACKING';

  /// 最新ポーズが [maxAge] 以内に届いていれば true。
  bool poseFresh(Duration maxAge) {
    final t = lastPoseAt;
    if (t == null || latest == null) return false;
    return DateTime.now().difference(t) <= maxAge;
  }
}

/// ウェイポイント走行の結果。
///
/// - [blocked] / [timeout]: 指令は届いていたが進めなかった（障害物の疑い）。
/// - [replan]: センサー喪失から復帰した。再計画して続行できる。
/// - [sensorLost]: センサーが復帰しなかった（[RealDriver.lastStopReason]
///   に 'tracking-lost' / 'scan-stale'）。地図へ障害物は書いていない。
/// - [linkLost]: ESP32 リンク切断または送信失敗（R3）。この場合も
///   地図へ障害物は書いていない。
enum DriveOutcome {
  arrived,
  blocked,
  timeout,
  aborted,
  replan,
  sensorLost,
  linkLost,
}

/// 実機ドライバ。
///
/// 中核（phone_robot_core）の MotionHooks は同期的だが、プラットフォーム
/// チャネル（ARCore / USB シリアル）は非同期にしか扱えない。そのため
/// 実機では、Explorer / GoalNavigator と同じ手順・同じガードを保ったまま、
/// 非同期の駆動ループとしてここに実装する。計画そのものは中核の部品
/// （OccupancyGrid.planMask / frontierCells / findPath）を共用するので、
/// 地図と計画の意味はシミュレーションと同一になる。
///
/// 経路（セル列）は方向転換点と最大間隔でウェイポイント列に圧縮し、
/// ARCore ポーズを見ながら連続制御で辿る（SPEC §5.3）。
///
/// 安全ゲート（NOTE10PLUS_RISKS R1〜R3）: 駆動の各周期で
/// (1) リンクが接続されていること、(2) トラッキングが TRACKING で
/// ポーズが新鮮なこと、(3) スキャンが新鮮なこと、を確認してからでないと
/// 駆動指令を出さない。喪失した場合は停止して短時間の復帰を待ち、
/// 復帰すれば再計画、しなければ理由付きで走行全体を止める。
/// 近接フェイルセーフ（新鮮なスキャンの最小距離が [proximityStopM] を
/// 切ったら即停止し、進行方向のセルを占有として記録して再計画）は
/// フェイルクローズ側に倒してある: スキャンが古ければ「障害物なし」
/// ではなく「進めない」になる。
class RealDriver {
  final MappingParams params;
  final OccupancyGrid grid;
  final Esp32Link link;
  final ScanFeed feed;

  /// believed pose の供給元（RealSession が PoseSource を流し込む）。
  final PoseFeed poseFeed;

  /// 状態メッセージの通知先（UI 表示用）。
  void Function(String status)? onStatus;

  /// 近接フェイルセーフの停止距離 [m]。
  /// スマホ（車体中心付近）から車体前面まで約 0.13 m + 余裕。
  double proximityStopM = 0.22;

  /// 通常走行の drive 正規化値（0..1）。
  double cruiseDrive = 0.6;

  /// ポーズの鮮度ゲート: これより古いポーズでは進まない（R1）。
  Duration poseMaxAge = const Duration(milliseconds: 500);

  /// トラッキング喪失時に復帰を待つ上限。これを超えたら諦めて止める。
  Duration trackingRecoveryTimeout = const Duration(seconds: 5);

  /// スキャンが古くなったときに新しいスキャンを待つ上限（R2）。
  Duration scanRecoveryTimeout = const Duration(seconds: 2);

  /// ウェイポイント1つあたりの走行タイムアウト。
  Duration waypointTimeout = const Duration(seconds: 30);

  /// sensorLost で止まった理由（'tracking-lost' / 'scan-stale' / 'link-lost'）。
  String? lastStopReason;

  /// HUD 表示用: 現在の目標進行方位 [rad] と前方自由距離 [m]。
  double? hudTargetHeading;
  double? hudTargetDistM;

  /// rotateTo で前方接近時に切り返し（K-turn）を行うか。
  bool enableKTurn = true;

  /// ステアリング出力の左右極性を反転するか（実車モーター配線合わせ用）。
  bool invertSteer = false;

  bool _aborted = false;
  bool _busy = false;
  int _appliedSeq = 0;
  double _lastScanX = 0;
  double _lastScanY = 0;
  double _lastScanHeading = 0;

  RealDriver({
    required this.params,
    required this.grid,
    required this.link,
    required this.feed,
    required this.poseFeed,
  });

  bool get isBusy => _busy;

  Pose get _pose {
    final p = poseFeed.latest;
    if (p == null) {
      throw StateError('ARCore のポーズがまだ得られていません');
    }
    return p;
  }

  /// ポーズが「信じて進める」状態か（TRACKING かつ新鮮）。
  bool get _poseOk => poseFeed.isTracking && poseFeed.poseFresh(poseMaxAge);

  Cell get currentCell {
    final p = _pose;
    final c = cellOf(params, p.x, p.y);
    return (
      r: c.r.clamp(0, params.gridH - 1),
      c: c.c.clamp(0, params.gridW - 1),
    );
  }

  double get heading => poseFeed.latest?.heading ?? _lastScanHeading;

  void _status(String s) => onStatus?.call(s);

  static double _normAngle(double a) {
    var r = a % (2 * math.pi);
    if (r > math.pi) r -= 2 * math.pi;
    if (r < -math.pi) r += 2 * math.pi;
    return r;
  }

  /// 停止を要求する。実行中のループは次の周期で止まり、's' を送る。
  Future<void> stop() async {
    _aborted = true;
    await link.sendJog('s');
  }

  // ------------------------------------------------------------------
  // 安全ゲート（R1 / R2 / R3）
  // ------------------------------------------------------------------

  /// リンク確認（R3）。接続されていれば null、切断なら linkLost。
  DriveOutcome? _checkLink() {
    if (link.state == Esp32LinkState.connected) return null;
    lastStopReason = 'link-lost';
    _status('ESP32 リンクの切断を検知しました。停止します');
    return DriveOutcome.linkLost;
  }

  /// ポーズの健全性ゲート（R1）。
  ///
  /// 健全なら null（そのまま進む）。喪失していた場合は停止して
  /// [trackingRecoveryTimeout] まで復帰を待ち、復帰したら replan
  /// （外側で再計画）、しなければ sensorLost を返す。
  Future<DriveOutcome?> _gatePose() async {
    if (_poseOk) return null;
    await link.sendJog('s');
    _status('トラッキング喪失を検知: 停止して復帰を待ちます');
    final sw = Stopwatch()..start();
    while (sw.elapsed < trackingRecoveryTimeout) {
      if (_aborted) return DriveOutcome.aborted;
      if (link.state != Esp32LinkState.connected) {
        return DriveOutcome.linkLost;
      }
      if (_poseOk) {
        _status('トラッキングが復帰しました。再計画します');
        return DriveOutcome.replan;
      }
      await Future<void>.delayed(const Duration(milliseconds: 50));
    }
    lastStopReason = 'tracking-lost';
    _status('トラッキングが復帰しませんでした。走行を止めます');
    return DriveOutcome.sensorLost;
  }

  /// スキャンの鮮度ゲート（R2、フェイルクローズ）。
  ///
  /// 新鮮なら null（そのまま進む）。古ければ停止して
  /// [scanRecoveryTimeout] まで新しいスキャンを待ち、来れば続行、
  /// 来なければ sensorLost を返す。「データなし＝障害物なし」にはしない。
  Future<DriveOutcome?> _gateScan() async {
    if (feed.isFresh) return null;
    await link.sendJog('s');
    _status('深度スキャンが古いため停止: 新しいスキャンを待ちます');
    final sw = Stopwatch()..start();
    while (sw.elapsed < scanRecoveryTimeout) {
      if (_aborted) return DriveOutcome.aborted;
      if (link.state != Esp32LinkState.connected) {
        return DriveOutcome.linkLost;
      }
      if (feed.isFresh) return null;
      await Future<void>.delayed(const Duration(milliseconds: 50));
    }
    lastStopReason = 'scan-stale';
    _status('新しいスキャンが得られませんでした。走行を止めます');
    return DriveOutcome.sensorLost;
  }

  // ------------------------------------------------------------------
  // スキャン登録
  // ------------------------------------------------------------------

  /// 新しいスキャンの到着を待ち、believed pose で地図へ登録する。
  /// タイムアウト時は登録せずに戻る（計測なしを異常扱いしない: SPEC §5.2）。
  /// ポーズが信じられない状態（トラッキング喪失・鮮度切れ）のスキャンは
  /// 位置がずれている疑いがあるため登録しない（R1）。
  Future<void> captureScan() async {
    final before = feed.seq;
    final sw = Stopwatch()..start();
    while (feed.seq == before && sw.elapsed < const Duration(seconds: 2)) {
      if (_aborted) return;
      await Future<void>.delayed(const Duration(milliseconds: 30));
    }
    final s = feed.latest;
    if (s == null || feed.seq == _appliedSeq) return;
    if (!_poseOk) return;
    _appliedSeq = feed.seq;
    grid.applyScan(s);
    _lastScanX = s.pose.x;
    _lastScanY = s.pose.y;
    _lastScanHeading = s.pose.heading;
  }

  /// 走行中に「十分に動いたら」最新スキャンを登録する（間引き）。
  void applyScanIfMoved() {
    if (!_poseOk) return;
    final s = feed.latest;
    if (s == null || feed.seq == _appliedSeq) return;
    final dx = s.pose.x - _lastScanX;
    final dy = s.pose.y - _lastScanY;
    final moved = math.sqrt(dx * dx + dy * dy);
    final turned = _normAngle(s.pose.heading - _lastScanHeading).abs();
    if (moved < 0.15 && turned < 0.30) return;
    _appliedSeq = feed.seq;
    grid.applyScan(s);
    _lastScanX = s.pose.x;
    _lastScanY = s.pose.y;
    _lastScanHeading = s.pose.heading;
  }

  // ------------------------------------------------------------------
  // 近接フェイルセーフ
  // ------------------------------------------------------------------

  /// 最新スキャンの最小距離 [m]。計測が無ければ null。
  /// 駆動中は _gateScan が鮮度を保証してから呼ばれる前提。
  double? get _minRange {
    final s = feed.latest;
    if (s == null || !feed.isFresh) return null;
    var m = double.infinity;
    for (final r in s.ranges) {
      if (r.isFinite && r < m) m = r;
    }
    return m.isFinite ? m : null;
  }

  bool get _proximityBlocked {
    final m = _minRange;
    return m != null && m < proximityStopM;
  }

  /// 進行方向のセルを強い占有証拠として記録する（SimRobot と同じ流儀）。
  ///
  /// これは「指令は届いたが進めなかった」場合専用（R3）。リンク切断・
  /// ポーズ不健全・スキャン不鮮のときは、進めなかった理由が障害物とは
  /// 限らないため記録しない（実在しない壁で地図を汚さない）。
  void _markBlockedAhead() {
    if (link.state != Esp32LinkState.connected) return;
    if (!_poseOk) return;
    if (!feed.isFresh) return;
    final p = _pose;
    final d = (_minRange ?? proximityStopM).clamp(0.10, 0.60);
    final bx = p.x + math.cos(p.heading) * d;
    final by = p.y + math.sin(p.heading) * d;
    final cell = cellOf(params, bx, by);
    if (!grid.inBounds(cell.r, cell.c)) return;
    final i = grid.index(cell.r, cell.c);
    if (grid.logOdds[i] < 2.0) grid.logOdds[i] = 2.0;
  }

  // ------------------------------------------------------------------
  // 基本動作（回転 / ウェイポイント走行 / パルス当て舵）
  // ------------------------------------------------------------------

  /// 送信結果をそのまま返す（R3: 送信失敗をドライバが見られるように）。
  /// 駆動はギアボックスの静止摩擦デッドゾーン補正済みの [driveOf] を使う。
  Future<bool> _sendNorm(double drive, double steer) {
    final s = invertSteer ? -steer : steer;
    return link.sendDriveSteer(
      SerialCommandScale.driveOf(drive),
      SerialCommandScale.of(s),
    );
  }

  /// Jozen 86 のバネ戻し付きバンバン操舵向け連続フルステア計算。
  /// 奇数ティックで 0 に落とすとバネで直進に戻ってしまうため、
  /// 不感帯（±0.09 rad ≈ 5°）の外では目標方向のフルステア（±1.0）を維持する。
  double _computePulseSteer(double err) {
    final absErr = err.abs();
    if (absErr < 0.09) {
      return 0.0;
    }
    return err > 0 ? 1.0 : -1.0;
  }

  /// ゲートの結果を回転動作の可否に読み替える。
  /// null / replan（復帰した）なら続行、それ以外は失敗として止める。
  bool _gateBlocks(DriveOutcome? g) => g != null && g != DriveOutcome.replan;

  /// その場向き変え。車両はステアリング式のため、転がしながら ARCore の
  /// 向きが目標に近づくまで操舵する（SPEC §7 の読み替え）。
  /// 前方が詰まった場合は [enableKTurn] 有効時に最大2回まで切り返しを行う。
  Future<bool> rotateTo(
    double targetHeading, {
    Duration timeout = const Duration(seconds: 12),
  }) async {
    final sw = Stopwatch()..start();
    var kTurns = 0;
    while (!_aborted && sw.elapsed < timeout) {
      if (_checkLink() != null) {
        await link.sendJog('s');
        return false;
      }
      if (_gateBlocks(await _gatePose())) {
        await link.sendJog('s');
        return false;
      }
      if (_gateBlocks(await _gateScan())) {
        await link.sendJog('s');
        return false;
      }
      final err = _normAngle(targetHeading - _pose.heading);
      if (err.abs() < 0.08) {
        await link.sendJog('s');
        return true;
      }
      final steer = err > 0 ? 1.0 : -1.0;
      if (_proximityBlocked && err.abs() > 0.3) {
        if (!enableKTurn || kTurns >= 2) {
          await link.sendJog('s');
          return false;
        }
        kTurns++;
        _status('前方接近のため切り返し（K-turn $kTurns/2）…');
        // 急反転による USB 電圧降下防止のため一瞬停止を挟み、逆ステアで後退
        await link.sendJog('s');
        await Future<void>.delayed(const Duration(milliseconds: 120));
        for (var step = 0; step < 3 && !_aborted; step++) {
          if (_checkLink() != null) {
            await link.sendJog('s');
            return false;
          }
          if (!await _sendNorm(-math.min(cruiseDrive, 0.55), -steer)) {
            lastStopReason = 'link-lost';
            _status('指令の送信に失敗しました（リンク切断の疑い）。停止します');
            await link.sendJog('s');
            return false;
          }
          await Future<void>.delayed(const Duration(milliseconds: 100));
        }
        await link.sendJog('s');
        await Future<void>.delayed(const Duration(milliseconds: 120));
        continue;
      }
      if (!await _sendNorm(math.min(cruiseDrive, 0.35), steer)) {
        lastStopReason = 'link-lost';
        _status('指令の送信に失敗しました（リンク切断の疑い）。停止します');
        await link.sendJog('s');
        return false;
      }
      await Future<void>.delayed(const Duration(milliseconds: 100));
    }
    await link.sendJog('s');
    return false;
  }

  /// 360° 回転スキャン（30° × 12 方向）。中核の rotationScan と同じ。
  Future<void> rotationScan() async {
    final base = heading;
    for (var k = 0; k < 12; k++) {
      if (_aborted) return;
      await rotateTo(base + k * math.pi / 6);
      await captureScan();
    }
  }

  /// セル列をウェイポイント列に圧縮する（方向転換点＋最大 0.5 m 間隔）。
  List<Cell> _compressWaypoints(List<Cell> path) {
    if (path.length <= 2) return List.of(path);
    final out = <Cell>[path.first];
    var lastKept = 0;
    for (var i = 1; i < path.length - 1; i++) {
      final dr1 = path[i].r - path[i - 1].r;
      final dc1 = path[i].c - path[i - 1].c;
      final dr2 = path[i + 1].r - path[i].r;
      final dc2 = path[i + 1].c - path[i].c;
      final turned = dr1 != dr2 || dc1 != dc2;
      final far = (i - lastKept) * params.cellSize >= 0.5;
      if (turned || far) {
        out.add(path[i]);
        lastKept = i;
      }
    }
    out.add(path.last);
    return out;
  }

  /// ウェイポイント（セル中心）まで連続制御で走行する。
  ///
  /// 各周期で安全ゲート（リンク→ポーズ→スキャン）を通ってからでないと
  /// 駆動指令を出さない。ゲートで止まった場合の結果:
  /// - リンク切断・送信失敗 → [DriveOutcome.linkLost]（地図に書かない）
  /// - センサー喪失から復帰 → [DriveOutcome.replan]
  /// - センサーが復帰しない → [DriveOutcome.sensorLost]
  Future<DriveOutcome> driveToWaypoint(Cell waypoint) async {
    final target = cellCenter(params, waypoint);
    final sw = Stopwatch()..start();
    while (!_aborted && sw.elapsed < waypointTimeout) {
      final linkOut = _checkLink();
      if (linkOut != null) {
        await link.sendJog('s');
        return linkOut;
      }
      final gp = await _gatePose();
      if (gp != null) {
        await link.sendJog('s');
        return gp;
      }
      final gs = await _gateScan();
      if (gs != null) {
        await link.sendJog('s');
        return gs;
      }
      final p = _pose;
      final dx = target.x - p.x;
      final dy = target.y - p.y;
      final dist = math.sqrt(dx * dx + dy * dy);
      if (dist < 0.07) {
        await link.sendJog('s');
        return DriveOutcome.arrived;
      }
      if (_proximityBlocked) {
        await link.sendJog('s');
        _markBlockedAhead();
        return DriveOutcome.blocked;
      }
      final desired = math.atan2(dy, dx);
      final err = _normAngle(desired - p.heading);
      hudTargetHeading = desired;
      hudTargetDistM = dist;
      final steer = _computePulseSteer(err);
      var drive = dist < 0.18 ? math.min(cruiseDrive, 0.40) : cruiseDrive;
      if (steer.abs() > 0.5) drive = math.min(drive, 0.38);
      if (!await _sendNorm(drive, steer)) {
        // 送信失敗 = リンクが死んでいる（R3）。障害物として扱わない。
        lastStopReason = 'link-lost';
        _status('指令の送信に失敗しました（リンク切断の疑い）。停止します');
        await link.sendJog('s');
        return DriveOutcome.linkLost;
      }
      applyScanIfMoved();
      await Future<void>.delayed(const Duration(milliseconds: 100));
    }
    await link.sendJog('s');
    if (_aborted) return DriveOutcome.aborted;
    // ゲートを通り続けたのに進めなかった = 実質的に塞がれている（安全側）。
    // _markBlockedAhead 側にも健全性ガードがある。
    _markBlockedAhead();
    return DriveOutcome.timeout;
  }

  // ------------------------------------------------------------------
  // 探索（中核 Explorer と同じ手順の非同期版）
  // ------------------------------------------------------------------

  Future<ExploreResult> explore({
    void Function(Float32List snapshot, Cell robot)? onFrame,
    Duration maxDuration = const Duration(minutes: 8),
    bool initialSpin = false,
  }) async {
    if (_busy) throw StateError('既に動作中です');
    _busy = true;
    _aborted = false;
    lastStopReason = null;
    final p = params;
    var steps = 0;
    var scans = 0;
    var failedPlans = 0;
    var sensorReplans = 0;
    var covAtLastReplan = -1.0;
    final sw = Stopwatch()..start();
    try {
      _status('初期スキャン中…');
      final start = currentCell;
      grid.logOdds[grid.index(start.r, start.c)] = p.lFree;
      await captureScan();
      scans++;
      if (initialSpin) {
        await rotationScan();
        scans += 12;
      }

      var cov = grid.coverage;
      var endReason = 'step-limit';

      while (cov < p.targetCoverage &&
          steps < 2000 &&
          !_aborted &&
          sw.elapsed < maxDuration) {
        final robot = currentCell;
        final mask = grid.planMask(forcedFree: robot);
        final fronts = grid.frontierCells(mask);
        if (fronts.isEmpty) {
          endReason = 'no-frontier';
          break;
        }
        fronts.sort((a, b) {
          final da =
              (a.r - robot.r) * (a.r - robot.r) +
              (a.c - robot.c) * (a.c - robot.c);
          final db =
              (b.r - robot.r) * (b.r - robot.r) +
              (b.c - robot.c) * (b.c - robot.c);
          return da.compareTo(db);
        });
        List<Cell>? path;
        for (final goal in fronts.take(60)) {
          if (goal == robot) continue;
          path = findPath(
            gridW: p.gridW,
            gridH: p.gridH,
            isFree: (i) => mask[i] == 1,
            start: robot,
            goal: goal,
          );
          if (path != null) break;
        }

        if (path == null || path.length < 2) {
          failedPlans++;
          if (failedPlans >= 2) {
            endReason = 'no-progress';
            break;
          }
          if (initialSpin) {
            await rotationScan();
            scans += 12;
          } else {
            await rotateTo(heading + math.pi / 3);
            await captureScan();
            scans++;
          }
          if (grid.coverage <= cov) {
            endReason = 'no-progress';
            break;
          }
          cov = grid.coverage;
          continue;
        }
        failedPlans = 0;
        _status('フロンティアへ移動中…（カバー率 ${(cov * 100).toStringAsFixed(1)}%）');

        final waypoints = _compressWaypoints(path);
        var blockedPath = false;
        String? fatal;
        for (final wp in waypoints.skip(1)) {
          final before = currentCell;
          final outcome = await driveToWaypoint(wp);
          final after = currentCell;
          final dCells = math.max(
            (after.r - before.r).abs(),
            (after.c - before.c).abs(),
          );
          steps += dCells;
          if (outcome == DriveOutcome.linkLost) {
            fatal = 'link-lost';
            break;
          }
          if (outcome == DriveOutcome.sensorLost) {
            fatal = lastStopReason ?? 'sensor-lost';
            break;
          }
          if (outcome == DriveOutcome.replan) {
            // センサー復帰。現在位置から計画し直す（回転スキャン経由）。
            if (cov <= covAtLastReplan + 1e-9) {
              sensorReplans++;
            } else {
              sensorReplans = 1;
            }
            covAtLastReplan = cov;
            break;
          }
          if (outcome == DriveOutcome.blocked ||
              outcome == DriveOutcome.timeout) {
            blockedPath = true;
            break;
          }
          if (outcome == DriveOutcome.aborted) break;
          await captureScan();
          scans++;
          if (steps % 10 == 0) {
            onFrame?.call(Float32List.fromList(grid.logOdds), currentCell);
          }
        }
        if (fatal != null) {
          endReason = fatal;
          break;
        }
        if (sensorReplans >= 3) {
          endReason = 'sensor-unstable';
          break;
        }
        if (_aborted) {
          endReason = 'aborted';
          break;
        }
        if (!blockedPath) {
          if (initialSpin) {
            await rotationScan();
            scans += 12;
          } else {
            await captureScan();
            scans++;
          }
        }
        cov = grid.coverage;
      }

      if (_aborted) endReason = 'aborted';
      final reached = cov >= p.targetCoverage;
      if (reached) endReason = 'target';
      _status('探索終了: $endReason（カバー率 ${(cov * 100).toStringAsFixed(1)}%）');
      return ExploreResult(
        steps: steps,
        scans: scans,
        coverage: cov,
        targetReached: reached,
        endReason: endReason,
      );
    } finally {
      hudTargetHeading = null;
      hudTargetDistM = null;
      await link.sendJog('s');
      _busy = false;
    }
  }

  // ------------------------------------------------------------------
  // ゴール指定ナビ（中核 GoalNavigator と同じ手順の非同期版）
  // ------------------------------------------------------------------

  Future<GoToResult> goTo({
    required Cell goal,
    KeepOutZones? keepOut,
    void Function(Float32List snapshot, Cell robot)? onFrame,
  }) async {
    if (_busy) throw StateError('既に動作中です');
    _busy = true;
    _aborted = false;
    lastStopReason = null;
    final p = params;
    final start = currentCell;
    final traversed = <Cell>[start];
    var steps = 0;
    var replans = 0;
    var lengthM = 0.0;

    GoToResult fail(String reason) => GoToResult(
      success: false,
      steps: steps,
      pathLengthM: lengthM,
      replans: replans,
      failureReason: reason,
      traversed: traversed,
    );

    try {
      if (!grid.inBounds(goal.r, goal.c)) return fail('goal-out-of-bounds');
      final goalIdx = grid.index(goal.r, goal.c);
      if (grid.logOdds[goalIdx] == 0) return fail('goal-unknown');
      if (!grid.isPlanFree(goalIdx)) return fail('goal-occupied');
      if (keepOut != null && keepOut.containsIndex(goalIdx)) {
        return fail('goal-keepout');
      }

      var noProgressPlans = 0;
      while (currentCell != goal) {
        if (_aborted) return fail('aborted');
        if (steps >= 2000) return fail('step-limit');
        final current = currentCell;
        final mask = grid.planMask(forcedFree: current);
        if (keepOut != null) {
          final curIdx = grid.index(current.r, current.c);
          for (final i in keepOut.cells) {
            if (i != curIdx) mask[i] = 0;
          }
        }
        final path = findPath(
          gridW: p.gridW,
          gridH: p.gridH,
          isFree: (i) => mask[i] == 1,
          start: current,
          goal: goal,
        );
        if (path == null || path.length < 2) return fail('unreachable');

        final waypoints = _compressWaypoints(path);
        var blocked = false;
        var movedThisPlan = 0;
        Pose? lastPose = poseFeed.latest;
        for (final wp in waypoints.skip(1)) {
          final outcome = await driveToWaypoint(wp);
          final now = poseFeed.latest;
          if (lastPose != null && now != null) {
            final dx = now.x - lastPose.x;
            final dy = now.y - lastPose.y;
            final d = math.sqrt(dx * dx + dy * dy);
            lengthM += d;
            steps += (d / p.cellSize).round();
            movedThisPlan += (d / p.cellSize).round();
          }
          lastPose = now;
          final cc = currentCell;
          if (traversed.last != cc) traversed.add(cc);
          if (outcome == DriveOutcome.linkLost) return fail('link-lost');
          if (outcome == DriveOutcome.sensorLost) {
            return fail(lastStopReason ?? 'sensor-lost');
          }
          if (outcome == DriveOutcome.blocked ||
              outcome == DriveOutcome.timeout ||
              outcome == DriveOutcome.replan) {
            // replan はセンサー復帰後の再計画。通常の塞がりと同じ
            // 経路で数えるが、障害物の記録は driveToWaypoint 側で
            // 行われていない（ゲート経由のため）。
            blocked = true;
            break;
          }
          if (outcome == DriveOutcome.aborted) return fail('aborted');
          await captureScan();
          onFrame?.call(Float32List.fromList(grid.logOdds), cc);
          if (cc == goal) break;
        }

        if (currentCell == goal) break;
        if (blocked) {
          replans++;
          if (movedThisPlan == 0) {
            noProgressPlans++;
            if (noProgressPlans >= 2) return fail('blocked-no-progress');
          } else {
            noProgressPlans = 0;
          }
          if (replans > 20) return fail('too-many-replans');
        }
      }

      return GoToResult(
        success: true,
        steps: steps,
        pathLengthM: lengthM,
        replans: replans,
        failureReason: null,
        traversed: traversed,
      );
    } finally {
      hudTargetHeading = null;
      hudTargetDistM = null;
      await link.sendJog('s');
      _busy = false;
    }
  }

  // ------------------------------------------------------------------
  // VFH フロアマップ作成（21 本深度レイ＋フロンティア誘導）
  // ------------------------------------------------------------------

  /// 深度スキャン（21 本）の隣接 3 本移動平均に加え、未探索領域（logOdds == 0）
  /// 利得と最寄りフロンティア方位をスコア化して進路を選出し、パルス当て舵で
  /// 走行しながら占有格子地図を完成させる。
  /// 前方中央コーンが 0.32 m 未満に詰まった場合は自動切り返しで回避する。
  Future<void> cruiseVfh({
    void Function(Float32List snapshot, Cell robot)? onFrame,
    Duration maxDuration = const Duration(minutes: 10),
  }) async {
    if (_busy) throw StateError('既に動作中です');
    _busy = true;
    _aborted = false;
    lastStopReason = null;
    final p = params;
    final offsets = rayOffsets(p);
    final sw = Stopwatch()..start();
    var ticks = 0;
    double? frontierBearing;

    try {
      _status('フロアマップ作成（VFH）を開始…');
      if (_gateBlocks(await _gatePose())) {
        return;
      }
      final start = currentCell;
      grid.logOdds[grid.index(start.r, start.c)] = p.lFree;
      await captureScan();
      onFrame?.call(Float32List.fromList(grid.logOdds), currentCell);

      while (!_aborted &&
          sw.elapsed < maxDuration &&
          grid.coverage < p.targetCoverage) {
        if (_checkLink() != null) break;
        if (_gateBlocks(await _gatePose())) break;
        if (_gateBlocks(await _gateScan())) break;

        final pose = _pose;
        final scan = feed.latest;
        if (scan == null || scan.ranges.isEmpty) {
          await Future<void>.delayed(const Duration(milliseconds: 50));
          continue;
        }
        final nRays = math.min(offsets.length, scan.ranges.length);

        double rayDist(int idx) {
          final r = scan.ranges[idx];
          if (!r.isFinite || r <= 0) return 2.2;
          return r.clamp(0.05, p.maxRange);
        }

        // 車幅コリドー（±12cm）内の前方障害物距離（2番目最小値で単発ノイズを除外）
        final corridorVals = <double>[];
        for (var i = 0; i < nRays; i++) {
          final r = scan.ranges[i];
          if (!r.isFinite || r <= 0) continue;
          final theta =
              nRays == 1 ? 0.0 : -p.halfFov + (2 * p.halfFov) * i / (nRays - 1);
          final lateralM = (r * math.sin(theta)).abs();
          if (lateralM <= 0.12) {
            corridorVals.add(r * math.cos(theta));
          }
        }
        corridorVals.sort();
        final double? centerMin = corridorVals.isEmpty
            ? null
            : (corridorVals.length >= 2 ? corridorVals[1] : corridorVals[0]);

        final frontBlocked =
            _proximityBlocked || (centerMin != null && centerMin <= 0.35);

        if (frontBlocked) {
          hudTargetHeading = null;
          hudTargetDistM = centerMin ?? proximityStopM;
          _status(
            '前方停止（${(centerMin ?? proximityStopM).toStringAsFixed(2)} m）',
          );
          await link.sendJog('s');
          await Future<void>.delayed(const Duration(milliseconds: 100));
          if (_aborted) break;
          applyScanIfMoved();
          onFrame?.call(Float32List.fromList(grid.logOdds), currentCell);
          continue;
        }

        // 1 秒ごとに 0.4 m 以上先の最寄りフロンティア方位を更新（既知エリアの往復回避）
        if (ticks % 10 == 0) {
          final robot = currentCell;
          final mask = grid.planMask(forcedFree: robot);
          final fronts = grid.frontierCells(mask);
          Cell? nearest;
          var bestDistSq = double.infinity;
          for (final f in fronts) {
            final fc = cellCenter(p, f);
            final dx = fc.x - pose.x;
            final dy = fc.y - pose.y;
            final d2 = dx * dx + dy * dy;
            if (d2 >= 0.16 && d2 < bestDistSq) {
              bestDistSq = d2;
              nearest = f;
            }
          }
          if (nearest != null) {
            final fc = cellCenter(p, nearest);
            frontierBearing = math.atan2(fc.y - pose.y, fc.x - pose.x);
          } else {
            frontierBearing = null;
          }
        }

        // 隣接 3 本移動平均＋未探索セル利得＋フロンティア誘導で進路を選ぶ
        var bestIdx = nRays ~/ 2;
        var bestScore = -1.0;
        final halfFov = p.fovRad / 2.0;
        for (var i = 0; i < nRays; i++) {
          var sum = 0.0;
          var count = 0;
          var minWin = double.infinity;
          for (
            var j = math.max(0, i - 1);
            j <= math.min(nRays - 1, i + 1);
            j++
          ) {
            final d = rayDist(j);
            sum += d;
            if (d < minWin) minWin = d;
            count++;
          }
          final avg = sum / count;
          final angleWeight = 1.0 - 0.12 * (offsets[i].abs() / halfFov);
          final measuredBonus = (scan.ranges[i].isFinite && scan.ranges[i] > 0)
              ? 0.30
              : 0.0;

          final double score;
          if (minWin < 0.32) {
            score = minWin * 0.3;
          } else {
            final ang = scan.pose.heading + offsets[i];
            final ca = math.cos(ang);
            final sa = math.sin(ang);
            final probeEnd = math.min(avg, 3.0);
            var probeTotal = 0;
            var probeUnknown = 0;
            for (var d = 0.4; d <= probeEnd + 1e-6; d += 0.4) {
              final c = cellOf(p, scan.pose.x + ca * d, scan.pose.y + sa * d);
              if (!grid.inBounds(c.r, c.c)) break;
              probeTotal++;
              if (grid.logOdds[grid.index(c.r, c.c)] == 0) {
                probeUnknown++;
              }
            }
            final unknownRatio = probeTotal > 0
                ? probeUnknown / probeTotal
                : 0.0;
            final frontierBonus = frontierBearing == null
                ? 0.0
                : 0.4 *
                      math.max(
                        0.0,
                        1.0 -
                            _normAngle(frontierBearing - ang).abs() / p.fovRad,
                      );
            score =
                avg * angleWeight +
                measuredBonus +
                0.6 * unknownRatio +
                frontierBonus;
          }
          if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
          }
        }

        final targetHeading = _normAngle(scan.pose.heading + offsets[bestIdx]);
        final targetDist = rayDist(bestIdx);
        hudTargetHeading = targetHeading;
        hudTargetDistM = targetDist;

        final err = _normAngle(targetHeading - pose.heading);
        final steer = _computePulseSteer(err);
        final frontClear = centerMin ?? targetDist;
        var drive = frontClear < 0.65
            ? math.min(cruiseDrive, 0.40)
            : cruiseDrive;
        if (steer.abs() > 0.5) drive = math.min(drive, 0.38);

        if (!await _sendNorm(drive, steer)) {
          lastStopReason = 'link-lost';
          _status('指令の送信に失敗しました（リンク切断の疑い）。停止します');
          break;
        }

        applyScanIfMoved();
        ticks++;
        if (ticks % 5 == 0) {
          final deg = (offsets[bestIdx] * 180 / math.pi).toStringAsFixed(0);
          _status(
            'フロアマップ作成中（VFH）: 進路 ${deg}° / 前方 ${frontClear.toStringAsFixed(2)} m / '
            '${(grid.coverage * 100).toStringAsFixed(1)}%',
          );
          onFrame?.call(Float32List.fromList(grid.logOdds), currentCell);
        }
        await Future<void>.delayed(const Duration(milliseconds: 100));
      }
      if (_aborted) {
        _status('フロアマップ作成を停止しました');
      } else if (lastStopReason == null && grid.coverage >= p.targetCoverage) {
        _status(
          'フロアマップ作成完了（カバー率 ${(grid.coverage * 100).toStringAsFixed(1)}%）',
        );
      }
    } finally {
      hudTargetHeading = null;
      hudTargetDistM = null;
      await link.sendJog('s');
      _busy = false;
    }
  }
}
