import 'dart:async';

import 'package:phone_robot_core/phone_robot_core.dart';

import '../platform/depth_source.dart';
import '../platform/keep_awake.dart';
import '../platform/pose_source.dart';
import '../platform/serial_link.dart';
import 'real_driver.dart';

/// 実機モードのセッション。
///
/// ARCore（PoseSource / DepthSource）と ESP32（Esp32Link）を束ね、
/// OccupancyGrid と RealDriver を駆動する。UI は [updates] を購読して
/// 再描画し、各状態は getter で読む。
///
/// R4 対策（NOTE10PLUS_RISKS）:
/// - センサー開始中は画面を常時点灯する（KeepAwake → ネイティブ側で
///   FLAG_KEEP_SCREEN_ON）。消灯がプロセス凍結の引き金になるため。
/// - ポーズ／深度ストリームの無音をウォッチドッグで監視し、黙ったら
///   ドライバを止めて状態表示に出す（[sensorHealth] / [sensorsStalled]）。
/// - アプリが背面に移ったら走行を止める（[handleAppPaused]）。
///   復帰しても走行は自動再開せず、次に走行を開始するときに現在位置
///   から再計画する（[handleAppResumed]）。
class RealSession {
  static const MappingParams params = MappingParams();

  /// ストリーム無音とみなすまでの時間。ドライバ側のゲート
  /// （ポーズ 500 ms / スキャン 600 ms）より長く、こちらは
  /// 「セッションが死んだ」判定と復帰導線のための閾値。
  static const Duration stallThreshold = Duration(seconds: 3);

  final OccupancyGrid grid = OccupancyGrid(params);
  final MethodChannelPoseSource poseSource = MethodChannelPoseSource();
  final MethodChannelDepthSource depthSource =
      const MethodChannelDepthSource();
  final MethodChannelEsp32Link link = MethodChannelEsp32Link();
  final ScanFeed feed = ScanFeed();
  final PoseFeed poseFeed = PoseFeed();

  late final RealDriver driver = RealDriver(
    params: params,
    grid: grid,
    link: link,
    feed: feed,
    poseFeed: poseFeed,
  );

  Esp32LinkState linkState = Esp32LinkState.disconnected;
  bool sensorsOn = false;
  bool explored = false;
  String status = '実機モード: まずセンサー開始と ESP32 接続をしてください';
  Cell? goal;
  List<Cell>? path;

  /// センサーストリームの健全性表示（'正常' / 'ポーズ無音 N 秒' 等）。
  String sensorHealth = '—';

  /// ポーズの配信が止まり「セッションが死んだ」判定になったか。
  bool sensorsStalled = false;

  DateTime? _sensorsStartedAt;
  Timer? _watchdog;

  final _updates = StreamController<void>.broadcast();
  Stream<void> get updates => _updates.stream;

  final List<StreamSubscription<dynamic>> _subs = [];
  Timer? _notifyTimer;

  RealSession() {
    driver.onStatus = (s) {
      status = s;
      _notify();
    };
  }

  Pose? get latestPose => poseFeed.latest;

  String get trackingState => poseFeed.trackingState;

  void _notify() {
    // 高頻度のポーズ更新での再描画を間引く
    _notifyTimer ??= Timer(const Duration(milliseconds: 120), () {
      _notifyTimer = null;
      if (!_updates.isClosed) _updates.add(null);
    });
  }

  Cell? get robotCell {
    final p = latestPose;
    if (p == null) return null;
    final c = cellOf(params, p.x, p.y);
    if (!grid.inBounds(c.r, c.c)) return null;
    return c;
  }

  bool get busy => driver.isBusy;

  Future<void> startSensors() async {
    if (sensorsOn) return;
    await poseSource.start();
    sensorsOn = true;
    _sensorsStartedAt = DateTime.now();
    sensorsStalled = false;
    sensorHealth = '正常';
    _subs.add(poseSource.poses.listen((pose) {
      poseFeed.addPose(pose);
      _notify();
    }));
    _subs.add(poseSource.trackingStates.listen((s) {
      poseFeed.setTrackingState(s);
      _notify();
    }));
    _subs.add(depthSource.scans(() => latestPose ?? Pose(0, 0, 0)).listen((s) {
      feed.add(s);
    }));
    _subs.add(link.states.listen((s) {
      linkState = s;
      _notify();
    }));
    // R4: 実機モード中は画面を消さない＋無音ウォッチドッグを開始。
    unawaited(KeepAwake.setEnabled(true));
    _watchdog?.cancel();
    _watchdog = Timer.periodic(
      const Duration(milliseconds: 500),
      (_) => evaluateSensorHealth(),
    );
    status = 'センサー開始: ARCore のトラッキングを待っています';
    _notify();
  }

  Future<void> stopSensors() async {
    _watchdog?.cancel();
    _watchdog = null;
    unawaited(KeepAwake.setEnabled(false));
    for (final s in _subs) {
      await s.cancel();
    }
    _subs.clear();
    await poseSource.stop();
    sensorsOn = false;
    sensorsStalled = false;
    sensorHealth = '—';
    poseFeed.setTrackingState('停止');
    _notify();
  }

  /// ストリームの無音を評価する（ウォッチドッグから定期的に呼ばれる）。
  ///
  /// ポーズが [stallThreshold] を超えて黙ったら「セッションが死んだ」
  /// とみなし、ドライバを止めて復帰導線を状態に出す。深度だけの無音は
  /// 走行中ならドライバを止める（ドライバ側ゲートが先に働く前提の
  /// 二重の安全）。配信が戻れば [sensorsStalled] を自動で解除するが、
  /// 走行までは自動で再開しない。
  void evaluateSensorHealth() {
    if (!sensorsOn) return;
    final now = DateTime.now();
    final startedAt = _sensorsStartedAt ?? now;
    final poseAge = poseFeed.lastPoseAt == null
        ? now.difference(startedAt)
        : now.difference(poseFeed.lastPoseAt!);
    final scanAge = feed.lastAt == null
        ? now.difference(startedAt)
        : now.difference(feed.lastAt!);

    final health = poseAge > stallThreshold
        ? 'ポーズ無音 ${poseAge.inSeconds} 秒'
        : scanAge > stallThreshold
            ? '深度無音 ${scanAge.inSeconds} 秒'
            : '正常';
    if (health != sensorHealth) sensorHealth = health;

    final stalledNow = poseAge > stallThreshold;
    if (stalledNow && !sensorsStalled) {
      sensorsStalled = true;
      status = 'センサーの配信が止まりました（ポーズ無音）。'
          '安全のため停止しました。'
          '「センサー停止」→「センサー開始」で再開してください';
      unawaited(driver.stop());
    } else if (!stalledNow && sensorsStalled) {
      sensorsStalled = false;
      status = 'センサーの配信が復帰しました。'
          '走行は自動では再開しません（開始時に現在位置から再計画します）';
    } else if (driver.isBusy && scanAge > stallThreshold) {
      status = '深度の配信が止まったため停止しました';
      unawaited(driver.stop());
    }
    _notify();
  }

  /// アプリが背面に移ったとき（R4）。走行中なら止める。
  /// セッション自体は殺さないが、復帰後に無音評価で実態と合わせる。
  Future<void> handleAppPaused() async {
    if (!driver.isBusy) return;
    await driver.stop();
    status = 'アプリが背面に移ったため走行を停止しました。'
        '復帰後にセンサーの状態を確認してから再開してください';
    _notify();
  }

  /// アプリに復帰したとき（R4）。無音評価を即時にやり直し、
  /// 生きていれば「再開は手動・開始時に再計画」と案内する。
  /// 走行の自動再開はしない。
  void handleAppResumed() {
    if (!sensorsOn) return;
    evaluateSensorHealth();
    if (!sensorsStalled) {
      status = 'アプリに復帰しました。センサーの配信を確認してから'
          '走行を再開してください（自動では再開しません。'
          '再開時は現在位置から再計画します）';
      _notify();
    }
  }

  Future<bool> connectSerial() async {
    final ok = await link.connect();
    linkState = link.state;
    status = ok ? 'ESP32 に接続しました' : 'ESP32 の接続に失敗しました';
    _notify();
    return ok;
  }

  Future<void> disconnectSerial() async {
    await driver.stop();
    await link.disconnect();
    _notify();
  }

  /// ゴールまでの計画経路（表示用）。到達不能なら null。
  List<Cell>? planTo(Cell goalCell) {
    final robot = robotCell;
    if (robot == null) return null;
    if (!grid.inBounds(goalCell.r, goalCell.c)) return null;
    final gi = grid.index(goalCell.r, goalCell.c);
    if (grid.logOdds[gi] == 0 || !grid.isPlanFree(gi)) return null;
    final mask = grid.planMask(forcedFree: robot);
    return findPath(
      gridW: params.gridW,
      gridH: params.gridH,
      isFree: (i) => mask[i] == 1,
      start: robot,
      goal: goalCell,
    );
  }

  void selectGoal(Cell goalCell) {
    goal = goalCell;
    path = planTo(goalCell);
    status = path == null
        ? 'その場所は既知の空間内に経路がありません（未探索・障害物・到達不能）'
        : '経路を表示しました（${path!.length} セル）。「ゴールへ移動」で走行します';
    _notify();
  }

  Future<void> runExploration() async {
    if (latestPose == null) {
      status = 'ARCore のポーズが得られていません（トラッキングを確認）';
      _notify();
      return;
    }
    try {
      final result = await driver.explore(
        onFrame: (_, __) => _notify(),
      );
      explored = true;
      status = '探索終了: ${result.endReason} / カバー率 '
          '${(result.coverage * 100).toStringAsFixed(1)}% / '
          '${result.steps} ステップ。地図をタップするとゴール指定できます';
    } catch (e) {
      status = '探索エラー: $e';
    }
    _notify();
  }

  Future<void> goToGoal() async {
    final g = goal;
    if (g == null) return;
    try {
      final result = await driver.goTo(
        goal: g,
        onFrame: (_, __) => _notify(),
      );
      status = result.success
          ? 'ゴールに到達しました（${result.steps} ステップ, '
              '${result.pathLengthM.toStringAsFixed(2)} m, 再計画 ${result.replans} 回）'
          : '到達できませんでした: ${result.failureReason}';
      path = null;
    } catch (e) {
      status = '移動エラー: $e';
    }
    _notify();
  }

  Future<void> emergencyStop() async {
    await driver.stop();
    status = '停止指令を送りました';
    _notify();
  }

  Future<void> jog(String ch) async {
    await link.sendJog(ch);
    _notify();
  }

  void dispose() {
    _watchdog?.cancel();
    unawaited(KeepAwake.setEnabled(false));
    _notifyTimer?.cancel();
    for (final s in _subs) {
      s.cancel();
    }
    _updates.close();
  }
}
