import 'dart:async';
import 'dart:math' as math;

import 'package:phone_robot_core/phone_robot_core.dart';

import '../platform/depth_source.dart';
import '../platform/keep_awake.dart';
import '../platform/pose_source.dart';
import '../platform/run_storage.dart';
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
  /// 標準の部屋スケール（6.0 m × 5.0 m、セル 5 cm）。
  static const MappingParams roomParams = MappingParams();

  /// 大学ワンフロア・廊下スケール（24.0 m × 24.0 m、セル 10 cm）。
  static const MappingParams floorParams = MappingParams(
    cellSize: 0.10,
    gridW: 240,
    gridH: 240,
    targetCoverage: 0.80,
  );

  /// ストリーム無音とみなすまでの時間。ドライバ側のゲート
  /// （ポーズ 500 ms / スキャン 600 ms）より長く、こちらは
  /// 「セッションが死んだ」判定と復帰導線のための閾値。
  static const Duration stallThreshold = Duration(seconds: 3);

  final MappingParams params;
  late final OccupancyGrid grid = OccupancyGrid(params);
  final MethodChannelPoseSource poseSource = MethodChannelPoseSource();
  late final MethodChannelDepthSource depthSource =
      MethodChannelDepthSource(params: params);
  final MethodChannelEsp32Link link = MethodChannelEsp32Link();
  final RunStorage storage;
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

  RunRecorder? _recorder;
  bool get isRecording => _recorder != null;
  int get recordedRowCount => _recorder?.rowCount ?? 0;

  /// 記録終了時に Google ドライブへ自動送信／共有画面を開くか。
  bool autoDriveSync = true;
  String driveWebhookUrl = '';
  String? lastSavedRunId;
  String lastSaveStatus = '未保存';

  DateTime? _sensorsStartedAt;
  Timer? _watchdog;

  final _updates = StreamController<void>.broadcast();
  Stream<void> get updates => _updates.stream;

  final List<StreamSubscription<dynamic>> _subs = [];
  Timer? _notifyTimer;

  RealSession({MappingParams? customParams, RunStorage? customStorage})
      : params = customParams ?? roomParams,
        storage = customStorage ?? MethodChannelRunStorage() {
    driver.onStatus = (s) {
      status = s;
      _notify();
    };
    link.onCommandSent = (drive, steer) {
      _recorder?.recordCommand(
        drive,
        steer,
        pose: latestPose,
        trackingState: trackingState,
        latestScan: latestScan,
      );
    };
  }

  Pose? get latestPose => poseFeed.latest;

  Scan? get latestScan => feed.latest;

  double? get hudTargetHeading => driver.hudTargetHeading;

  double? get hudTargetDistM => driver.hudTargetDistM;

  /// これまでに観測・記録したマッピング済み面積 [m²]。
  double get mappedAreaM2 => grid.knownCount * params.cellSize * params.cellSize;

  /// 現在の走行速度（正規化値 0.10..1.0）と、実車へ送る PWM 値（152..255）。
  double get speedNorm => driver.cruiseDrive;
  int get currentDrivePwm => SerialCommandScale.driveOf(driver.cruiseDrive);

  void setSpeedNorm(double value) {
    driver.cruiseDrive = value.clamp(0.10, 1.0);
    _notify();
  }

  /// ステアリング出力の左右極性反転設定。
  bool get invertSteer => driver.invertSteer;

  void setInvertSteer(bool value) {
    driver.invertSteer = value;
    _notify();
  }

  String get trackingState => poseFeed.trackingState;

  /// ARCore の相対座標（開始位置 (0, 0)）をグリッド中央基準の座標へ変換する。
  Pose toGridPose(Pose raw) => Pose(
        raw.x + params.roomW / 2.0,
        raw.y + params.roomH / 2.0,
        raw.heading,
      );

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
      poseFeed.addPose(toGridPose(pose));
      _notify();
    }));
    _subs.add(poseSource.trackingStates.listen((s) {
      poseFeed.setTrackingState(s);
      _notify();
    }));
    _subs.add(depthSource
        .scans(() => latestPose ?? toGridPose(Pose(0, 0, math.pi / 2)))
        .listen(_onIncomingScan));
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

  void _onIncomingScan(Scan scan) {
    feed.add(scan);
    if (_recorder != null) {
      if (!driver.isBusy) {
        grid.applyScan(scan);
      }
      _recorder!.recordScan(scan, trackingState: trackingState);
    }
  }

  /// テスト用: プラットフォームチャネルを経由せずに深度スキャンを投入する。
  void recordScanForTest(Scan scan) => _onIncomingScan(scan);

  void startRecording({String mode = 'manual'}) {
    _recorder = RunRecorder(mode: mode);
    _notify();
  }

  Future<SaveRunResult?> stopAndSaveRecording({
    bool openDriveSheet = true,
  }) async {
    final rec = _recorder;
    if (rec == null) return null;
    _recorder = null;

    final bundle = rec.buildBundle(
      grid: grid,
      params: params,
      speedNorm: speedNorm,
      currentDrivePwm: currentDrivePwm,
      invertSteer: invertSteer,
      robotCell: robotCell,
    );
    lastSaveStatus = '保存中...';
    _notify();

    final res = await storage.saveRunBundle(
      bundle,
      autoDriveSync: autoDriveSync,
      openDriveSheet: openDriveSheet,
    );
    lastSavedRunId = bundle.runId;
    lastSaveStatus = switch (res.driveAction) {
      'webhook_sent' => res.ok ? 'ドライブ自動転送済' : '端末保存済 (転送失敗)',
      'intent_opened' => res.ok ? '端末保存＋ドライブ起動' : '端末保存済',
      'local_only' => res.ok ? '端末のみ保存' : '保存エラー',
      _ => '保存処理完了',
    };
    _notify();
    return res;
  }

  Future<void> toggleManualRecording() async {
    if (isRecording) {
      await stopAndSaveRecording();
    } else {
      startRecording(mode: 'manual');
      status = '手動走行の記録を開始しました（停止時に保存＋ドライブ連携）';
      _notify();
    }
  }

  Future<bool> shareLatestToDrive() async {
    final ok = await storage.shareRunToDrive(runId: lastSavedRunId);
    lastSaveStatus = ok ? 'ドライブ共有を起動' : '共有失敗';
    _notify();
    return ok;
  }

  void setAutoDriveSync(bool enabled) {
    autoDriveSync = enabled;
    _notify();
  }

  Future<void> updateDriveWebhookUrl(String url) async {
    final trimmed = url.trim();
    await storage.setWebhookUrl(trimmed);
    driveWebhookUrl = trimmed;
    _notify();
  }

  /// 画面表示時や USB 接続時に、センサー開始と USB シリアル自動接続を行う。
  Future<void> autoSetup() async {
    try {
      driveWebhookUrl = await storage.getWebhookUrl();
    } catch (_) {
      // ネイティブチャネル未実装環境では無視
    }
    try {
      if (!sensorsOn) {
        await startSensors();
      }
    } catch (_) {
      // ネイティブチャネル未実装環境では無視
    }
    try {
      if (linkState != Esp32LinkState.connected) {
        final devices = await link.listDevices();
        if (devices.isNotEmpty) {
          await connectSerial();
        }
      }
    } catch (_) {
      // ネイティブチャネル未実装環境では無視
    }
    _notify();
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
    final startedHere = !isRecording;
    if (startedHere) startRecording(mode: 'explore');
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
    } finally {
      if (startedHere && isRecording) {
        await stopAndSaveRecording();
      }
    }
    _notify();
  }

  /// VFH 自律走行で未探索エリア・フロンティアを埋め、フロアマップを作成する。
  Future<void> runAutoCruise() async {
    if (latestPose == null) {
      status = 'ARCore のポーズが得られていません（トラッキングを確認）';
      _notify();
      return;
    }
    if (linkState != Esp32LinkState.connected) {
      status = 'ESP32 が接続されていません（USB 接続を確認）';
      _notify();
      return;
    }
    final startedHere = !isRecording;
    if (startedHere) startRecording(mode: 'vfh_cruise');
    try {
      await driver.cruiseVfh(
        onFrame: (_, __) => _notify(),
      );
      explored = true;
      status = 'フロアマップ作成終了: マッピング面積 '
          '${mappedAreaM2.toStringAsFixed(1)} m²（カバー率 '
          '${(grid.coverage * 100).toStringAsFixed(1)}%）';
    } catch (e) {
      status = 'フロアマップ作成エラー: $e';
    } finally {
      if (startedHere && isRecording) {
        await stopAndSaveRecording();
      }
    }
    _notify();
  }

  Future<void> goToGoal() async {
    final g = goal;
    if (g == null) return;
    final startedHere = !isRecording;
    if (startedHere) startRecording(mode: 'go_to_goal');
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
    } finally {
      if (startedHere && isRecording) {
        await stopAndSaveRecording();
      }
    }
    _notify();
  }

  Future<void> emergencyStop() async {
    final wasBusy = driver.isBusy;
    await driver.stop();
    status = '停止指令を送りました';
    if (!wasBusy && isRecording) {
      await stopAndSaveRecording();
    }
    _notify();
  }

  Future<void> jog(String ch) async {
    final pwm = currentDrivePwm;
    final leftSteer = invertSteer ? 255 : -255;
    final rightSteer = invertSteer ? -255 : 255;
    switch (ch) {
      case 's':
        await link.sendJog('s');
      case 'f':
        await link.sendDriveSteer(pwm, 0);
      case 'b':
        await link.sendDriveSteer(-pwm, 0);
      case 'l':
        await link.sendDriveSteer(pwm, leftSteer);
      case 'r':
        await link.sendDriveSteer(pwm, rightSteer);
      default:
        await link.sendJog(ch);
    }
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
