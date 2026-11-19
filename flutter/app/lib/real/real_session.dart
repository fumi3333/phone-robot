import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:phone_robot_core/phone_robot_core.dart';

import '../platform/depth_source.dart';
import '../platform/keep_awake.dart';
import '../platform/pose_source.dart';
import '../platform/run_storage.dart';
import '../platform/serial_link.dart';
import 'real_driver.dart';

/// 前方障害物の危険度レベル（直感バナー表示用）。
enum ObstacleSeverity { unknown, clear, caution, danger }

/// 現在の前方カメラ深度から判定した障害物アラート情報。
class ObstacleAlert {
  final ObstacleSeverity severity;
  final double? frontMinM;
  final double? leftClearM;
  final double? rightClearM;
  final String message;

  const ObstacleAlert({
    required this.severity,
    this.frontMinM,
    this.leftClearM,
    this.rightClearM,
    required this.message,
  });
}

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
  /// 標準の部屋スケール（12.0 m × 12.0 m、セル 5 cm）。
  static const MappingParams roomParams = MappingParams(
    cellSize: 0.05,
    gridW: 240,
    gridH: 240,
    targetCoverage: 0.80,
  );

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
  late final MethodChannelDepthSource depthSource = MethodChannelDepthSource(
    params: params,
  );
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
  bool isDemoMode = false;
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

  /// 前方安全ブレーキのしきい値（メートル）とヒステリシス設定。
  static const double frontStopDistM = 0.25;
  static const double frontResumeDistM = 0.35;
  static const double frontSlowDistM = 0.55;

  /// 記録終了時に Google ドライブへ自動送信／共有画面を開くか。
  bool autoDriveSync = true;
  bool recordMp4 = false;
  bool frontBrakeEnabled = true;
  bool frontBrakeActive = false;
  double? vehicleBatteryV;
  String driveWebhookUrl = '';
  String? lastSavedRunId;
  String lastSaveStatus = '未保存';
  int? cameraTextureId;

  DateTime? _sensorsStartedAt;
  DateTime? _lastCloseWallAt;
  Pose? _lastRawPose;
  double _originOffsetX = 0.0;
  double _originOffsetY = 0.0;
  Timer? _watchdog;

  final _updates = StreamController<void>.broadcast();
  Stream<void> get updates => _updates.stream;

  final List<StreamSubscription<dynamic>> _subs = [];
  StreamSubscription<Esp32LinkState>? _linkSub;
  StreamSubscription<String>? _rxSub;
  String _rxBuf = '';
  Timer? _notifyTimer;

  String? _activeManualDir;
  bool _preSteerReady = false;
  int _manualHoldEpoch = 0;

  RealSession({MappingParams? customParams, RunStorage? customStorage})
    : params = customParams ?? roomParams,
      storage = customStorage ?? MethodChannelRunStorage() {
    driver.onStatus = (s) {
      status = s;
      _notify();
    };
    _linkSub = link.states.listen((s) {
      linkState = s;
      if (s == Esp32LinkState.disconnected) {
        vehicleBatteryV = null;
      }
      _notify();
    });
    _rxSub = link.received.listen(ingestSerialTelemetry);
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
  double get mappedAreaM2 =>
      grid.knownCount * params.cellSize * params.cellSize;

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

  void setFrontBrakeEnabled(bool value) {
    frontBrakeEnabled = value;
    if (!value) {
      frontBrakeActive = false;
    }
    _notify();
  }

  void setRecordMp4(bool value) {
    recordMp4 = value;
    _notify();
  }

  String get trackingState => poseFeed.trackingState;

  /// 現在の前方カメラ 21 本深度レイから直感的な障害物アラートを算出する。
  ObstacleAlert get obstacleAlert {
    final scan = latestScan;
    if ((!sensorsOn && !isDemoMode) || scan == null || scan.ranges.isEmpty) {
      return const ObstacleAlert(
        severity: ObstacleSeverity.unknown,
        message: 'カメラで前方を計測中…',
      );
    }
    final ranges = scan.ranges;
    final n = ranges.length;
    final cStart = (n * 0.33).floor();
    final cEnd = (n * 0.67).ceil().clamp(cStart, n - 1);
    final frontMin = RunRecorder.frontMinDepthM(scan);

    double? avgMeasuredSide(int from, int to) {
      if (from > to) return null;
      var sum = 0.0;
      var cnt = 0;
      for (var i = from; i <= to; i++) {
        final r = ranges[i];
        if (r.isFinite && r > 0) {
          sum += r.clamp(0.05, params.maxRange);
          cnt++;
        }
      }
      return cnt > 0 ? sum / cnt : null;
    }

    // ArCoreManager 側: i=0 が右 (-30°)、i=n-1 が左 (+30°)
    final rightClear = avgMeasuredSide(0, math.max(0, cStart - 1));
    final leftClear = avgMeasuredSide(math.min(n - 1, cEnd + 1), n - 1);

    if (frontMin == null) {
      return ObstacleAlert(
        severity: ObstacleSeverity.unknown,
        leftClearM: leftClear,
        rightClearM: rightClear,
        message: 'カメラで前方を計測中…',
      );
    }
    if (frontMin <= 0.25) {
      return ObstacleAlert(
        severity: ObstacleSeverity.danger,
        frontMinM: frontMin,
        leftClearM: leftClear,
        rightClearM: rightClear,
        message: '🚨 目の前に障害物を感知！ (${frontMin.toStringAsFixed(2)} m)',
      );
    }
    if (frontMin < 0.55) {
      final hasSide = leftClear != null || rightClear != null;
      final openSide = (leftClear ?? 0) >= (rightClear ?? 0) ? '左' : '右';
      return ObstacleAlert(
        severity: ObstacleSeverity.caution,
        frontMinM: frontMin,
        leftClearM: leftClear,
        rightClearM: rightClear,
        message: hasSide
            ? '⚠️ 前方に障害物あり (${frontMin.toStringAsFixed(2)} m) — $openSide側が空いています'
            : '⚠️ 前方に障害物あり (${frontMin.toStringAsFixed(2)} m)',
      );
    }
    return ObstacleAlert(
      severity: ObstacleSeverity.clear,
      frontMinM: frontMin,
      leftClearM: leftClear,
      rightClearM: rightClear,
      message: '✅ 前方クリア (${frontMin.toStringAsFixed(2)} m)',
    );
  }

  /// ARCore の相対座標（開始位置）をグリッド中央基準の座標へ変換する。
  Pose toGridPose(Pose raw) => Pose(
    raw.x + params.roomW / 2.0 - _originOffsetX,
    raw.y + params.roomH / 2.0 - _originOffsetY,
    raw.heading,
  );

  void _notify() {
    // 高頻度のポーズ更新での再描画を 25fps (40ms) 上限で間引く
    _notifyTimer ??= Timer(const Duration(milliseconds: 40), () {
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
    isDemoMode = false;
    await poseSource.start();
    sensorsOn = true;
    _sensorsStartedAt = DateTime.now();
    sensorsStalled = false;
    sensorHealth = '正常';
    _subs.add(
      poseSource.poses.listen((pose) {
        _lastRawPose = pose;
        poseFeed.addPose(toGridPose(pose));
        _notify();
      }),
    );
    _subs.add(
      poseSource.trackingStates.listen((s) {
        poseFeed.setTrackingState(s);
        if (sensorsOn && (s == 'PAUSED' || s == 'STOPPED')) {
          status =
              'ARCore ロスト ($s): カメラを床の模様やドア・家具に向けてゆっくり動かしてください';
        }
        _notify();
      }),
    );
    _subs.add(
      depthSource
          .scans(() => latestPose ?? toGridPose(Pose(0, 0, math.pi / 2)))
          .listen(_onIncomingScan),
    );
    cameraTextureId = await poseSource.getCameraTextureId();
    // R4: 実機モード中は画面を消さない＋無音ウォッチドッグを開始。
    unawaited(KeepAwake.setEnabled(true));
    _watchdog?.cancel();
    _watchdog = Timer.periodic(
      const Duration(milliseconds: 500),
      (_) => evaluateSensorHealth(),
    );
    status = 'センサー稼働中: カメラをゆっくり動かして周囲を映してください';
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
    cameraTextureId = null;
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
      status =
          'センサーの配信が止まりました（ポーズ無音）。'
          '安全のため停止しました。'
          '「センサー停止」→「センサー開始」で再開してください';
      unawaited(driver.stop());
    } else if (!stalledNow && sensorsStalled) {
      sensorsStalled = false;
      status =
          'センサーの配信が復帰しました。'
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
    status =
        'アプリが背面に移ったため走行を停止しました。'
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
      status =
          'アプリに復帰しました。センサーの配信を確認してから'
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

  void ingestSerialTelemetry(String chunk) {
    _rxBuf += chunk;
    while (true) {
      final idx = _rxBuf.indexOf('\n');
      if (idx < 0) break;
      final line = _rxBuf.substring(0, idx).trim();
      _rxBuf = _rxBuf.substring(idx + 1);
      if (line.startsWith('v,')) {
        final mv = int.tryParse(line.substring(2).trim());
        if (mv != null && mv > 0 && mv < 20000) {
          vehicleBatteryV = mv / 1000.0;
          _notify();
        }
      } else if (line.startsWith('v') && line.length > 1) {
        final val = double.tryParse(line.substring(1).trim());
        if (val != null && val > 0) {
          if (val >= 100.0 && val < 20000.0) {
            vehicleBatteryV = val / 1000.0;
            _notify();
          } else if (val < 20.0) {
            vehicleBatteryV = val;
            _notify();
          }
        }
      }
    }
  }

  void _onIncomingScan(Scan scan) {
    feed.add(scan);
    if (!driver.isBusy) {
      grid.applyScan(scan);
    }
    if (_recorder != null) {
      _recorder!.recordScan(scan, trackingState: trackingState);
    }
    final frontMin = RunRecorder.frontMinDepthM(scan);
    if (frontMin != null && frontMin < 0.70) {
      _lastCloseWallAt = DateTime.now();
    } else if (frontMin != null && frontMin >= frontResumeDistM) {
      _lastCloseWallAt = null;
    }
    if (frontBrakeEnabled &&
        (_activeManualDir == 'f' ||
            _activeManualDir == 'l' ||
            _activeManualDir == 'r')) {
      final lostCloseWall =
          frontMin == null &&
          _lastCloseWallAt != null &&
          DateTime.now().difference(_lastCloseWallAt!) <=
              const Duration(milliseconds: 1500);
      final shouldStop =
          lostCloseWall ||
          (frontMin != null &&
              (frontMin < frontStopDistM ||
                  (frontBrakeActive && frontMin < frontResumeDistM)));
      if (shouldStop) {
        if (!frontBrakeActive) {
          frontBrakeActive = true;
          _manualHoldEpoch++;
          unawaited(link.sendJog('s'));
        }
        status = lostCloseWall || frontMin == null
            ? '前方ブレーキ作動: 至近距離のため自動停止（後退は可能）'
            : '前方ブレーキ作動: 壁まで ${frontMin.toStringAsFixed(2)} m（後退は可能）';
      } else if (frontBrakeActive &&
          frontMin != null &&
          frontMin >= frontResumeDistM) {
        frontBrakeActive = false;
      }
    }
    _notify();
  }

  /// テスト用: プラットフォームチャネルを経由せずに深度スキャンを投入する。
  void recordScanForTest(Scan scan) => _onIncomingScan(scan);

  void resetMap() {
    if (driver.isBusy) return;
    isDemoMode = false;
    final raw = _lastRawPose;
    if (raw != null) {
      // マップ消去時に現在位置をグリッド中央へ戻し、枠外切れを防ぐ
      _originOffsetX = raw.x;
      _originOffsetY = raw.y;
      poseFeed.addPose(toGridPose(raw));
    } else {
      _originOffsetX = 0.0;
      _originOffsetY = 0.0;
    }
    grid.logOdds.fillRange(0, grid.logOdds.length, 0.0);
    goal = null;
    path = null;
    explored = false;
    status = 'マップをリセットしました（現在位置を中央に配置）';
    _notify();
  }

  /// カメラ・実車なしでも新しいマップ描画・連続壁補間・距離バナーを確認できるデモを生成する。
  void loadDemoPreviewMap() {
    if (driver.isBusy) return;
    isDemoMode = true;
    grid.logOdds.fillRange(0, grid.logOdds.length, 0.0);
    goal = null;
    path = null;

    // 部屋外周壁＋左の棚＋正面やや手前のテーブル（現在地から左・正面・右の違いが出る配置）
    const boxes = <({double minX, double maxX, double minY, double maxY})>[
      (minX: -1.90, maxX: -1.75, minY: -1.80, maxY: 1.90),
      (minX: 1.75, maxX: 1.90, minY: -1.80, maxY: 1.90),
      (minX: -1.90, maxX: 1.90, minY: 1.75, maxY: 1.90),
      (minX: -1.90, maxX: 1.90, minY: -1.90, maxY: -1.75),
      (minX: -0.35, maxX: 0.35, minY: 0.72, maxY: 1.15),
      (minX: -1.45, maxX: -0.90, minY: 0.10, maxY: 0.85),
    ];

    final cx = params.roomW / 2.0;
    final cy = params.roomH / 2.0;
    final offsets = rayOffsets(params);

    Scan castDemoScan(double relX, double relY, double heading) {
      final pose = Pose(cx + relX, cy + relY, heading);
      final ranges = Float32List(params.nRays);
      for (var i = 0; i < params.nRays; i++) {
        final ang = heading + offsets[i];
        final ca = math.cos(ang);
        final sa = math.sin(ang);
        var hit = double.nan;
        for (var d = 0.15; d <= params.maxRange; d += 0.03) {
          final px = relX + ca * d;
          final py = relY + sa * d;
          var blocked = false;
          for (final b in boxes) {
            if (px >= b.minX && px <= b.maxX && py >= b.minY && py <= b.maxY) {
              blocked = true;
              break;
            }
          }
          if (blocked) {
            hit = d;
            break;
          }
        }
        ranges[i] = hit;
      }
      return Scan(pose, ranges);
    }

    const demoSteps = <({double x, double y, double h})>[
      (x: 0.0, y: -0.90, h: 1.57),
      (x: 0.0, y: -0.90, h: 2.35),
      (x: 0.0, y: -0.90, h: 0.78),
      (x: 0.0, y: -0.45, h: 1.95),
      (x: 0.0, y: -0.45, h: 1.15),
      (x: 0.0, y: 0.0, h: 2.25),
      (x: 0.0, y: 0.0, h: 0.85),
      (x: 0.0, y: 0.0, h: math.pi / 2),
    ];

    poseFeed.setTrackingState('DEMO');
    sensorHealth = 'デモ表示';
    Scan? lastScan;
    for (final st in demoSteps) {
      final scan = castDemoScan(st.x, st.y, st.h);
      poseFeed.addPose(scan.pose);
      grid.applyScan(scan);
      lastScan = scan;
    }
    if (lastScan != null) {
      feed.add(lastScan);
    }
    explored = true;
    status = 'デモマップ表示中: 赤=壁・障害物 / 白=通路 / 青扇形=カメラ向き';
    _notify();
  }

  void startRecording({String mode = 'manual'}) {
    final rec = RunRecorder(mode: mode);
    _recorder = rec;
    if (recordMp4) {
      unawaited(poseSource.startMp4Recording(rec.runId));
    }
    _notify();
  }

  Future<SaveRunResult?> stopAndSaveRecording({
    bool openDriveSheet = false,
  }) async {
    final rec = _recorder;
    if (rec == null) return null;
    _recorder = null;

    final mp4Path = await poseSource.stopMp4Recording();
    final hasMp4 = mp4Path != null && mp4Path.isNotEmpty;

    final bundle = rec.buildBundle(
      grid: grid,
      params: params,
      speedNorm: speedNorm,
      currentDrivePwm: currentDrivePwm,
      invertSteer: invertSteer,
      robotCell: robotCell,
      latestPose: latestPose,
      latestScan: latestScan,
      hasMp4Video: hasMp4,
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
      'local_only' => res.ok ? '端末保存済み' : '保存エラー',
      _ => '保存処理完了',
    };
    _notify();
    return res;
  }

  Future<void> toggleManualRecording() async {
    if (isRecording) {
      await stopAndSaveRecording(openDriveSheet: false);
      status = '記録を端末に保存しました（必要に応じて「ドライブへ送る」で送信できます）';
      _notify();
    } else {
      startRecording(mode: 'manual');
      status = 'カメラ動画＋マップ記録を開始しました（停止で端末に保存）';
      _notify();
    }
  }

  Future<bool> shareLatestToDrive() async {
    final ok = await storage.shareRunToDrive(runId: lastSavedRunId);
    lastSaveStatus = ok ? 'ドライブ共有を起動' : '共有失敗';
    _notify();
    return ok;
  }

  /// 記録中なら終了・保存してドライブへ送り、記録済みなら直近の記録をドライブへ送る。
  /// 未記録でも現在のマップをスナップショット保存してそのまま送信できる。
  Future<bool> saveAndShareToDrive() async {
    if (isRecording) {
      final res = await stopAndSaveRecording(openDriveSheet: true);
      return res?.ok ?? false;
    }
    if (lastSavedRunId == null) {
      startRecording(mode: 'snapshot');
      final res = await stopAndSaveRecording(openDriveSheet: true);
      return res?.ok ?? false;
    }
    return shareLatestToDrive();
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
      final result = await driver.explore(onFrame: (_, __) => _notify());
      explored = true;
      status =
          '探索終了: ${result.endReason} / カバー率 '
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
      await driver.cruiseVfh(onFrame: (_, __) => _notify());
      explored = true;
      status =
          'フロアマップ作成終了: マッピング面積 '
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
      final result = await driver.goTo(goal: g, onFrame: (_, __) => _notify());
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
    _manualHoldEpoch++;
    _activeManualDir = null;
    _preSteerReady = false;
    final wasBusy = driver.isBusy;
    await driver.stop();
    status = '停止指令を送りました';
    if (!wasBusy && isRecording) {
      await stopAndSaveRecording(openDriveSheet: false);
    }
    _notify();
  }

  int _evaluateFrontDrivePwm(int requestedPwm) {
    if (!frontBrakeEnabled) {
      frontBrakeActive = false;
      return requestedPwm;
    }
    final scan = latestScan;
    final frontMin = scan == null ? null : RunRecorder.frontMinDepthM(scan);
    if (frontMin == null) {
      final lostCloseWall =
          _lastCloseWallAt != null &&
          DateTime.now().difference(_lastCloseWallAt!) <=
              const Duration(milliseconds: 1500);
      if (lostCloseWall) {
        frontBrakeActive = true;
        return 0;
      }
      frontBrakeActive = false;
      return requestedPwm;
    }
    if (frontMin < frontStopDistM ||
        (frontBrakeActive && frontMin < frontResumeDistM)) {
      frontBrakeActive = true;
      return 0;
    }
    frontBrakeActive = false;
    if (frontMin < frontSlowDistM) {
      final ratio =
          ((frontMin - frontStopDistM) / (frontSlowDistM - frontStopDistM))
              .clamp(0.0, 1.0);
      final minPwm = SerialCommandScale.minEffectiveDrivePwm;
      if (requestedPwm <= minPwm) return requestedPwm;
      return (minPwm + ratio * (requestedPwm - minPwm))
          .round()
          .clamp(minPwm, requestedPwm);
    }
    return requestedPwm;
  }

  Future<void> jog(String ch) async {
    final pwm = currentDrivePwm;
    final leftSteer = invertSteer ? 255 : -255;
    final rightSteer = invertSteer ? -255 : 255;
    switch (ch) {
      case 's':
        _manualHoldEpoch++;
        _activeManualDir = null;
        _preSteerReady = false;
        await link.sendJog('s');
      case 'f':
        _manualHoldEpoch++;
        _activeManualDir = 'f';
        _preSteerReady = false;
        final safePwm = _evaluateFrontDrivePwm(pwm);
        if (safePwm == 0) {
          await link.sendJog('s');
        } else {
          await link.sendDriveSteer(safePwm, 0);
        }
      case 'b':
        _manualHoldEpoch++;
        _activeManualDir = 'b';
        _preSteerReady = false;
        await link.sendDriveSteer(-pwm, 0);
      case 'l':
      case 'r':
        final steer = ch == 'l' ? leftSteer : rightSteer;
        final turnPwm = _evaluateFrontDrivePwm(pwm.clamp(152, 178));
        if (turnPwm == 0) {
          _manualHoldEpoch++;
          _activeManualDir = ch;
          _preSteerReady = false;
          await link.sendJog('s');
        } else if (_activeManualDir != ch) {
          final epoch = ++_manualHoldEpoch;
          _activeManualDir = ch;
          _preSteerReady = false;
          // バネ戻し対策: まず停止状態のまま前輪をフルロックへ先当て舵する
          await link.sendDriveSteer(0, steer);
          _notify();
          await Future<void>.delayed(const Duration(milliseconds: 150));
          if (epoch != _manualHoldEpoch) return;
          final recheckedPwm = _evaluateFrontDrivePwm(pwm.clamp(152, 178));
          if (recheckedPwm == 0) {
            await link.sendJog('s');
            return;
          }
          _preSteerReady = true;
          await link.sendDriveSteer(recheckedPwm, steer);
        } else if (_preSteerReady) {
          await link.sendDriveSteer(turnPwm, steer);
        }
      default:
        await link.sendJog(ch);
    }
    _notify();
  }

  void dispose() {
    _manualHoldEpoch++;
    _watchdog?.cancel();
    unawaited(KeepAwake.setEnabled(false));
    _notifyTimer?.cancel();
    _linkSub?.cancel();
    _rxSub?.cancel();
    for (final s in _subs) {
      s.cancel();
    }
    _updates.close();
  }
}
