import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:phone_robot_app/platform/keep_awake.dart';
import 'package:phone_robot_app/platform/serial_link.dart';
import 'package:phone_robot_app/real/real_session.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// R4 のセッション側対策（無音検出・復帰手順・画面常時点灯チャネル）の
/// 単体テスト。プラットフォームチャネルはモックで受ける。
/// 画面消灯そのものの挙動は実機でしか確認できない（README 参照）。
void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  const serialChannel = MethodChannel('phone_robot/serial');
  const systemChannel = MethodChannel('phone_robot/system');
  const storageChannel = MethodChannel('phone_robot/storage');
  const arcoreChannel = MethodChannel('phone_robot/arcore');
  final messenger =
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;

  final serialCalls = <MethodCall>[];
  final systemCalls = <MethodCall>[];
  final storageCalls = <MethodCall>[];
  final arcoreCalls = <MethodCall>[];

  setUp(() {
    serialCalls.clear();
    systemCalls.clear();
    storageCalls.clear();
    arcoreCalls.clear();
    messenger.setMockMethodCallHandler(serialChannel, (call) async {
      serialCalls.add(call);
      return true;
    });
    messenger.setMockMethodCallHandler(systemChannel, (call) async {
      systemCalls.add(call);
      return null;
    });
    messenger.setMockMethodCallHandler(arcoreChannel, (call) async {
      arcoreCalls.add(call);
      if (call.method == 'startMp4Recording') {
        final runId = (call.arguments as Map<dynamic, dynamic>)['runId'];
        return '/sdcard/Android/data/test/files/runs/run_$runId/camera_recording_$runId.mp4';
      }
      if (call.method == 'stopMp4Recording') {
        return '/sdcard/Android/data/test/files/runs/run_test/camera_recording_test.mp4';
      }
      if (call.method == 'getCameraTextureId') {
        return 42;
      }
      return null;
    });
    messenger.setMockMethodCallHandler(storageChannel, (call) async {
      storageCalls.add(call);
      if (call.method == 'saveRunBundle') {
        final args = call.arguments as Map<dynamic, dynamic>;
        final openSheet = args['openDriveSheet'] == true;
        return {
          'savedDir': '/sdcard/Android/data/test/files/runs/run_test',
          'driveAction': openSheet ? 'intent_opened' : 'local_only',
          'ok': true,
        };
      }
      if (call.method == 'getWebhookUrl') {
        return '';
      }
      return true;
    });
  });

  tearDown(() {
    messenger.setMockMethodCallHandler(serialChannel, null);
    messenger.setMockMethodCallHandler(systemChannel, null);
    messenger.setMockMethodCallHandler(storageChannel, null);
    messenger.setMockMethodCallHandler(arcoreChannel, null);
  });

  test('R4: ポーズ無音を検出して stall 扱いにし、停止指令を送る', () async {
    final session = RealSession();
    session.sensorsOn = true;
    session.poseFeed.addPose(Pose(1.0, 1.0, 0));
    session.poseFeed.lastPoseAt = DateTime.now().subtract(
      const Duration(seconds: 10),
    );

    session.evaluateSensorHealth();

    expect(session.sensorsStalled, isTrue);
    expect(session.sensorHealth, contains('ポーズ無音'));
    expect(session.status, contains('再開してください'));
    // ドライバ停止 → 's' がシリアルへ送られる
    await Future<void>.delayed(const Duration(milliseconds: 100));
    expect(
      serialCalls.any(
        (c) =>
            c.method == 'send' &&
            (c.arguments as Map<dynamic, dynamic>)['text'] == 's',
      ),
      isTrue,
    );

    // 配信が戻れば stall は解除される（走行は自動再開しない）
    session.poseFeed.addPose(Pose(1.0, 1.0, 0));
    session.feed.add(Scan(Pose(1.0, 1.0, 0), Float32List(21)));
    session.evaluateSensorHealth();
    expect(session.sensorsStalled, isFalse);
    expect(session.sensorHealth, '正常');
    expect(session.status, contains('自動では再開しません'));
    session.dispose();
  });

  test('R4: 復帰時は自動再開せず、再計画の案内になる', () {
    final session = RealSession();
    session.sensorsOn = true;
    session.poseFeed.addPose(Pose(0.5, 0.5, 0));
    session.feed.add(Scan(Pose(0.5, 0.5, 0), Float32List(21)));

    session.handleAppResumed();

    expect(session.status, contains('自動では再開しません'));
    expect(session.status, contains('再計画'));
    session.dispose();
  });

  test('R4: 背面遷移（走行していない時）は何も壊さない', () async {
    final session = RealSession();
    final before = session.status;
    await session.handleAppPaused();
    expect(session.status, before);
    session.dispose();
  });

  test('R4: KeepAwake が setKeepScreenOn を正しく送る', () async {
    await KeepAwake.setEnabled(true);
    expect(systemCalls, hasLength(1));
    expect(systemCalls.single.method, 'setKeepScreenOn');
    expect(
      (systemCalls.single.arguments as Map<dynamic, dynamic>)['on'],
      isTrue,
    );

    await KeepAwake.setEnabled(false);
    expect(systemCalls, hasLength(2));
    expect(
      (systemCalls.last.arguments as Map<dynamic, dynamic>)['on'],
      isFalse,
    );
  });

  test('原点オフセット: ARCore (0, 0) をグリッド中央へ変換する', () {
    final roomSession = RealSession();
    final roomCenter = roomSession.toGridPose(Pose(0, 0, 1.57));
    expect(roomCenter.x, closeTo(6.0, 1e-6));
    expect(roomCenter.y, closeTo(6.0, 1e-6));
    expect(roomCenter.heading, closeTo(1.57, 1e-6));
    roomSession.poseFeed.addPose(roomCenter);
    expect(roomSession.robotCell, (r: 120, c: 120));
    roomSession.dispose();

    final floorSession = RealSession(customParams: RealSession.floorParams);
    final floorCenter = floorSession.toGridPose(Pose(0, 0, 0.0));
    expect(floorCenter.x, closeTo(12.0, 1e-6));
    expect(floorCenter.y, closeTo(12.0, 1e-6));
    floorSession.poseFeed.addPose(floorCenter);
    expect(floorSession.robotCell, (r: 120, c: 120));
    floorSession.dispose();
  });

  test('速度スライダー・先当て舵・旋回PWM上限・操舵反転がジョグ指令に反映される', () async {
    final session = RealSession();
    List<String> sentList() => serialCalls
        .where((c) => c.method == 'send')
        .map((c) => (c.arguments as Map<dynamic, dynamic>)['text'] as String)
        .toList();
    String? sentText() {
      final sent = sentList();
      return sent.isEmpty ? null : sent.last;
    }

    session.setSpeedNorm(0.20);
    expect(session.speedNorm, closeTo(0.20, 1e-6));
    expect(session.currentDrivePwm, 163);
    await session.jog('f');
    expect(sentText(), 'c,163,0\n');

    session.setSpeedNorm(1.0);
    expect(session.currentDrivePwm, 255);
    await session.jog('f');
    expect(sentText(), 'c,255,0\n');

    // 左旋回はまず駆動0で先当て舵 (-255) し、その後 turnPwm 上限 178 で曲がる
    final beforeLeft = sentList().length;
    await session.jog('l');
    final leftSeq = sentList().sublist(beforeLeft);
    expect(leftSeq, ['c,0,-255\n', 'c,178,-255\n']);

    await session.jog('s');
    expect(sentText(), 's');

    session.setInvertSteer(true);
    expect(session.invertSteer, isTrue);
    final beforeInv = sentList().length;
    await session.jog('l');
    final invSeq = sentList().sublist(beforeInv);
    expect(invSeq, ['c,0,255\n', 'c,178,255\n']);

    await session.jog('s');
    expect(sentText(), 's');

    session.dispose();
  });

  test('記録未開始でもスキャン到着で地図更新＆障害物アラートが更新される', () {
    final session = RealSession();
    session.sensorsOn = true;
    final p = session.toGridPose(Pose(0, 0, math.pi / 2));
    session.poseFeed.addPose(p);

    expect(session.grid.knownCount, 0);
    session.recordScanForTest(
      Scan(p, Float32List.fromList(List<double>.filled(21, 0.25))),
    );
    expect(session.grid.knownCount, greaterThan(0));
    expect(session.obstacleAlert.severity, ObstacleSeverity.danger);
    expect(session.obstacleAlert.message, contains('障害物を感知'));

    session.resetMap();
    expect(session.grid.knownCount, 0);
    session.dispose();
  });

  test('走行記録でコマンド効果CSV・JSON・MDを生成し停止時は自動でドライブを開かない', () async {
    final session = RealSession();
    session.setRecordMp4(true);
    session.startRecording(mode: 'manual');
    expect(session.isRecording, isTrue);

    final startPose = session.toGridPose(Pose(0, 0, 0));
    session.poseFeed.addPose(startPose);
    final scan1 = Scan(
      startPose,
      Float32List.fromList(List<double>.filled(21, 1.8)),
    );
    session.recordScanForTest(scan1);

    session.setSpeedNorm(0.20);
    await session.jog('f');

    // 前進命令後に位置と方位と障害物距離が変化した状態を与える
    final movedPose = session.toGridPose(Pose(0.25, 0.0, 0.15));
    session.poseFeed.addPose(movedPose);
    final scan2 = Scan(
      movedPose,
      Float32List.fromList(List<double>.filled(21, 1.55)),
    );
    session.recordScanForTest(scan2);

    await session.jog('s');

    final res = await session.stopAndSaveRecording();
    expect(session.isRecording, isFalse);
    expect(res, isNotNull);
    // 停止時は勝手にドライブを開かない
    expect(res!.driveAction, 'local_only');
    expect(res.ok, isTrue);

    final saveCall = storageCalls.lastWhere((c) => c.method == 'saveRunBundle');
    final args = saveCall.arguments as Map<dynamic, dynamic>;
    expect(args['openDriveSheet'], isFalse);
    final runId = args['runId'] as String;
    final files = args['files'] as Map<dynamic, dynamic>;

    final csv = files['driving_log_$runId.csv'] as String;
    expect(csv, contains('timestamp_ms,elapsed_ms,event,mode,drive_pwm'));
    expect(csv, contains(',scan,manual,'));
    expect(csv, contains(',cmd,manual,163,0,'));
    expect(csv, contains(',cmd,manual,0,0,'));

    final effectsCsv = files['command_effects_$runId.csv'] as String;
    expect(effectsCsv, contains('action_label,start_x_m'));
    expect(effectsCsv, contains(',163,0,前進,'));
    expect(effectsCsv, contains(',0,0,停止,'));

    final png = files['floor_map_$runId.png'] as Uint8List;
    expect(png.sublist(0, 8), [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]);

    final jsonMap =
        jsonDecode(files['floor_map_$runId.json'] as String)
            as Map<String, dynamic>;
    expect(jsonMap['runId'], runId);
    expect(jsonMap['mode'], 'manual');
    expect(jsonMap['hasMp4Video'], isTrue);
    final effects = jsonMap['commandEffects'] as List<dynamic>;
    expect(effects, hasLength(2));
    final forwardSeg = effects.first as Map<String, dynamic>;
    expect(forwardSeg['actionLabel'], '前進');
    expect(
      (forwardSeg['distanceMovedM'] as num).toDouble(),
      closeTo(0.25, 0.01),
    );
    expect((jsonMap['logOdds'] as List<dynamic>), isNotEmpty);

    final md = files['run_summary_$runId.md'] as String;
    expect(md, contains('# 走行記録レポート ($runId)'));
    expect(md, contains('## コマンド別の車体・センサー変化一覧（命令→結果）'));

    // 明示的にドライブ送信を呼ぶと shareRunToDrive が起動する
    final shared = await session.saveAndShareToDrive();
    expect(shared, isTrue);
    expect(storageCalls.any((c) => c.method == 'shareRunToDrive'), isTrue);

    session.dispose();
  });

  test('前方安全ブレーキ: 90cm以内で減速、50cm未満で停止し後退は許可、65cm以上で再開', () async {
    final session = RealSession();
    session.setSpeedNorm(1.0); // PWM 255
    expect(session.frontBrakeEnabled, isTrue);
    expect(session.recordMp4, isFalse);

    Scan scanAt(double frontM) {
      final ranges = Float32List(21)..fillRange(0, 21, 2.5);
      for (var i = 7; i <= 13; i++) {
        ranges[i] = frontM;
      }
      return Scan(Pose(3.0, 2.5, math.pi / 2), ranges);
    }

    // 1. 0.70m（減速帯 0.50..0.90m）: フル 255 より減速した値で前進
    session.recordScanForTest(scanAt(0.70));
    await session.jog('f');
    expect(session.frontBrakeActive, isFalse);
    final slowCmd = serialCalls.last.arguments['text'] as String;
    expect(slowCmd.startsWith('c,'), isTrue);
    final slowPwm = int.parse(slowCmd.split(',')[1]);
    expect(slowPwm, greaterThanOrEqualTo(SerialCommandScale.minEffectiveDrivePwm));
    expect(slowPwm, lessThan(255));

    // 2. 0.40m（50cm未満）: 強制停止 's' が送られ、frontBrakeActive = true
    session.recordScanForTest(scanAt(0.40));
    expect(session.frontBrakeActive, isTrue);
    expect(serialCalls.last.arguments['text'], 's');

    // 3. 0.58m（ヒステリシス帯 0.50..0.65m）: まだ前進は禁止だが後退 'b' は許可
    session.recordScanForTest(scanAt(0.58));
    await session.jog('f');
    expect(session.frontBrakeActive, isTrue);
    expect(serialCalls.last.arguments['text'], 's');

    await session.jog('b');
    expect(serialCalls.last.arguments['text'], 'c,-255,0\n');

    // 4. 0.70m（0.65m以上）: ブレーキ解除され前進再開
    session.recordScanForTest(scanAt(0.70));
    await session.jog('f');
    expect(session.frontBrakeActive, isFalse);
    expect((serialCalls.last.arguments['text'] as String).startsWith('c,'), isTrue);

    session.dispose();
  });

  test('ToF至近距離抜けフェイルセーフ: 直前に0.70m未満の壁があり急にnullになったら安全停止', () async {
    final session = RealSession();
    session.setSpeedNorm(1.0);

    Scan scanAt(double? frontM) {
      final ranges = Float32List(21)
        ..fillRange(0, 21, frontM == null ? double.nan : 2.5);
      if (frontM != null) {
        for (var i = 6; i <= 15; i++) {
          ranges[i] = frontM;
        }
      }
      return Scan(Pose(3.0, 2.5, math.pi / 2), ranges);
    }

    // 0.55m で壁を認識した直後に、至近距離(15cm未満)/黒面で計測不能(null)になった場合
    session.recordScanForTest(scanAt(0.55));
    session.recordScanForTest(scanAt(null));
    await session.jog('f');
    expect(session.frontBrakeActive, isTrue);
    expect(serialCalls.last.arguments['text'], 's');

    session.dispose();
  });

  test('OpenBot互換電圧テレメトリ: v,<mV> と v<V> の両形式の受信で vehicleBatteryV を更新する', () {
    final session = RealSession();
    expect(session.vehicleBatteryV, isNull);

    session.ingestSerialTelemetry('v,78');
    expect(session.vehicleBatteryV, isNull); // 改行前は保留

    session.ingestSerialTelemetry('50\nv,6800\n');
    expect(session.vehicleBatteryV, closeTo(6.8, 1e-6));

    session.ingestSerialTelemetry('v8.42\n');
    expect(session.vehicleBatteryV, closeTo(8.42, 1e-6));

    session.dispose();
  });

  test('車幅コリドー判定: 車幅(±12cm)の外にある物は通り抜け可として前進できる', () async {
    final session = RealSession();
    session.setSpeedNorm(1.0);

    // ±24°〜±30° (i=0..2, 18..20) の外側に 0.30m の障害物があっても横ズレ >= 0.12m なので進める
    final ranges = Float32List(21)..fillRange(0, 21, 2.5);
    for (final i in [0, 1, 2, 18, 19, 20]) {
      ranges[i] = 0.30; // 0.30 * sin(21°..30°) ≈ 0.11..0.15m
    }
    // 外側の 0.30m は sin(24°)=0.122m > 0.12m になる i=0,1,19,20 だけ残す
    ranges[2] = 2.5;
    ranges[18] = 2.5;
    session.recordScanForTest(Scan(Pose(6.0, 6.0, math.pi / 2), ranges));
    await session.jog('f');
    expect(session.frontBrakeActive, isFalse);
    expect(serialCalls.last.arguments['text'], 'c,255,0\n');

    session.dispose();
  });
}
