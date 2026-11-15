import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:phone_robot_app/platform/keep_awake.dart';
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
  final messenger =
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;

  final serialCalls = <MethodCall>[];
  final systemCalls = <MethodCall>[];
  final storageCalls = <MethodCall>[];

  setUp(() {
    serialCalls.clear();
    systemCalls.clear();
    storageCalls.clear();
    messenger.setMockMethodCallHandler(serialChannel, (call) async {
      serialCalls.add(call);
      return true;
    });
    messenger.setMockMethodCallHandler(systemChannel, (call) async {
      systemCalls.add(call);
      return null;
    });
    messenger.setMockMethodCallHandler(storageChannel, (call) async {
      storageCalls.add(call);
      if (call.method == 'saveRunBundle') {
        return {
          'savedDir': '/sdcard/Android/data/test/files/runs/run_test',
          'driveAction': 'intent_opened',
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
  });

  test('R4: ポーズ無音を検出して stall 扱いにし、停止指令を送る', () async {
    final session = RealSession();
    session.sensorsOn = true;
    session.poseFeed.addPose(Pose(1.0, 1.0, 0));
    session.poseFeed.lastPoseAt =
        DateTime.now().subtract(const Duration(seconds: 10));

    session.evaluateSensorHealth();

    expect(session.sensorsStalled, isTrue);
    expect(session.sensorHealth, contains('ポーズ無音'));
    expect(session.status, contains('再開してください'));
    // ドライバ停止 → 's' がシリアルへ送られる
    await Future<void>.delayed(const Duration(milliseconds: 100));
    expect(
      serialCalls.any((c) =>
          c.method == 'send' &&
          (c.arguments as Map<dynamic, dynamic>)['text'] == 's'),
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
    expect((systemCalls.single.arguments as Map<dynamic, dynamic>)['on'],
        isTrue);

    await KeepAwake.setEnabled(false);
    expect(systemCalls, hasLength(2));
    expect((systemCalls.last.arguments as Map<dynamic, dynamic>)['on'],
        isFalse);
  });

  test('原点オフセット: ARCore (0, 0) をグリッド中央へ変換する', () {
    final roomSession = RealSession();
    final roomCenter = roomSession.toGridPose(Pose(0, 0, 1.57));
    expect(roomCenter.x, closeTo(3.0, 1e-6));
    expect(roomCenter.y, closeTo(2.5, 1e-6));
    expect(roomCenter.heading, closeTo(1.57, 1e-6));
    roomSession.poseFeed.addPose(roomCenter);
    expect(roomSession.robotCell, (r: 50, c: 60));
    roomSession.dispose();

    final floorSession = RealSession(customParams: RealSession.floorParams);
    final floorCenter = floorSession.toGridPose(Pose(0, 0, 0.0));
    expect(floorCenter.x, closeTo(12.0, 1e-6));
    expect(floorCenter.y, closeTo(12.0, 1e-6));
    floorSession.poseFeed.addPose(floorCenter);
    expect(floorSession.robotCell, (r: 120, c: 120));
    floorSession.dispose();
  });

  test('速度スライダー設定と操舵反転がジョグ指令の PWM と極性に反映される', () async {
    final session = RealSession();
    String? sentText() {
      final sent = serialCalls
          .where((c) => c.method == 'send')
          .map((c) => (c.arguments as Map<dynamic, dynamic>)['text'] as String)
          .toList();
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

    await session.jog('l');
    expect(sentText(), 'c,255,-255\n');

    session.setInvertSteer(true);
    expect(session.invertSteer, isTrue);
    await session.jog('l');
    expect(sentText(), 'c,255,255\n');

    await session.jog('s');
    expect(sentText(), 's');

    session.dispose();
  });

  test('走行記録で CSV・PNG・JSON・MD を生成し storage チャネルへ渡す', () async {
    final session = RealSession();
    session.startRecording(mode: 'manual');
    expect(session.isRecording, isTrue);

    final startPose = session.toGridPose(Pose(0, 0, 1.57));
    session.poseFeed.addPose(startPose);
    final scan = Scan(
      startPose,
      Float32List.fromList(List<double>.filled(21, 1.8)),
    );
    session.recordScanForTest(scan);

    session.setSpeedNorm(0.20);
    await session.jog('f');
    await session.jog('s');

    final res = await session.stopAndSaveRecording();
    expect(session.isRecording, isFalse);
    expect(res, isNotNull);
    expect(res!.driveAction, 'intent_opened');
    expect(res.ok, isTrue);

    final saveCall = storageCalls.lastWhere((c) => c.method == 'saveRunBundle');
    final args = saveCall.arguments as Map<dynamic, dynamic>;
    final runId = args['runId'] as String;
    final files = args['files'] as Map<dynamic, dynamic>;

    final csv = files['driving_log_$runId.csv'] as String;
    expect(csv, contains('timestamp_ms,elapsed_ms,event,mode,drive_pwm'));
    expect(csv, contains(',scan,manual,'));
    expect(csv, contains(',cmd,manual,163,0,'));
    expect(csv, contains(',cmd,manual,0,0,'));

    final png = files['floor_map_$runId.png'] as Uint8List;
    expect(png.sublist(0, 8), [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]);

    final jsonMap =
        jsonDecode(files['floor_map_$runId.json'] as String) as Map<String, dynamic>;
    expect(jsonMap['runId'], runId);
    expect(jsonMap['mode'], 'manual');
    expect((jsonMap['logOdds'] as List<dynamic>), isNotEmpty);

    final md = files['run_summary_$runId.md'] as String;
    expect(md, contains('# 走行記録レポート ($runId)'));

    session.dispose();
  });
}
