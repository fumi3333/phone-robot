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
  final messenger =
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;

  final serialCalls = <MethodCall>[];
  final systemCalls = <MethodCall>[];

  setUp(() {
    serialCalls.clear();
    systemCalls.clear();
    messenger.setMockMethodCallHandler(serialChannel, (call) async {
      serialCalls.add(call);
      return true;
    });
    messenger.setMockMethodCallHandler(systemChannel, (call) async {
      systemCalls.add(call);
      return null;
    });
  });

  tearDown(() {
    messenger.setMockMethodCallHandler(serialChannel, null);
    messenger.setMockMethodCallHandler(systemChannel, null);
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
}
