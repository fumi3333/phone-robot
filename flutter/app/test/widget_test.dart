import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:phone_robot_app/real/real_session.dart';
import 'package:phone_robot_app/real_page.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

void main() {
  const serialChannel = MethodChannel('phone_robot/serial');
  const systemChannel = MethodChannel('phone_robot/system');
  const storageChannel = MethodChannel('phone_robot/storage');
  const arcoreChannel = MethodChannel('phone_robot/arcore');

  setUp(() {
    final messenger =
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;
    messenger.setMockMethodCallHandler(serialChannel, (_) async => true);
    messenger.setMockMethodCallHandler(systemChannel, (_) async => null);
    messenger.setMockMethodCallHandler(storageChannel, (call) async {
      if (call.method == 'getWebhookUrl') return '';
      return true;
    });
    messenger.setMockMethodCallHandler(arcoreChannel, (_) async => null);
  });

  tearDown(() {
    final messenger =
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger;
    messenger.setMockMethodCallHandler(serialChannel, null);
    messenger.setMockMethodCallHandler(systemChannel, null);
    messenger.setMockMethodCallHandler(storageChannel, null);
    messenger.setMockMethodCallHandler(arcoreChannel, null);
  });

  testWidgets('2モード切替・障害物アラート・ボタンとフリック入力切替が動作する', (WidgetTester tester) async {
    final session = RealSession();
    try {
      await tester.pumpWidget(MaterialApp(home: RealPage(session: session)));

      // 初期モード: マップ測定（ESP32 ボタンは表示されない）
      expect(find.text('マップ測定'), findsOneWidget);
      expect(find.text('ラジコン'), findsOneWidget);
      expect(find.byKey(const Key('sensor_power_btn')), findsOneWidget);
      expect(find.byKey(const Key('recenter_map_btn')), findsOneWidget);
      expect(find.byKey(const Key('record_toggle_btn')), findsOneWidget);
      expect(find.byKey(const Key('drive_send_btn')), findsOneWidget);
      expect(find.byKey(const Key('reset_map_btn')), findsOneWidget);
      expect(find.byKey(const Key('esp32_connect_btn')), findsNothing);
      expect(find.text('カメラで前方を計測中…'), findsOneWidget);

      // センサー有効化して前方 0.25m の障害物スキャンを投入
      session.sensorsOn = true;
      final pose = session.toGridPose(Pose(0, 0, math.pi / 2));
      session.poseFeed.addPose(pose);
      session.recordScanForTest(
        Scan(pose, Float32List.fromList(List<double>.filled(21, 0.25))),
      );
      await tester.pump(const Duration(milliseconds: 60));

      expect(find.textContaining('障害物を感知'), findsOneWidget);

      // ② ラジコン操縦モードへ切替
      await tester.tap(find.byKey(const Key('mode_rc')));
      await tester.pumpAndSettle();

      expect(find.byKey(const Key('esp32_connect_btn')), findsOneWidget);
      expect(find.text('▲ 前'), findsOneWidget);
      expect(find.text('◀ 左'), findsOneWidget);
      expect(find.text('右 ▶'), findsOneWidget);
      expect(find.text('▼ 後'), findsOneWidget);
      expect(find.byKey(const Key('flick_pad')), findsNothing);

      // ボタン ⇔ フリック切替
      await tester.tap(find.byKey(const Key('input_toggle_flick')));
      await tester.pumpAndSettle();

      expect(find.byKey(const Key('flick_pad')), findsOneWidget);
      expect(find.text('▲ 前'), findsNothing);

      // 設定シートからデモマップを表示すると地図がメイン表示に切り替わり距離バーも更新される
      await tester.tap(find.byKey(const Key('settings_btn')));
      await tester.pumpAndSettle();
      expect(find.byKey(const Key('demo_preview_btn')), findsOneWidget);
      await tester.tap(find.byKey(const Key('demo_preview_btn')));
      await tester.pumpAndSettle();

      expect(find.byKey(const Key('obstacle_dist_bar')), findsOneWidget);
      expect(find.text('カメラ (タップで拡大)'), findsOneWidget);
      await tester.tap(find.byKey(const Key('mode_walk')));
      await tester.pumpAndSettle();
      expect(find.textContaining('デモマップ表示中'), findsOneWidget);
    } finally {
      await tester.pump(const Duration(milliseconds: 60));
      session.dispose();
    }
  });
}
