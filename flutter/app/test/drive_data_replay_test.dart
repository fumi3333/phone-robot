import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:phone_robot_app/real/real_session.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// Google ドライブ実機ログ（20261116_005808 / 20261116_005440）に基づくリプレイテスト。
///
/// 旧仕様では ToF 未計測 (-1.0) を double.infinity 扱いしたため 4.0m の偽自由空間扇形が
/// 描かれていた（5.9秒静止で 30.4% 暴走）。NaN スキップ・隣接壁補間・デモ表示を検証する。
void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

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

  Float32List parseDriveRawRanges(List<double> rawValues) {
    final ranges = Float32List(rawValues.length);
    for (var i = 0; i < rawValues.length; i++) {
      final v = rawValues[i];
      ranges[i] = (v.isNaN || v <= 0) ? double.nan : v;
    }
    return ranges;
  }

  test('実機ログ 20261116_005808: 未計測(-1.0)レイが 4m 先の偽自由空間を作らない', () {
    const params = MappingParams();
    final grid = OccupancyGrid(params);

    // 20261116_005808.csv の実測ポーズと 21 本レイ（先頭 ray[0] が -1.0 未計測）
    final realSamples = <({double x, double y, double h, List<double> raw})>[
      (
        x: 2.1841,
        y: 3.2863,
        h: 1.0305,
        raw: [
          -1.0,
          0.305,
          0.303,
          0.298,
          0.290,
          0.281,
          0.277,
          0.272,
          0.273,
          0.274,
          0.275,
          0.273,
          0.276,
          0.277,
          0.279,
          0.283,
          0.289,
          0.295,
          0.290,
          0.286,
          0.298,
        ],
      ),
      (
        x: 2.1827,
        y: 3.3005,
        h: 0.7986,
        raw: [
          -1.0,
          0.300,
          0.296,
          0.291,
          0.287,
          0.284,
          0.280,
          0.279,
          0.278,
          0.277,
          0.279,
          0.280,
          0.284,
          0.287,
          0.290,
          0.296,
          0.303,
          0.308,
          0.316,
          0.324,
          0.335,
        ],
      ),
      (
        x: 2.1858,
        y: 3.3044,
        h: 0.5983,
        raw: [
          -1.0,
          0.300,
          0.293,
          0.289,
          0.286,
          0.283,
          0.280,
          0.278,
          0.278,
          0.277,
          0.278,
          0.280,
          0.283,
          0.286,
          0.290,
          0.295,
          0.302,
          0.308,
          0.316,
          0.327,
          0.339,
        ],
      ),
      (
        x: 2.1774,
        y: 3.3054,
        h: 0.8861,
        raw: [
          -1.0,
          0.310,
          0.305,
          0.302,
          0.301,
          0.298,
          0.298,
          0.299,
          0.300,
          0.301,
          0.303,
          0.307,
          0.310,
          0.315,
          0.321,
          0.327,
          0.335,
          0.345,
          0.355,
          0.369,
          0.385,
        ],
      ),
      (
        x: 2.1681,
        y: 3.2984,
        h: 0.8206,
        raw: [
          -1.0,
          0.311,
          0.305,
          0.304,
          0.301,
          0.300,
          0.301,
          0.301,
          0.302,
          0.303,
          0.306,
          0.310,
          0.314,
          0.319,
          0.324,
          0.331,
          0.341,
          0.350,
          0.361,
          0.377,
          0.389,
        ],
      ),
    ];

    for (final s in realSamples) {
      grid.applyScan(Scan(Pose(s.x, s.y, s.h), parseDriveRawRanges(s.raw)));
    }

    // 偽の 4m 扇形（30.4% / 3,651 セル）ではなく、0.3m 圏内のみが更新される
    expect(grid.knownCount, greaterThan(10));
    expect(grid.knownCount, lessThan(200));

    // ray[0] (-30°) 方向の 2.5m 先は依然として未探索 (logOdds == 0.0)
    final ray0Angle = realSamples.first.h - math.pi / 6;
    final farCell = cellOf(
      params,
      realSamples.first.x + math.cos(ray0Angle) * 2.5,
      realSamples.first.y + math.sin(ray0Angle) * 2.5,
    );
    expect(grid.logOdds[grid.index(farCell.r, farCell.c)], 0.0);
  });

  test('全レイ未計測は地図を更新せず、double.infinity(シミュレータ)のみ開空間を更新する', () {
    const params = MappingParams();
    final grid = OccupancyGrid(params);
    final pose = Pose(3.0, 2.5, math.pi / 2);

    grid.applyScan(
      Scan(pose, parseDriveRawRanges(List<double>.filled(21, -1.0))),
    );
    expect(grid.knownCount, 0);

    grid.applyScan(
      Scan(
        pose,
        Float32List.fromList(List<double>.filled(21, double.infinity)),
      ),
    );
    expect(grid.knownCount, greaterThan(0));
  });

  test('隣接レイ間の距離差が小さい壁は途切れなく連続セルとして補間される', () {
    const params = MappingParams();
    final grid = OccupancyGrid(params);
    final pose = Pose(3.0, 1.5, math.pi / 2);
    final offsets = rayOffsets(params);

    // 前方 1.80 m にある横一直線の平らな壁を模擬（隣接ヒット間隔は ~0.10m = 2セル）
    final wallRanges = Float32List(params.nRays);
    for (var i = 0; i < params.nRays; i++) {
      wallRanges[i] = 1.80 / math.cos(offsets[i]);
    }
    grid.applyScan(Scan(pose, wallRanges));

    // y = 1.5 + 1.80 = 3.30 m -> row = 66。-0.95m..+0.95m の幅で黒ドットが途切れないこと
    final wallRow = (3.30 / params.cellSize).floor();
    final cLeft = ((3.0 - 0.90) / params.cellSize).floor();
    final cRight = ((3.0 + 0.90) / params.cellSize).floor();
    for (var c = cLeft; c <= cRight; c++) {
      final hasWallNear =
          grid.logOdds[grid.index(wallRow, c)] > 0 ||
          grid.logOdds[grid.index(wallRow - 1, c)] > 0 ||
          grid.logOdds[grid.index(wallRow + 1, c)] > 0;
      expect(hasWallNear, isTrue, reason: 'col=$c の壁が途切れている');
    }
  });

  test('デモマップ読込で壁・自由空間・左正面右アラートが生成されリセットで戻る', () {
    final session = RealSession();
    try {
      session.loadDemoPreviewMap();
      expect(session.isDemoMode, isTrue);
      expect(session.explored, isTrue);
      expect(session.latestPose, isNotNull);
      expect(session.latestScan, isNotNull);
      expect(session.grid.knownCount, greaterThan(200));

      var occ = 0;
      var free = 0;
      for (final v in session.grid.logOdds) {
        if (v > 0) occ++;
        if (v < 0) free++;
      }
      expect(occ, greaterThan(30));
      expect(free, greaterThan(150));

      final alert = session.obstacleAlert;
      expect(alert.severity, isNot(ObstacleSeverity.unknown));
      expect(alert.frontMinM, isNotNull);
      expect(alert.leftClearM, isNotNull);
      expect(alert.rightClearM, isNotNull);

      session.resetMap();
      expect(session.isDemoMode, isFalse);
      expect(session.grid.knownCount, 0);
    } finally {
      session.dispose();
    }
  });
}
