import 'dart:math' as math;
import 'dart:typed_data';

import 'explorer.dart';
import 'occupancy_grid.dart';
import 'params.dart';
import 'pose.dart';
import 'scan.dart';

/// シミュレーション専用の世界モデル（参照実装 sim_mapping.py と同一条件）。
/// 実機では使わない。真値ジオメトリ＋前方扇形レイキャスト＋自己位置誤差。

/// 障害物（箱）: (x0, y0, x1, y1) [m]、中身の詰まった矩形。
const simBoxes = <(double, double, double, double)>[
  (1.50, 1.20, 2.10, 1.70),
  (3.60, 3.00, 4.30, 3.50),
  (4.60, 0.70, 5.10, 1.50),
];

/// 真値グリッドを作る（1 = 占有: 外周の壁＋箱、0 = 空間）。
Uint8List buildGroundTruth(MappingParams p) {
  final gt = Uint8List(p.cellCount);
  final w = p.gridW;
  final h = p.gridH;
  for (var c = 0; c < w; c++) {
    gt[c] = 1;
    gt[(h - 1) * w + c] = 1;
  }
  for (var r = 0; r < h; r++) {
    gt[r * w] = 1;
    gt[r * w + (w - 1)] = 1;
  }
  for (final (x0, y0, x1, y1) in simBoxes) {
    final c0 = (x0 / p.cellSize).floor();
    final c1 = (x1 / p.cellSize).ceil();
    final r0 = (y0 / p.cellSize).floor();
    final r1 = (y1 / p.cellSize).ceil();
    for (var r = r0; r < r1; r++) {
      for (var c = c0; c < c1; c++) {
        gt[r * w + c] = 1;
      }
    }
  }
  return gt;
}

/// シード付きガウス乱数（Box-Muller）。参照実装の numpy RNG と
/// 数列は一致しないが、統計的には同等の条件になる。
class GaussianRandom {
  final math.Random _rng;
  double? _spare;

  GaussianRandom(int seed) : _rng = math.Random(seed);

  double next() {
    final spare = _spare;
    if (spare != null) {
      _spare = null;
      return spare;
    }
    var u = 0.0;
    while (u == 0.0) {
      u = _rng.nextDouble();
    }
    final v = _rng.nextDouble();
    final mag = math.sqrt(-2.0 * math.log(u));
    _spare = mag * math.sin(2 * math.pi * v);
    return mag * math.cos(2 * math.pi * v);
  }
}

/// シミュレーション内の車両。真値の位置を持ち、スキャンは真値から
/// レイキャストし、believed pose（真値＋誤差）で地図へ登録する。
///
/// ドリフトモードでは believed pose の誤差がランダムウォークで累積する
/// （補正なしの自己位置推定の模倣。DRIFT_MEMO.md 参照）。
class SimRobot implements MotionHooks {
  final MappingParams params;
  final Uint8List groundTruth;
  final OccupancyGrid grid;
  final GaussianRandom rng;
  final bool drift;

  /// 現在の真値セル。
  Cell cell;

  double _heading;
  double _driftX = 0;
  double _driftY = 0;
  double _driftH = 0;

  SimRobot({
    required this.params,
    required this.groundTruth,
    required this.grid,
    required this.rng,
    required this.drift,
    required Cell start,
    double startHeading = 0,
  })  : cell = start,
        _heading = startHeading;

  double get trueX => (cell.c + 0.5) * params.cellSize;
  double get trueY => (cell.r + 0.5) * params.cellSize;

  @override
  double get heading => _heading;

  /// 真値ポーズから扇形レイを飛ばし、距離ノイズ付きの計測値を返す。
  Float32List castRanges(double physHeading) {
    final p = params;
    final offsets = rayOffsets(p);
    final ranges = Float32List(p.nRays);
    for (var i = 0; i < p.nRays; i++) {
      final ang = physHeading + offsets[i];
      final ca = math.cos(ang);
      final sa = math.sin(ang);
      var measured = double.infinity;
      for (var d = p.rayDs; d < p.maxRange; d += p.rayDs) {
        final x = trueX + ca * d;
        final y = trueY + sa * d;
        var cc = (x / p.cellSize).floor();
        var rr = (y / p.cellSize).floor();
        if (cc < 0) cc = 0;
        if (cc > p.gridW - 1) cc = p.gridW - 1;
        if (rr < 0) rr = 0;
        if (rr > p.gridH - 1) rr = p.gridH - 1;
        if (groundTruth[rr * p.gridW + cc] != 0) {
          var m = d + rng.next() * p.rangeNoise;
          if (m < p.minRange) m = p.minRange;
          if (m > p.maxRange) m = p.maxRange;
          measured = m;
          break;
        }
      }
      ranges[i] = measured;
    }
    return ranges;
  }

  @override
  void scan(double physHeading) {
    final ranges = castRanges(physHeading);
    final double bx;
    final double by;
    final double bh;
    if (drift) {
      bx = trueX + _driftX;
      by = trueY + _driftY;
      bh = physHeading + _driftH;
    } else {
      bx = trueX + rng.next() * params.posePosNoise;
      by = trueY + rng.next() * params.posePosNoise;
      bh = physHeading + rng.next() * params.poseHeadNoise;
    }
    grid.applyScan(Scan(Pose(bx, by, bh), ranges));
  }

  @override
  bool moveTo(Cell next) {
    final ni = next.r * params.gridW + next.c;
    if (groundTruth[ni] != 0) {
      // 物理フェイルセーフ: 占有セルへは進入できない。拒否した移動は
      // 強い占有証拠として地図に記録する（参照実装と同じ）。
      if (grid.logOdds[ni] < 2.0) grid.logOdds[ni] = 2.0;
      return false;
    }
    _heading = math.atan2(
      (next.r - cell.r).toDouble(),
      (next.c - cell.c).toDouble(),
    );
    cell = next;
    if (drift) {
      _driftX += rng.next() * params.driftPosStep;
      _driftY += rng.next() * params.driftPosStep;
      _driftH += rng.next() * params.driftHeadStep;
    }
    return true;
  }
}
