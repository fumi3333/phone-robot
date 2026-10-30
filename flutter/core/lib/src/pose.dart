import 'params.dart';

/// グリッドのセル座標。row = y 方向（北が +）、col = x 方向（東が +）。
typedef Cell = ({int r, int c});

/// 自己位置（SPEC §2.2）。単位は m / rad、座標系は SPEC §1。
///
/// 実機では ARCore の推定位置がこの「信じている位置（believed pose）」に
/// 対応する（真値は実機には存在しない）。
class Pose {
  double x;
  double y;
  double heading;

  Pose(this.x, this.y, this.heading);

  Pose copy() => Pose(x, y, heading);

  @override
  String toString() =>
      'Pose(x=${x.toStringAsFixed(2)}, y=${y.toStringAsFixed(2)}, '
      'heading=${heading.toStringAsFixed(2)})';
}

/// 世界座標 [m] → セル。
Cell cellOf(MappingParams p, double x, double y) =>
    (r: (y / p.cellSize).floor(), c: (x / p.cellSize).floor());

/// セル → その中心の世界座標 [m]。
({double x, double y}) cellCenter(MappingParams p, Cell cell) => (
      x: (cell.c + 0.5) * p.cellSize,
      y: (cell.r + 0.5) * p.cellSize,
    );
