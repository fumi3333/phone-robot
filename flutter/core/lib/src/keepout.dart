import 'params.dart';
import 'pose.dart';

/// 立ち入り禁止エリア（keep-out zones）。
///
/// 地図上のセル集合を「進入禁止」として保持し、経路計画
/// （[GoalNavigator] / [PatrolRunner]）に反映させるためのデータ構造。
/// 禁止セルは占有セルと同じく計画マスクから除外されるが、地図の
/// ログオッズ自体は書き換えない（禁止は「運用上の制約」であって
/// 「観測結果」ではないため。解除すれば元の地図のまま再び通れる）。
class KeepOutZones {
  /// 禁止セルのグリッド添字集合（row * gridW + col）。
  final Set<int> cells;

  const KeepOutZones(this.cells);

  /// 空の禁止エリア。
  factory KeepOutZones.empty() => KeepOutZones(<int>{});

  /// 世界座標の矩形リスト [m] から作る。各矩形は (x0, y0, x1, y1)。
  /// 矩形に少しでも重なるセルを禁止にする（安全側に広めに取る）。
  factory KeepOutZones.fromWorldRects(
    MappingParams p,
    List<({double x0, double y0, double x1, double y1})> rects,
  ) {
    final cells = <int>{};
    for (final rect in rects) {
      final c0 = (rect.x0 / p.cellSize).floor();
      final c1 = (rect.x1 / p.cellSize).ceil();
      final r0 = (rect.y0 / p.cellSize).floor();
      final r1 = (rect.y1 / p.cellSize).ceil();
      for (var r = r0; r < r1; r++) {
        for (var c = c0; c < c1; c++) {
          if (r < 0 || r >= p.gridH || c < 0 || c >= p.gridW) continue;
          cells.add(r * p.gridW + c);
        }
      }
    }
    return KeepOutZones(cells);
  }

  /// セル座標の集合から作る。
  factory KeepOutZones.fromCells(MappingParams p, Iterable<Cell> cellSet) {
    return KeepOutZones(
        {for (final cell in cellSet) cell.r * p.gridW + cell.c});
  }

  int get size => cells.length;
  bool get isEmpty => cells.isEmpty;

  bool containsIndex(int index) => cells.contains(index);
  bool containsCell(MappingParams p, Cell cell) =>
      cells.contains(cell.r * p.gridW + cell.c);
}
