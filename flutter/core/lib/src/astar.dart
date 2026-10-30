import 'dart:math' as math;
import 'dart:typed_data';

import 'pose.dart';

class _HeapItem {
  final double f;
  final double g;
  final int r;
  final int c;
  const _HeapItem(this.f, this.g, this.r, this.c);
}

/// (f, g, r, c) の順で比較する二分ヒープ（参照実装の heapq と同じ順序）。
class _OpenHeap {
  final List<_HeapItem> _items = [];

  bool get isNotEmpty => _items.isNotEmpty;

  static bool _less(_HeapItem a, _HeapItem b) {
    if (a.f != b.f) return a.f < b.f;
    if (a.g != b.g) return a.g < b.g;
    if (a.r != b.r) return a.r < b.r;
    return a.c < b.c;
  }

  void push(_HeapItem item) {
    _items.add(item);
    var i = _items.length - 1;
    while (i > 0) {
      final parent = (i - 1) >> 1;
      if (!_less(_items[i], _items[parent])) break;
      final tmp = _items[parent];
      _items[parent] = _items[i];
      _items[i] = tmp;
      i = parent;
    }
  }

  _HeapItem pop() {
    final top = _items.first;
    final last = _items.removeLast();
    if (_items.isNotEmpty) {
      _items[0] = last;
      var i = 0;
      while (true) {
        final l = 2 * i + 1;
        final rgt = l + 1;
        var smallest = i;
        if (l < _items.length && _less(_items[l], _items[smallest])) {
          smallest = l;
        }
        if (rgt < _items.length && _less(_items[rgt], _items[smallest])) {
          smallest = rgt;
        }
        if (smallest == i) break;
        final tmp = _items[smallest];
        _items[smallest] = _items[i];
        _items[i] = tmp;
        i = smallest;
      }
    }
    return top;
  }
}

/// A* 経路計画（SPEC §3.3）。
///
/// [isFree] が真のセルだけを通る。8近傍移動、斜めはコスト √2、
/// 斜め移動は両脇の直交セルが走行可能な場合のみ許可（角抜け防止）。
/// ヒューリスティクスは octile 距離。
/// 到達不能なら null を返す。到達したら start を含むセル列を返す。
List<Cell>? findPath({
  required int gridW,
  required int gridH,
  required bool Function(int index) isFree,
  required Cell start,
  required Cell goal,
}) {
  if (start == goal) return [start];
  final n = gridW * gridH;
  final gScore = Float64List(n)..fillRange(0, n, double.infinity);
  final parent = Int32List(n)..fillRange(0, n, -1);

  double heuristic(int r, int c) {
    final dx = (c - goal.c).abs();
    final dy = (r - goal.r).abs();
    final mx = math.max(dx, dy);
    final mn = math.min(dx, dy);
    return mx + (math.sqrt2 - 1.0) * mn;
  }

  final startIdx = start.r * gridW + start.c;
  gScore[startIdx] = 0.0;
  final heap = _OpenHeap()
    ..push(_HeapItem(heuristic(start.r, start.c), 0.0, start.r, start.c));

  const dirs = <(int, int, double)>[
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt2),
    (-1, 1, math.sqrt2),
    (1, -1, math.sqrt2),
    (1, 1, math.sqrt2),
  ];

  while (heap.isNotEmpty) {
    final cur = heap.pop();
    final r = cur.r;
    final c = cur.c;
    final g = cur.g;
    if (r == goal.r && c == goal.c) {
      final path = <Cell>[];
      var idx = r * gridW + c;
      while (idx != -1) {
        path.add((r: idx ~/ gridW, c: idx % gridW));
        idx = parent[idx];
      }
      return path.reversed.toList();
    }
    if (g > gScore[r * gridW + c] + 1e-6) continue; // 古いエントリ
    for (final (dr, dc, cost) in dirs) {
      final nr = r + dr;
      final nc = c + dc;
      if (nr < 0 || nr >= gridH || nc < 0 || nc >= gridW) continue;
      final nIdx = nr * gridW + nc;
      if (!isFree(nIdx)) continue;
      if (dr != 0 && dc != 0) {
        // 角抜け防止: 両脇の直交セルが走行可能なときだけ斜めを許可
        if (!isFree(r * gridW + (c + dc)) || !isFree((r + dr) * gridW + c)) {
          continue;
        }
      }
      final ng = g + cost;
      if (ng < gScore[nIdx] - 1e-6) {
        gScore[nIdx] = ng;
        parent[nIdx] = r * gridW + c;
        heap.push(_HeapItem(ng + heuristic(nr, nc), ng, nr, nc));
      }
    }
  }
  return null;
}
