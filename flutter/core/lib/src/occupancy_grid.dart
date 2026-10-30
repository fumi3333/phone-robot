import 'dart:math' as math;
import 'dart:typed_data';

import 'params.dart';
import 'pose.dart';
import 'scan.dart';

/// 占有格子地図（SPEC §2.1）。
///
/// 各セルは「障害物である確率」の対数オッズを保持する。
/// 0.0 = 未観測、正 = 障害物らしい、負 = 空間らしい。
class OccupancyGrid {
  final MappingParams params;

  /// ログオッズ本体。添字は row * gridW + col（=[y][x]）。
  final Float32List logOdds;

  OccupancyGrid(this.params) : logOdds = Float32List(params.cellCount);

  int index(int r, int c) => r * params.gridW + c;

  bool inBounds(int r, int c) =>
      r >= 0 && r < params.gridH && c >= 0 && c < params.gridW;

  /// 一度でも更新されたセルか。
  bool isKnown(int i) => logOdds[i] != 0;

  /// 計画用「走行可能」か（SPEC §3.1: 分類より厳しいしきい値）。
  /// logOdds == 0（未探索）は 0 > planFreeL なので自動的に含まれない。
  bool isPlanFree(int i) => logOdds[i] <= params.planFreeL;

  int get knownCount {
    var n = 0;
    for (final v in logOdds) {
      if (v != 0) n++;
    }
    return n;
  }

  /// カバー率 = 更新されたセル数 / 全セル数。
  /// 箱の内部など観測不能なセルを含むため上限は 100% にならない。
  double get coverage => knownCount / logOdds.length;

  /// スキャン1回をログオッズへ反映する（SPEC §3.1）。
  ///
  /// - 各レイについて、終端の [MappingParams.freeMargin] 手前までの
  ///   セルへ L_FREE を加算する（同一スキャン内で重複加算しない）。
  /// - 命中したレイの終端セルへ L_OCC を加算する。
  ///   同一セルが free と hit の両方に該当したら hit を優先する。
  /// - 全セルを [-L_CLAMP, +L_CLAMP] にクランプする。
  void applyScan(Scan scan) {
    final p = params;
    final offsets = rayOffsets(p);
    final free = <int>{};
    final hit = <int>{};
    final bx = scan.pose.x;
    final by = scan.pose.y;
    final bh = scan.pose.heading;

    for (var i = 0; i < scan.ranges.length; i++) {
      final rMeas = scan.ranges[i];
      final ang = bh + offsets[i];
      final ca = math.cos(ang);
      final sa = math.sin(ang);
      final hasHit = !rMeas.isInfinite;
      final limit = hasHit ? math.min(rMeas, p.maxRange) : p.maxRange;

      // free サンプル（終端手前で止める: SPEC §6 の表面誤判定対策）
      var freeEnd = limit - p.freeMargin;
      if (freeEnd < p.rayDs) freeEnd = p.rayDs;
      for (var s = p.rayDs; s < freeEnd; s += p.cellSize) {
        var cc = ((bx + ca * s) / p.cellSize).floor();
        var rr = ((by + sa * s) / p.cellSize).floor();
        if (cc < 0) cc = 0;
        if (cc > p.gridW - 1) cc = p.gridW - 1;
        if (rr < 0) rr = 0;
        if (rr > p.gridH - 1) rr = p.gridH - 1;
        free.add(rr * p.gridW + cc);
      }

      if (hasHit) {
        final ec = ((bx + ca * rMeas) / p.cellSize).floor();
        final er = ((by + sa * rMeas) / p.cellSize).floor();
        if (er >= 0 && er < p.gridH && ec >= 0 && ec < p.gridW) {
          hit.add(er * p.gridW + ec);
        }
      }
    }

    free.removeAll(hit);
    for (final i in free) {
      logOdds[i] += p.lFree;
    }
    for (final i in hit) {
      logOdds[i] += p.lOcc;
    }
    for (var i = 0; i < logOdds.length; i++) {
      final v = logOdds[i];
      if (v > p.lClamp) {
        logOdds[i] = p.lClamp;
      } else if (v < -p.lClamp) {
        logOdds[i] = -p.lClamp;
      }
    }
  }

  /// 計画用マスク（1 = 走行可能）を返す。
  /// [forcedFree] を渡すと、そのセル（現在地）を強制的に走行可能にする
  /// （参照実装と同じ扱い）。
  Uint8List planMask({Cell? forcedFree}) {
    final mask = Uint8List(params.cellCount);
    for (var i = 0; i < mask.length; i++) {
      if (isPlanFree(i)) mask[i] = 1;
    }
    if (forcedFree != null) {
      mask[index(forcedFree.r, forcedFree.c)] = 1;
    }
    return mask;
  }

  /// フロンティア検出（SPEC §3.2）。
  /// 「走行可能（[mask]）」かつ4近傍のいずれかが未探索のセル。
  List<Cell> frontierCells(Uint8List mask) {
    final w = params.gridW;
    final h = params.gridH;
    final result = <Cell>[];
    for (var r = 0; r < h; r++) {
      for (var c = 0; c < w; c++) {
        final i = r * w + c;
        if (mask[i] != 1) continue;
        var adjacentUnknown = false;
        if (r > 0 && logOdds[i - w] == 0) adjacentUnknown = true;
        if (r < h - 1 && logOdds[i + w] == 0) adjacentUnknown = true;
        if (c > 0 && logOdds[i - 1] == 0) adjacentUnknown = true;
        if (c < w - 1 && logOdds[i + 1] == 0) adjacentUnknown = true;
        if (adjacentUnknown) result.add((r: r, c: c));
      }
    }
    return result;
  }
}
