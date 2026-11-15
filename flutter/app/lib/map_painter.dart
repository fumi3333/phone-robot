import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 占有格子地図および直感オーバーレイの描画。
///
/// [followRobot] が true かつ自己位置があるときは現在地を中心に拡大表示（1m/2mリング付き）し、
/// 周囲の障害物（赤）と通路（白）を直感的に見せる。false のときは全体図を表示する。
class MapPainter extends CustomPainter {
  final Float32List logOdds;
  final int gridW;
  final int gridH;
  final Cell robot;
  final Cell? goal;
  final List<Cell>? path;
  final Pose? robotPose;
  final Scan? latestScan;
  final double? targetHeading;
  final double? targetDistM;
  final MappingParams mappingParams;
  final bool followRobot;
  final double viewRadiusM;

  const MapPainter({
    required this.logOdds,
    required this.gridW,
    required this.gridH,
    required this.robot,
    this.goal,
    this.path,
    this.robotPose,
    this.latestScan,
    this.targetHeading,
    this.targetDistM,
    this.mappingParams = const MappingParams(),
    this.followRobot = true,
    this.viewRadiusM = 2.5,
  });

  /// 画面タップ座標を地図グリッドのセルに変換する。範囲外なら null。
  static Cell? cellAtTap({
    required Offset tap,
    required Size size,
    required MappingParams params,
    Pose? robotPose,
    bool followRobot = false,
    double viewRadiusM = 2.5,
  }) {
    if (size.width <= 0 || size.height <= 0) return null;
    final int r;
    final int c;
    if (followRobot && robotPose != null) {
      final pxPerM = math.min(size.width, size.height) / (2.0 * viewRadiusM);
      final wx = robotPose.x + (tap.dx - size.width / 2.0) / pxPerM;
      final wy = robotPose.y - (tap.dy - size.height / 2.0) / pxPerM;
      c = (wx / params.cellSize).floor();
      r = (wy / params.cellSize).floor();
    } else {
      final cw = size.width / params.gridW;
      final ch = size.height / params.gridH;
      c = (tap.dx / cw).floor();
      final rFromTop = (tap.dy / ch).floor();
      r = params.gridH - 1 - rFromTop;
    }
    if (r < 0 || r >= params.gridH || c < 0 || c >= params.gridW) return null;
    return (r: r, c: c);
  }

  @override
  void paint(Canvas canvas, Size size) {
    if (size.width <= 0 || size.height <= 0) return;
    canvas.save();
    canvas.clipRect(Offset.zero & size);

    final rp = robotPose;
    final useFollow = followRobot && rp != null;
    final cs = mappingParams.cellSize;

    final double cw;
    final double ch;
    final double pxPerM;
    late final Offset Function(double wx, double wy) worldToCanvas;
    late final Offset Function(Cell cell) centerOf;

    if (useFollow) {
      pxPerM = math.min(size.width, size.height) / (2.0 * viewRadiusM);
      cw = cs * pxPerM;
      ch = cw;
      final cx = size.width / 2.0;
      final cy = size.height / 2.0;
      worldToCanvas = (wx, wy) =>
          Offset(cx + (wx - rp.x) * pxPerM, cy - (wy - rp.y) * pxPerM);
      centerOf = (cell) =>
          worldToCanvas((cell.c + 0.5) * cs, (cell.r + 0.5) * cs);
    } else {
      cw = size.width / gridW;
      ch = size.height / gridH;
      pxPerM = math.min(cw, ch) / cs;
      worldToCanvas = (wx, wy) =>
          Offset((wx / cs) * cw, (gridH - wy / cs) * ch);
      centerOf = (cell) =>
          Offset((cell.c + 0.5) * cw, (gridH - 1 - cell.r + 0.5) * ch);
    }

    final unknownPaint = Paint()..color = const Color(0xFFCBD5E1);
    final freePaint = Paint()..color = Colors.white;
    final occPaint = Paint()..color = const Color(0xFFEF4444);

    canvas.drawRect(Offset.zero & size, unknownPaint);

    // 可視範囲だけを求め、行単位で連続セルをまとめて描画して負荷を大幅に下げる
    var minR = 0;
    var maxR = gridH - 1;
    var minC = 0;
    var maxC = gridW - 1;
    if (useFollow) {
      final halfW = (size.width / 2.0) / pxPerM;
      final halfH = (size.height / 2.0) / pxPerM;
      minC = ((rp.x - halfW) / cs).floor().clamp(0, gridW - 1);
      maxC = ((rp.x + halfW) / cs).ceil().clamp(0, gridW - 1);
      minR = ((rp.y - halfH) / cs).floor().clamp(0, gridH - 1);
      maxR = ((rp.y + halfH) / cs).ceil().clamp(0, gridH - 1);
    }

    for (var r = minR; r <= maxR; r++) {
      final rowBase = r * gridW;
      final top = useFollow
          ? size.height / 2.0 - ((r + 1) * cs - rp.y) * pxPerM
          : (gridH - 1 - r) * ch;
      var c = minC;
      while (c <= maxC) {
        final v = logOdds[rowBase + c];
        if (v == 0) {
          c++;
          continue;
        }
        final isFree = v < 0;
        var runEnd = c + 1;
        while (runEnd <= maxC) {
          final nv = logOdds[rowBase + runEnd];
          if (nv == 0 || (nv < 0) != isFree) break;
          runEnd++;
        }
        final left = useFollow
            ? size.width / 2.0 + (c * cs - rp.x) * pxPerM
            : c * cw;
        final span = (runEnd - c) * cw;
        canvas.drawRect(
          Rect.fromLTWH(left, top, span + 0.5, ch + 0.5),
          isFree ? freePaint : occPaint,
        );
        c = runEnd;
      }
    }

    if (useFollow) {
      final center = worldToCanvas(rp.x, rp.y);
      final ringPaint = Paint()
        ..color = const Color(0x5564748B)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.0;
      for (final distM in [1.0, 2.0]) {
        final radiusPx = distM * pxPerM;
        canvas.drawCircle(center, radiusPx, ringPaint);
      }
    }

    final scan = latestScan;
    if (scan != null && scan.ranges.isNotEmpty) {
      final offsets = rayOffsets(mappingParams);
      final rayMeasuredPaint = Paint()
        ..color = const Color.fromARGB(130, 0, 188, 212)
        ..strokeWidth = 1.4;
      final rayOpenPaint = Paint()
        ..color = const Color.fromARGB(45, 0, 188, 212)
        ..strokeWidth = 1.0;
      final hitDotPaint = Paint()
        ..color = const Color.fromARGB(210, 220, 38, 38);
      final origin = worldToCanvas(scan.pose.x, scan.pose.y);
      final n = math.min(offsets.length, scan.ranges.length);
      for (var i = 0; i < n; i++) {
        final r = scan.ranges[i];
        final measured = r.isFinite && r > 0;
        final dist = measured
            ? math.min(r, mappingParams.maxRange)
            : math.min(2.5, mappingParams.maxRange);
        final ang = scan.pose.heading + offsets[i];
        final end = worldToCanvas(
          scan.pose.x + math.cos(ang) * dist,
          scan.pose.y + math.sin(ang) * dist,
        );
        canvas.drawLine(
          origin,
          end,
          measured ? rayMeasuredPaint : rayOpenPaint,
        );
        if (measured) {
          canvas.drawCircle(end, 2.5, hitDotPaint);
        }
      }
    }

    final route = path;
    if (route != null && route.length > 1) {
      final linePaint = Paint()
        ..color = Colors.green
        ..strokeWidth = 2.4
        ..style = PaintingStyle.stroke;
      final line = Path()
        ..moveTo(centerOf(route.first).dx, centerOf(route.first).dy);
      for (final cell in route.skip(1)) {
        final o = centerOf(cell);
        line.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(line, linePaint);
    }

    final th = targetHeading;
    if (rp != null && th != null) {
      final dist = (targetDistM ?? 1.2).clamp(0.3, mappingParams.maxRange);
      final startPt = worldToCanvas(rp.x, rp.y);
      final endPt = worldToCanvas(
        rp.x + math.cos(th) * dist,
        rp.y + math.sin(th) * dist,
      );
      final targetPaint = Paint()
        ..color = const Color.fromARGB(230, 0, 200, 83)
        ..strokeWidth = 3.0
        ..strokeCap = StrokeCap.round;
      canvas.drawLine(startPt, endPt, targetPaint);
      canvas.drawCircle(
        endPt,
        4.0,
        Paint()..color = const Color.fromARGB(230, 0, 200, 83),
      );
    }

    final g = goal;
    if (g != null) {
      canvas.drawCircle(
        centerOf(g),
        math.max(cw * 1.4, 5.5),
        Paint()..color = Colors.blue,
      );
    }

    final rc = rp != null ? worldToCanvas(rp.x, rp.y) : centerOf(robot);
    final robotRadius = math.max(cw * 1.2, 6.5).clamp(6.5, 14.0).toDouble();
    canvas.drawCircle(
      rc,
      robotRadius,
      Paint()..color = const Color(0xFF111827),
    );
    if (rp != null) {
      final tip = Offset(
        rc.dx + math.cos(rp.heading) * (robotRadius * 1.8),
        rc.dy - math.sin(rp.heading) * (robotRadius * 1.8),
      );
      canvas.drawLine(
        rc,
        tip,
        Paint()
          ..color = Colors.white
          ..strokeWidth = 2.5
          ..strokeCap = StrokeCap.round,
      );
    }
    canvas.restore();
  }

  @override
  bool shouldRepaint(covariant MapPainter oldDelegate) => true;
}
