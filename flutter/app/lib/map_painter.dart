import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 占有格子地図および Tesla HUD オーバーレイの描画。
/// 未探索 = 灰、空間 = 白、障害物 = 黒、ロボット = 赤（＋白い向き矢印）、
/// ゴール = 青、経路 = 緑、21 本深度レイ = 水色、VFH 目標ベクトル = 明るめ緑。
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
  });

  @override
  void paint(Canvas canvas, Size size) {
    final cw = size.width / gridW;
    final ch = size.height / gridH;
    final unknownPaint = Paint()
      ..color = const Color.fromARGB(255, 120, 120, 120);
    final freePaint = Paint()..color = const Color.fromARGB(255, 245, 245, 245);
    final occPaint = Paint()..color = const Color.fromARGB(255, 20, 20, 20);

    // 全体を未探索色で塗ってから既知セルのみ描く（240×240 など大グリッド対策）
    canvas.drawRect(Offset.zero & size, unknownPaint);

    for (var r = 0; r < gridH; r++) {
      // 画面は y 下向き、地図は y 上向き（北が上）→ 行を反転して描く
      final top = (gridH - 1 - r) * ch;
      for (var c = 0; c < gridW; c++) {
        final v = logOdds[r * gridW + c];
        if (v == 0) continue;
        final paint = v < 0 ? freePaint : occPaint;
        canvas.drawRect(Rect.fromLTWH(c * cw, top, cw + 0.5, ch + 0.5), paint);
      }
    }

    Offset centerOf(Cell cell) =>
        Offset((cell.c + 0.5) * cw, (gridH - 1 - cell.r + 0.5) * ch);

    Offset worldToCanvas(double wx, double wy) => Offset(
          (wx / mappingParams.cellSize) * cw,
          (gridH - wy / mappingParams.cellSize) * ch,
        );

    final scan = latestScan;
    if (scan != null && scan.ranges.isNotEmpty) {
      final offsets = rayOffsets(mappingParams);
      final rayMeasuredPaint = Paint()
        ..color = const Color.fromARGB(130, 0, 188, 212)
        ..strokeWidth = 1.4;
      final rayOpenPaint = Paint()
        ..color = const Color.fromARGB(45, 0, 188, 212)
        ..strokeWidth = 1.0;
      final hitDotPaint = Paint()..color = const Color.fromARGB(190, 0, 151, 167);
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
        canvas.drawLine(origin, end, measured ? rayMeasuredPaint : rayOpenPaint);
        if (measured) {
          canvas.drawCircle(end, 2.0, hitDotPaint);
        }
      }
    }

    final route = path;
    if (route != null && route.length > 1) {
      final linePaint = Paint()
        ..color = Colors.green
        ..strokeWidth = 2
        ..style = PaintingStyle.stroke;
      final line = Path()
        ..moveTo(centerOf(route.first).dx, centerOf(route.first).dy);
      for (final cell in route.skip(1)) {
        final o = centerOf(cell);
        line.lineTo(o.dx, o.dy);
      }
      canvas.drawPath(line, linePaint);
    }

    final rp = robotPose;
    final th = targetHeading;
    if (rp != null && th != null) {
      final dist = (targetDistM ?? 1.2).clamp(0.3, mappingParams.maxRange);
      final startPt = worldToCanvas(rp.x, rp.y);
      final endPt = worldToCanvas(
        rp.x + math.cos(th) * dist,
        rp.y + math.sin(th) * dist,
      );
      final targetPaint = Paint()
        ..color = const Color.fromARGB(230, 0, 230, 118)
        ..strokeWidth = 3.0
        ..strokeCap = StrokeCap.round;
      canvas.drawLine(startPt, endPt, targetPaint);
      canvas.drawCircle(
        endPt,
        4.0,
        Paint()..color = const Color.fromARGB(230, 0, 230, 118),
      );
    }

    final g = goal;
    if (g != null) {
      canvas.drawCircle(
        centerOf(g),
        math.max(cw * 1.4, 5.0),
        Paint()..color = Colors.blue,
      );
    }

    final rc = rp != null ? worldToCanvas(rp.x, rp.y) : centerOf(robot);
    final robotRadius = math.max(cw * 1.4, 5.5);
    canvas.drawCircle(rc, robotRadius, Paint()..color = Colors.red);
    if (rp != null) {
      final tip = Offset(
        rc.dx + math.cos(rp.heading) * (robotRadius * 1.6),
        rc.dy - math.sin(rp.heading) * (robotRadius * 1.6),
      );
      canvas.drawLine(
        rc,
        tip,
        Paint()
          ..color = Colors.white
          ..strokeWidth = 2.2
          ..strokeCap = StrokeCap.round,
      );
    }
  }

  @override
  bool shouldRepaint(covariant MapPainter oldDelegate) => true;
}
