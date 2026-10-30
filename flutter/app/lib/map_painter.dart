import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 占有格子地図の描画。色は参照実装の render_map と同じ割当て:
/// 未探索 = 灰、空間 = 白、障害物 = 黒、ロボット = 赤、
/// ゴール = 青、経路 = 緑。
class MapPainter extends CustomPainter {
  final Float32List logOdds;
  final int gridW;
  final int gridH;
  final Cell robot;
  final Cell? goal;
  final List<Cell>? path;

  const MapPainter({
    required this.logOdds,
    required this.gridW,
    required this.gridH,
    required this.robot,
    this.goal,
    this.path,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final cw = size.width / gridW;
    final ch = size.height / gridH;
    final unknownPaint = Paint()
      ..color = const Color.fromARGB(255, 120, 120, 120);
    final freePaint = Paint()..color = const Color.fromARGB(255, 245, 245, 245);
    final occPaint = Paint()..color = const Color.fromARGB(255, 20, 20, 20);

    for (var r = 0; r < gridH; r++) {
      // 画面は y 下向き、地図は y 上向き（北が上）→ 行を反転して描く
      final top = (gridH - 1 - r) * ch;
      for (var c = 0; c < gridW; c++) {
        final v = logOdds[r * gridW + c];
        final paint = v == 0 ? unknownPaint : (v < 0 ? freePaint : occPaint);
        canvas.drawRect(Rect.fromLTWH(c * cw, top, cw + 0.5, ch + 0.5), paint);
      }
    }

    Offset centerOf(Cell cell) =>
        Offset((cell.c + 0.5) * cw, (gridH - 1 - cell.r + 0.5) * ch);

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

    final g = goal;
    if (g != null) {
      canvas.drawCircle(centerOf(g), cw * 1.4, Paint()..color = Colors.blue);
    }
    canvas.drawCircle(centerOf(robot), cw * 1.4, Paint()..color = Colors.red);
  }

  @override
  bool shouldRepaint(covariant MapPainter oldDelegate) => true;
}
