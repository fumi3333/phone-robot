import 'dart:convert';
import 'dart:math' as math;
import 'dart:typed_data';

import 'pose.dart';
import 'sim_world.dart';

/// 定点観測スナップショット（変化検出の比較元・比較先）。
///
/// ある地点に立ったときの「周囲の見え方」を再現可能な要約として保存する。
/// 要約の中身は、固定した4方位（東・北・西・南 = 0, π/2, π, 3π/2）
/// それぞれについて前方扇形スキャン（[MappingParams.nRays] 本）を
/// 取ったレイ距離列の連結。同じ地点・同じ世界なら、計測ノイズ
/// （σ=3cm）の範囲でほぼ同じ列が再現される。
///
/// シミュレーションでは SimRobot.castRanges（真値レイキャスト）で作る。
/// 実機では Depth API のスキャンを believed pose で同じ4方位分
/// 集めることに読み替える（SPEC §5.2 の差し替え点）。
class ObservationSnapshot {
  /// 観測地点の名前（ルートのウェイポイント名）。
  final String name;

  /// 観測したセル。
  final Cell cell;

  /// レイ距離列 [m]。長さ = 4方位 × nRays。無限遠 = 計測なし。
  final Float32List ranges;

  /// スナップショットに使う固定方位（[rad]、東・北・西・南）。
  static const headings = <double>[0, math.pi / 2, math.pi, 3 * math.pi / 2];

  const ObservationSnapshot(this.name, this.cell, this.ranges);

  /// シミュレーション用: 現在の真値位置から4方位分を撮影する。
  factory ObservationSnapshot.capture(SimRobot robot, String name) {
    final perHeading = robot.params.nRays;
    final ranges = Float32List(headings.length * perHeading);
    for (var h = 0; h < headings.length; h++) {
      final r = robot.castRanges(headings[h]);
      ranges.setRange(h * perHeading, (h + 1) * perHeading, r);
    }
    return ObservationSnapshot(name, robot.cell, ranges);
  }

  int get rayCount => ranges.length;

  /// JSON 化。無限遠は -1.0 で表す（JSON は Infinity を表せない）。
  Map<String, dynamic> toJson() => {
        'name': name,
        'cell': {'r': cell.r, 'c': cell.c},
        'ranges': [
          for (final v in ranges) v.isInfinite ? -1.0 : v,
        ],
      };

  String toJsonString() => jsonEncode(toJson());

  factory ObservationSnapshot.fromJson(Map<String, dynamic> json) {
    final cellJson = json['cell'] as Map<String, dynamic>;
    final raw = json['ranges'] as List<dynamic>;
    final ranges = Float32List(raw.length);
    for (var i = 0; i < raw.length; i++) {
      final v = (raw[i] as num).toDouble();
      ranges[i] = v < 0 ? double.infinity : v;
    }
    return ObservationSnapshot(
      json['name'] as String,
      (r: cellJson['r'] as int, c: cellJson['c'] as int),
      ranges,
    );
  }
}

/// 2つのスナップショットの比較結果（変化検出の指標）。
class ChangeAssessment {
  /// 「変化あり」と判定したか。
  final bool changed;

  /// しきい値を超えて差が出たレイの本数。
  final int changedRayCount;

  /// 比較したレイの総数。
  final int totalRays;

  /// 全レイの絶対差の平均 [m]（片方だけ計測なしのレイは最大距離分の差として数える）。
  final double meanAbsDiff;

  /// 全レイの絶対差の最大 [m]。
  final double maxAbsDiff;

  const ChangeAssessment({
    required this.changed,
    required this.changedRayCount,
    required this.totalRays,
    required this.meanAbsDiff,
    required this.maxAbsDiff,
  });
}

/// 変化検出（定点の見比べ）。
///
/// 判定指標:
/// - レイごとの絶対差が [rayDiffThresholdM]（既定 0.30 m）を超えたら
///   そのレイは「変化した」と数える。計測ノイズ σ=3cm に対して
///   0.30 m は約10σ で、ノイズだけではまず超えない。
/// - 変化レイ数が [changedRayThreshold]（既定 4 本）以上で
///   「その地点は変化あり」と判定する。箱1個の追加・移動は
///   1方位だけでも数本〜十数本のレイに 0.3 m 超の差を出す。
class ChangeDetector {
  /// レイ単位の差のしきい値 [m]。
  final double rayDiffThresholdM;

  /// 「変化あり」とする変化レイ数の下限。
  final int changedRayThreshold;

  /// 片方だけ「計測なし」のレイに与える差 [m]（センサー最大距離）。
  final double noMeasureDiffM;

  const ChangeDetector({
    this.rayDiffThresholdM = 0.30,
    this.changedRayThreshold = 4,
    this.noMeasureDiffM = 5.0,
  });

  ChangeAssessment compare(
      ObservationSnapshot before, ObservationSnapshot after) {
    if (before.ranges.length != after.ranges.length) {
      throw ArgumentError('snapshot ray counts differ: '
          '${before.ranges.length} vs ${after.ranges.length}');
    }
    var changedRays = 0;
    var sum = 0.0;
    var maxDiff = 0.0;
    for (var i = 0; i < before.ranges.length; i++) {
      final a = before.ranges[i];
      final b = after.ranges[i];
      final double diff;
      if (a.isInfinite && b.isInfinite) {
        diff = 0.0;
      } else if (a.isInfinite || b.isInfinite) {
        diff = noMeasureDiffM;
      } else {
        diff = (a - b).abs();
      }
      if (diff > rayDiffThresholdM) changedRays++;
      sum += diff;
      if (diff > maxDiff) maxDiff = diff;
    }
    final n = before.ranges.length;
    return ChangeAssessment(
      changed: changedRays >= changedRayThreshold,
      changedRayCount: changedRays,
      totalRays: n,
      meanAbsDiff: n == 0 ? 0.0 : sum / n,
      maxAbsDiff: maxDiff,
    );
  }
}
