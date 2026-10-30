import 'dart:typed_data';

import 'params.dart';
import 'pose.dart';

/// 距離スキャン（SPEC §2.3）。
///
/// [ranges] は [MappingParams.nRays] 本、単位 m。
/// 無限遠（double.infinity）は「計測なし」を表す。
class Scan {
  /// スキャンした時点の believed pose。
  final Pose pose;

  /// レイごとの計測距離 [m]。
  final Float32List ranges;

  const Scan(this.pose, this.ranges);
}

/// レイの角度オフセット表（前方向の扇形、等間隔、[rad]）。
Float64List rayOffsets(MappingParams p) {
  final o = Float64List(p.nRays);
  for (var i = 0; i < p.nRays; i++) {
    o[i] = -p.fovRad / 2 + p.fovRad * i / (p.nRays - 1);
  }
  return o;
}
