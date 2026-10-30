import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 距離スキャンの出所（SPEC §5.2 の差し替え点）。
///
/// 実機では ARCore Depth API の深度画像を入力し、次の手順で
/// Scan.ranges（21 本）を作る:
/// 1. 深度画像の中央の帯（車両の進路高さに相当する行）を取り、
///    水平方向に 21 点を等間隔サンプリングする。
/// 2. 生深度ではなく Raw Depth（信頼度付き）を使い、信頼度の低い点は
///    「計測なし（無限遠）」として扱う。
/// 3. 深度画像の画角と FOV 60° の対応は機種ごとのキャリブレーション値。
///
/// 白壁・暗所では深度が外れる前提で、「計測なし」を異常扱いしない。
abstract class DepthSource {
  /// [currentPose] はスキャン時点の believed pose を返す関数。
  Stream<Scan> scans(Pose Function() currentPose);
  Future<void> stop();
}

/// プラットフォームチャネル経由の Depth API 実装（骨格のみ・未接続）。
///
/// ネイティブ側が信頼度フィルタ済みの 21 本の距離 [m] を送る契約。
/// 負値は「計測なし」を表し、無限遠へ変換する。
/// TODO(実機): ネイティブ側の Depth API 実装と接続する。
class MethodChannelDepthSource implements DepthSource {
  static const EventChannel _events = EventChannel('phone_robot/depth');

  final MappingParams params;

  const MethodChannelDepthSource({this.params = const MappingParams()});

  @override
  Stream<Scan> scans(Pose Function() currentPose) {
    return _events.receiveBroadcastStream().map((dynamic e) {
      final m = e as Map<dynamic, dynamic>;
      final raw = m['ranges'] as List<dynamic>;
      final ranges = Float32List(params.nRays);
      for (var i = 0; i < params.nRays && i < raw.length; i++) {
        final v = (raw[i] as num).toDouble();
        ranges[i] = v < 0 ? double.infinity : v;
      }
      return Scan(currentPose(), ranges);
    });
  }

  @override
  Future<void> stop() async {}
}
