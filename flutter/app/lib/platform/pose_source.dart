import 'dart:async';

import 'package:flutter/services.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 自己位置の出所（SPEC §5.1 の差し替え点）。
///
/// 実機では ARCore Motion Tracking のカメラポーズを使う。ARCore の
/// 座標系（右手系・y 上・カメラ後方 +z）は SPEC §1 の座標系と異なる
/// ため、ネイティブ側で一度だけ変換し、Dart 側はすべて SPEC §1 の
/// 座標で統一する。変換はセッション開始時の初期位置・初期方位で固定
/// する（ネイティブ側 ArCoreManager の契約）。
///
/// 注意: ARCore の自己位置にもドリフトがある（走行距離の 1〜5% が目安）。
/// 将来の対策（ToF 実測による位置補正、ループクロージャ）は Pose に
/// 補正量を差し込む形で拡張する想定（SPEC §5.1）。
abstract class PoseSource {
  Future<void> start();
  Stream<Pose> get poses;

  /// ARCore のトラッキング状態。値はネイティブ側の文字列そのまま:
  /// 'TRACKING' / 'PAUSED' / 'STOPPED' / 'UNSUPPORTED' / 'ERROR'。
  Stream<String> get trackingStates;

  Future<void> stop();
}

/// プラットフォームチャネル経由の ARCore 実装。
///
/// ネイティブ側（Android / ArCoreManager.kt）が SPEC §1 の座標系へ
/// 変換済みの {x, y, heading} を EventChannel で送る契約。
class MethodChannelPoseSource implements PoseSource {
  static const MethodChannel _method = MethodChannel('phone_robot/arcore');
  static const EventChannel _poseEvents =
      EventChannel('phone_robot/arcore/pose');
  static const EventChannel _trackingEvents =
      EventChannel('phone_robot/arcore/tracking');

  Stream<Pose>? _poseStream;
  Stream<String>? _trackingStream;

  @override
  Future<void> start() => _method.invokeMethod<void>('startPoseTracking');

  @override
  Stream<Pose> get poses {
    return _poseStream ??= _poseEvents.receiveBroadcastStream().map((dynamic e) {
      final m = e as Map<dynamic, dynamic>;
      return Pose(
        (m['x'] as num).toDouble(),
        (m['y'] as num).toDouble(),
        (m['heading'] as num).toDouble(),
      );
    });
  }

  @override
  Stream<String> get trackingStates {
    return _trackingStream ??= _trackingEvents
        .receiveBroadcastStream()
        .map((dynamic e) => e.toString());
  }

  @override
  Future<void> stop() => _method.invokeMethod<void>('stopPoseTracking');
}
