import 'package:flutter/services.dart';

/// 画面の常時点灯（R4 対策）。
///
/// 実機モード中は画面消灯が One UI のプロセス凍結の引き金になるため、
/// センサー開始中はネイティブ側で FLAG_KEEP_SCREEN_ON を立てる。
/// チャネルが無い環境（テスト等）では静かに何もしない。
class KeepAwake {
  static const MethodChannel _channel = MethodChannel('phone_robot/system');

  static Future<void> setEnabled(bool on) async {
    try {
      await _channel.invokeMethod<void>('setKeepScreenOn', {'on': on});
    } catch (_) {
      // ネイティブ側未実装の環境では無視する（安全側の補助策のため）。
    }
  }
}
