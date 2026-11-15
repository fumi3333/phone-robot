import 'dart:async';

import 'package:flutter/services.dart';

/// ESP32 との USB シリアルリンクの状態。
enum Esp32LinkState { disconnected, connecting, connected, error }

/// USB シリアルデバイスの情報（ネイティブ側の列挙結果）。
class Esp32Device {
  final int deviceId;
  final int vendorId;
  final int productId;
  final String name;

  const Esp32Device({
    required this.deviceId,
    required this.vendorId,
    required this.productId,
    required this.name,
  });

  factory Esp32Device.fromMap(Map<dynamic, dynamic> m) => Esp32Device(
        deviceId: (m['deviceId'] as num).toInt(),
        vendorId: (m['vendorId'] as num).toInt(),
        productId: (m['productId'] as num).toInt(),
        name: m['name']?.toString() ?? 'unknown',
      );

  @override
  String toString() =>
      '$name (VID ${vendorId.toRadixString(16)} / PID ${productId.toRadixString(16)})';
}

/// ESP32 へのシリアルリンク（ファーム v2.0 系プロトコル）。
///
/// プロトコル: 115200 bps、コマンド "c,drive,steer\n"（連続値）と
/// 単文字コマンド f/b/l/r/s。ファーム側に 400 ms の通信断ウォッチ
/// ドッグがあるため、走行中は 5 Hz 以上でコマンドを送り続ける必要が
/// ある（RealDriver の制御ループが 10 Hz で送る）。停止は 's' を送る
/// か、送信を止めればウォッチドッグで止まる。
abstract class Esp32Link {
  /// リンク状態の変化。
  Stream<Esp32LinkState> get states;

  /// 現在のリンク状態（最後に分かった値）。
  Esp32LinkState get state;

  /// ESP32 から受信したテキスト（ログ表示用）。
  Stream<String> get received;

  /// 最後に送信したコマンド文字列（UI の「現在の指令」表示用）。
  String get lastCommand;

  /// 駆動・操舵指令を送信した直後に呼ばれるコールバック（走行ログ記録用）。
  void Function(int drive, int steer)? onCommandSent;

  Future<List<Esp32Device>> listDevices();

  /// 接続する。[deviceId] が null なら先頭の対応デバイスを使う。
  /// USB 権限が未許可ならネイティブ側が権限要求ダイアログを出す。
  Future<bool> connect({int? deviceId, int baudRate = 115200});

  Future<void> disconnect();

  /// 生テキストを送る。
  Future<bool> send(String text);

  /// 連続値コマンド "c,drive,steer\n" を送る。
  /// drive / steer の値域はファーム側の受付範囲に合わせる
  /// （[SerialCommandScale] を参照）。
  Future<bool> sendDriveSteer(int drive, int steer);

  /// 単文字コマンド（f/b/l/r/s）を送る。手動ジョグ用。
  Future<bool> sendJog(String ch);
}

/// drive / steer の整数レンジ変換。
///
/// ファーム（esp32_toyota86 / esp32_toyota86_asbuilt）の "c,drive,steer" は
/// -255..255 の PWM duty を受け付ける。駆動モーターは静止摩擦デッドゾーンを
/// 超える [minEffectiveDrivePwm..maxValue] へ [driveOf] でマッピングする。
class SerialCommandScale {
  static const int maxValue = 255;
  static const int minEffectiveDrivePwm = 140;

  static int of(double normalized) {
    final v = (normalized.clamp(-1.0, 1.0) * maxValue).round();
    return v;
  }

  /// 駆動モーター用: 0 は 0、非ゼロはギアボックスの静止摩擦を超える
  /// [minEffectiveDrivePwm..maxValue] に線形マッピングする。
  static int driveOf(double normalized) {
    final clamped = normalized.clamp(-1.0, 1.0);
    final mag = clamped.abs();
    if (mag < 1e-3) return 0;
    final pwm = (minEffectiveDrivePwm +
            mag * (maxValue - minEffectiveDrivePwm))
        .round()
        .clamp(minEffectiveDrivePwm, maxValue);
    return clamped < 0 ? -pwm : pwm;
  }
}

/// プラットフォームチャネル経由の実装（ネイティブ側は
/// usb-serial-for-android を使う SerialManager.kt）。
class MethodChannelEsp32Link implements Esp32Link {
  static const MethodChannel _method = MethodChannel('phone_robot/serial');
  static const EventChannel _stateEvents =
      EventChannel('phone_robot/serial/state');
  static const EventChannel _rxEvents = EventChannel('phone_robot/serial/rx');

  Esp32LinkState _state = Esp32LinkState.disconnected;
  String _lastCommand = '(なし)';
  Stream<Esp32LinkState>? _states;

  @override
  void Function(int drive, int steer)? onCommandSent;

  @override
  Esp32LinkState get state => _state;

  @override
  String get lastCommand => _lastCommand;

  @override
  Stream<Esp32LinkState> get states {
    return _states ??= _stateEvents.receiveBroadcastStream().map((dynamic e) {
      final s = switch (e.toString()) {
        'connected' => Esp32LinkState.connected,
        'connecting' => Esp32LinkState.connecting,
        'error' => Esp32LinkState.error,
        _ => Esp32LinkState.disconnected,
      };
      _state = s;
      return s;
    });
  }

  @override
  Stream<String> get received =>
      _rxEvents.receiveBroadcastStream().map((dynamic e) => e.toString());

  @override
  Future<List<Esp32Device>> listDevices() async {
    final raw = await _method.invokeMethod<List<dynamic>>('listDevices');
    if (raw == null) return const [];
    return raw
        .map((e) => Esp32Device.fromMap(e as Map<dynamic, dynamic>))
        .toList();
  }

  @override
  Future<bool> connect({int? deviceId, int baudRate = 115200}) async {
    final ok = await _method.invokeMethod<bool>('connect', {
      'deviceId': deviceId,
      'baudRate': baudRate,
    });
    if (ok == true) {
      _state = Esp32LinkState.connected;
    }
    return ok ?? false;
  }

  @override
  Future<void> disconnect() async {
    await _method.invokeMethod<void>('disconnect');
    _state = Esp32LinkState.disconnected;
  }

  @override
  Future<bool> send(String text) async {
    _lastCommand = text.replaceAll('\n', r'\n');
    final ok = await _method.invokeMethod<bool>('send', {'text': text});
    return ok ?? false;
  }

  @override
  Future<bool> sendDriveSteer(int drive, int steer) async {
    final ok = await send('c,$drive,$steer\n');
    onCommandSent?.call(drive, steer);
    return ok;
  }

  @override
  Future<bool> sendJog(String ch) async {
    final ok = await send(ch);
    if (ch == 's') {
      onCommandSent?.call(0, 0);
    }
    return ok;
  }
}
