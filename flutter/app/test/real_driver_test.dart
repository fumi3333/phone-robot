import 'dart:async';
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:phone_robot_app/platform/serial_link.dart';
import 'package:phone_robot_app/real/real_driver.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// テスト用のフェイク ESP32 リンク。送った文字列を全部記録する。
class FakeLink implements Esp32Link {
  Esp32LinkState _state = Esp32LinkState.connected;
  final List<String> sent = [];
  bool sendOk = true;

  @override
  Esp32LinkState get state => _state;
  set state(Esp32LinkState v) => _state = v;

  @override
  Stream<Esp32LinkState> get states => const Stream.empty();

  @override
  Stream<String> get received => const Stream.empty();

  @override
  String get lastCommand => sent.isEmpty ? '(なし)' : sent.last;

  @override
  Future<List<Esp32Device>> listDevices() async => const [];

  @override
  Future<bool> connect({int? deviceId, int baudRate = 115200}) async {
    _state = Esp32LinkState.connected;
    return true;
  }

  @override
  Future<void> disconnect() async {
    _state = Esp32LinkState.disconnected;
  }

  @override
  Future<bool> send(String text) async {
    sent.add(text);
    return sendOk;
  }

  @override
  Future<bool> sendDriveSteer(int drive, int steer) => send('c,$drive,$steer\n');

  @override
  Future<bool> sendJog(String ch) => send(ch);

  int get driveCommandCount => sent.where((s) => s.startsWith('c,')).length;
}

/// 実ストリームの代わりに、ポーズとスキャンを定期的に流し続ける。
class StreamPump {
  final PoseFeed poseFeed;
  final ScanFeed feed;
  Pose pose;
  Timer? _timer;

  StreamPump(this.poseFeed, this.feed, this.pose);

  void start() {
    _tick();
    _timer = Timer.periodic(const Duration(milliseconds: 50), (_) => _tick());
  }

  void _tick() {
    poseFeed.addPose(pose);
    feed.add(Scan(
      pose,
      Float32List.fromList(List.filled(21, double.infinity)),
    ));
  }

  void stop() => _timer?.cancel();
}

void main() {
  const params = MappingParams();
  // 目標ウェイポイント: 中心 (1.525, 1.025)。開始ポーズは (1.0, 1.0, 0°)。
  const target = (r: 20, c: 30);

  RealDriver makeDriver({
    required FakeLink link,
    required PoseFeed poseFeed,
    required ScanFeed feed,
    required OccupancyGrid grid,
  }) {
    final d = RealDriver(
      params: params,
      grid: grid,
      link: link,
      feed: feed,
      poseFeed: poseFeed,
    );
    // テストでは変数を分離するためポーズ鮮度を緩め、復帰待ちを短くする
    d.poseMaxAge = const Duration(seconds: 5);
    d.trackingRecoveryTimeout = const Duration(milliseconds: 400);
    d.scanRecoveryTimeout = const Duration(milliseconds: 400);
    return d;
  }

  Float32List farRanges() =>
      Float32List.fromList(List.filled(21, double.infinity));

  test('R1: トラッキング喪失中は駆動指令を出さず sensorLost で止まる', () async {
    final link = FakeLink();
    final poseFeed = PoseFeed();
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    poseFeed.addPose(Pose(1.0, 1.0, 0));
    poseFeed.setTrackingState('PAUSED');
    feed.add(Scan(Pose(1.0, 1.0, 0), farRanges()));
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );

    final outcome = await driver.driveToWaypoint(target);

    expect(outcome, DriveOutcome.sensorLost);
    expect(driver.lastStopReason, 'tracking-lost');
    // 修正前は古いポーズを信じて 'c,' を送り続けていた
    expect(link.driveCommandCount, 0);
    expect(link.sent, contains('s'));
  });

  test('R1: 走行中に喪失→復帰したら replan を返し、その間は止まっている',
      () async {
    final link = FakeLink();
    final poseFeed = PoseFeed()..setTrackingState('TRACKING');
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );
    final pump = StreamPump(poseFeed, feed, Pose(1.0, 1.0, 0))..start();
    try {
      final future = driver.driveToWaypoint(target);
      await Future<void>.delayed(const Duration(milliseconds: 250));
      expect(link.driveCommandCount, greaterThan(0)); // 走行していた
      poseFeed.setTrackingState('PAUSED');
      final atPause = link.driveCommandCount;
      await Future<void>.delayed(const Duration(milliseconds: 250));
      poseFeed.setTrackingState('TRACKING');
      final outcome = await future;

      expect(outcome, DriveOutcome.replan);
      // 喪失検知後に余分に進んでいない（検知と同一周期の1回まで許容）
      expect(link.driveCommandCount - atPause, lessThanOrEqualTo(1));
      expect(link.sent, contains('s'));
    } finally {
      pump.stop();
    }
  });

  test('R2: スキャンが古ければ進まず sensorLost（フェイルクローズ）', () async {
    final link = FakeLink();
    final poseFeed = PoseFeed()..setTrackingState('TRACKING');
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    poseFeed.addPose(Pose(1.0, 1.0, 0));
    feed.add(Scan(Pose(1.0, 1.0, 0), farRanges()));
    feed.lastAt = DateTime.now().subtract(const Duration(seconds: 10));
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );

    final outcome = await driver.driveToWaypoint(target);

    expect(outcome, DriveOutcome.sensorLost);
    expect(driver.lastStopReason, 'scan-stale');
    // 修正前は「データなし＝障害物なし」で進んでいた
    expect(link.driveCommandCount, 0);
    // 地図にも何も書かれていない
    expect(grid.logOdds.every((v) => v == 0), isTrue);
  });

  test('対照: 健全な入力では通常どおり到達する', () async {
    final link = FakeLink();
    final poseFeed = PoseFeed()..setTrackingState('TRACKING');
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );
    final pump = StreamPump(poseFeed, feed, Pose(1.0, 1.0, 0))..start();
    try {
      final future = driver.driveToWaypoint(target);
      await Future<void>.delayed(const Duration(milliseconds: 250));
      pump.pose = Pose(1.5, 1.0, 0); // 目標付近へ移動したことにする
      final outcome = await future;
      expect(outcome, DriveOutcome.arrived);
      expect(link.driveCommandCount, greaterThan(0));
    } finally {
      pump.stop();
    }
  });

  test('R3: 走行中にリンクが切れたら linkLost で止まり、地図に壁を書かない',
      () async {
    final link = FakeLink();
    final poseFeed = PoseFeed()..setTrackingState('TRACKING');
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );
    final pump = StreamPump(poseFeed, feed, Pose(1.0, 1.0, 0))..start();
    try {
      final future = driver.driveToWaypoint(target);
      await Future<void>.delayed(const Duration(milliseconds: 250));
      expect(link.driveCommandCount, greaterThan(0)); // 走行していた
      link.state = Esp32LinkState.disconnected;
      link.sendOk = false;
      final outcome = await future;

      expect(outcome, DriveOutcome.linkLost);
      expect(driver.lastStopReason, 'link-lost');
      // 修正前はタイムアウト後に _markBlockedAhead で偽の壁を書いていた。
      // 通常のスキャン適用による「空き」証拠（負値）は正当だが、
      // 障害物マーク（>= 2.0）はどこにも書かれていないはず。
      expect(grid.logOdds.every((v) => v < 2.0), isTrue);
      final ahead = cellOf(params, 1.22, 1.0);
      expect(grid.logOdds[grid.index(ahead.r, ahead.c)], lessThan(2.0));
    } finally {
      pump.stop();
    }
  });

  test('R3 対照: 健全なのに進めない（タイムアウト）場合は障害物を記録する',
      () async {
    final link = FakeLink();
    final poseFeed = PoseFeed()..setTrackingState('TRACKING');
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );
    driver.waypointTimeout = const Duration(milliseconds: 700);
    final pump = StreamPump(poseFeed, feed, Pose(1.0, 1.0, 0))..start();
    try {
      final outcome = await driver.driveToWaypoint(target);
      expect(outcome, DriveOutcome.timeout);
      // 進行方向（+x、0.22 m 先）のセルが占有として記録される
      final ahead = cellOf(params, 1.22, 1.0);
      expect(grid.logOdds[grid.index(ahead.r, ahead.c)], greaterThanOrEqualTo(2.0));
    } finally {
      pump.stop();
    }
  });

  test('R1: 回転中もトラッキング喪失では回らず false を返す', () async {
    final link = FakeLink();
    final poseFeed = PoseFeed();
    final feed = ScanFeed();
    final grid = OccupancyGrid(params);
    poseFeed.addPose(Pose(1.0, 1.0, 0));
    poseFeed.setTrackingState('PAUSED');
    feed.add(Scan(Pose(1.0, 1.0, 0), farRanges()));
    final driver = makeDriver(
      link: link,
      poseFeed: poseFeed,
      feed: feed,
      grid: grid,
    );

    final ok = await driver.rotateTo(1.0);

    expect(ok, isFalse);
    expect(link.driveCommandCount, 0);
  });
}
