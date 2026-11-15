import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 走行軌跡の1点（ARCore グリッド座標）。
typedef TrajectoryPoint = ({
  int elapsedMs,
  double x,
  double y,
  double heading,
});

/// 走行終了時（または手動記録停止時）に出力される 4 ファイル一式。
class RunBundle {
  final String runId;
  final String mode;
  final DateTime startedAt;
  final DateTime endedAt;
  final String csvContent;
  final String jsonContent;
  final String summaryMarkdown;
  final Uint8List mapPngBytes;
  final int commandCount;
  final int scanCount;
  final double mappedAreaM2;
  final double coverage;

  const RunBundle({
    required this.runId,
    required this.mode,
    required this.startedAt,
    required this.endedAt,
    required this.csvContent,
    required this.jsonContent,
    required this.summaryMarkdown,
    required this.mapPngBytes,
    required this.commandCount,
    required this.scanCount,
    required this.mappedAreaM2,
    required this.coverage,
  });

  /// MethodChannel 経由で Kotlin 側へ渡すファイルマップ。
  Map<String, Object> toChannelFiles() => {
        'driving_log_$runId.csv': csvContent,
        'floor_map_$runId.png': mapPngBytes,
        'floor_map_$runId.json': jsonContent,
        'run_summary_$runId.md': summaryMarkdown,
      };
}

/// 1回の走行中のコマンド・深度スキャン・軌跡を記録し、[RunBundle] を組み立てる。
class RunRecorder {
  static const int maxRows = 20000;
  static const int minScanIntervalMs = 100;

  final String runId;
  final String mode;
  final DateTime startedAt;

  final StringBuffer _csv = StringBuffer();
  final List<TrajectoryPoint> _trajectory = [];

  int _commandCount = 0;
  int _scanCount = 0;
  int _lastDrivePwm = 0;
  int _lastSteerPwm = 0;
  int _lastScanRecordedMs = -minScanIntervalMs;
  int _lastTrajectoryMs = -1000;
  double? _lastTrajectoryX;
  double? _lastTrajectoryY;

  RunRecorder({
    String? runId,
    required this.mode,
    DateTime? startedAt,
  })  : startedAt = startedAt ?? DateTime.now(),
        runId = runId ?? formatRunId(startedAt ?? DateTime.now()) {
    _csv.writeln(
      'timestamp_ms,elapsed_ms,event,mode,drive_pwm,steer_pwm,'
      'x_m,y_m,heading_rad,tracking_state,min_depth_m,rays_m',
    );
  }

  static String formatRunId(DateTime dt) {
    String d2(int v) => v.toString().padLeft(2, '0');
    return '${dt.year}${d2(dt.month)}${d2(dt.day)}_'
        '${d2(dt.hour)}${d2(dt.minute)}${d2(dt.second)}';
  }

  int get rowCount => _commandCount + _scanCount;
  int get commandCount => _commandCount;
  int get scanCount => _scanCount;
  List<TrajectoryPoint> get trajectory => List.unmodifiable(_trajectory);

  void recordCommand(
    int drivePwm,
    int steerPwm, {
    Pose? pose,
    String trackingState = '',
    Scan? latestScan,
    DateTime? at,
  }) {
    _lastDrivePwm = drivePwm;
    _lastSteerPwm = steerPwm;
    if (rowCount >= maxRows) return;

    final now = at ?? DateTime.now();
    final elapsedMs = now.difference(startedAt).inMilliseconds;
    _appendRow(
      timestampMs: now.millisecondsSinceEpoch,
      elapsedMs: elapsedMs,
      event: 'cmd',
      drivePwm: drivePwm,
      steerPwm: steerPwm,
      pose: pose ?? latestScan?.pose,
      trackingState: trackingState,
      scan: latestScan,
    );
    _commandCount++;
    if (pose != null) {
      _maybeAddTrajectoryPoint(elapsedMs, pose);
    }
  }

  void recordScan(
    Scan scan, {
    required String trackingState,
    DateTime? at,
  }) {
    final now = at ?? DateTime.now();
    final elapsedMs = now.difference(startedAt).inMilliseconds;
    _maybeAddTrajectoryPoint(elapsedMs, scan.pose);

    if (rowCount >= maxRows) return;
    if (elapsedMs - _lastScanRecordedMs < minScanIntervalMs) return;
    _lastScanRecordedMs = elapsedMs;

    _appendRow(
      timestampMs: now.millisecondsSinceEpoch,
      elapsedMs: elapsedMs,
      event: 'scan',
      drivePwm: _lastDrivePwm,
      steerPwm: _lastSteerPwm,
      pose: scan.pose,
      trackingState: trackingState,
      scan: scan,
    );
    _scanCount++;
  }

  void _maybeAddTrajectoryPoint(int elapsedMs, Pose pose) {
    final lx = _lastTrajectoryX;
    final ly = _lastTrajectoryY;
    if (lx == null || ly == null) {
      _trajectory.add((
        elapsedMs: elapsedMs,
        x: pose.x,
        y: pose.y,
        heading: pose.heading,
      ));
      _lastTrajectoryX = pose.x;
      _lastTrajectoryY = pose.y;
      _lastTrajectoryMs = elapsedMs;
      return;
    }
    final distSq = (pose.x - lx) * (pose.x - lx) + (pose.y - ly) * (pose.y - ly);
    if (distSq >= 0.03 * 0.03 || elapsedMs - _lastTrajectoryMs >= 500) {
      _trajectory.add((
        elapsedMs: elapsedMs,
        x: pose.x,
        y: pose.y,
        heading: pose.heading,
      ));
      _lastTrajectoryX = pose.x;
      _lastTrajectoryY = pose.y;
      _lastTrajectoryMs = elapsedMs;
    }
  }

  void _appendRow({
    required int timestampMs,
    required int elapsedMs,
    required String event,
    required int drivePwm,
    required int steerPwm,
    required Pose? pose,
    required String trackingState,
    required Scan? scan,
  }) {
    final xStr = pose == null ? '' : pose.x.toStringAsFixed(4);
    final yStr = pose == null ? '' : pose.y.toStringAsFixed(4);
    final hStr = pose == null ? '' : pose.heading.toStringAsFixed(4);

    var minDepthStr = '';
    var raysStr = '';
    if (scan != null && scan.ranges.isNotEmpty) {
      var minVal = double.infinity;
      final rayTokens = <String>[];
      for (final r in scan.ranges) {
        if (r.isFinite && r > 0) {
          if (r < minVal) minVal = r;
          rayTokens.add(r.toStringAsFixed(3));
        } else {
          rayTokens.add('inf');
        }
      }
      if (minVal.isFinite) {
        minDepthStr = minVal.toStringAsFixed(3);
      }
      raysStr = rayTokens.join(';');
    }

    _csv.writeln(
      '$timestampMs,$elapsedMs,${_csvCell(event)},${_csvCell(mode)},'
      '$drivePwm,$steerPwm,$xStr,$yStr,$hStr,${_csvCell(trackingState)},'
      '$minDepthStr,${_csvCell(raysStr)}',
    );
  }

  static String _csvCell(String raw) {
    if (!raw.contains(',') && !raw.contains('"') && !raw.contains('\n')) {
      return raw;
    }
    final escaped = raw.replaceAll('"', '""');
    return '"$escaped"';
  }

  RunBundle buildBundle({
    required OccupancyGrid grid,
    required MappingParams params,
    required double speedNorm,
    required int currentDrivePwm,
    required bool invertSteer,
    Cell? robotCell,
    DateTime? endedAt,
  }) {
    final end = endedAt ?? DateTime.now();
    final durationSec = math.max(0.0, end.difference(startedAt).inMilliseconds / 1000.0);
    final mappedAreaM2 = grid.knownCount * params.cellSize * params.cellSize;
    final coverage = grid.coverage;

    final mapPngBytes = encodeOccupancyGridPng(
      grid: grid,
      trajectory: _trajectory,
      params: params,
      robotCell: robotCell,
    );

    final jsonPayload = {
      'runId': runId,
      'mode': mode,
      'startedAt': startedAt.toIso8601String(),
      'endedAt': end.toIso8601String(),
      'durationSec': double.parse(durationSec.toStringAsFixed(2)),
      'mappingParams': {
        'cellSizeM': params.cellSize,
        'gridW': params.gridW,
        'gridH': params.gridH,
        'roomWM': params.roomW,
        'roomHM': params.roomH,
      },
      'settings': {
        'speedNorm': double.parse(speedNorm.toStringAsFixed(2)),
        'currentDrivePwm': currentDrivePwm,
        'invertSteer': invertSteer,
      },
      'stats': {
        'mappedAreaM2': double.parse(mappedAreaM2.toStringAsFixed(3)),
        'coverage': double.parse(coverage.toStringAsFixed(4)),
        'knownCount': grid.knownCount,
        'commandCount': _commandCount,
        'scanCount': _scanCount,
      },
      'trajectory': [
        for (final pt in _trajectory)
          {
            'elapsedMs': pt.elapsedMs,
            'x': double.parse(pt.x.toStringAsFixed(4)),
            'y': double.parse(pt.y.toStringAsFixed(4)),
            'heading': double.parse(pt.heading.toStringAsFixed(4)),
          },
      ],
      'logOdds': [
        for (final v in grid.logOdds) double.parse(v.toStringAsFixed(3)),
      ],
    };

    final md = StringBuffer()
      ..writeln('# 走行記録レポート ($runId)')
      ..writeln()
      ..writeln('- 走行ID: $runId')
      ..writeln('- 走行モード: $mode')
      ..writeln('- 開始時刻: ${startedAt.toIso8601String()}')
      ..writeln('- 終了時刻: ${end.toIso8601String()}（${durationSec.toStringAsFixed(1)} 秒）')
      ..writeln('- 対象スケール: ${params.roomW.toStringAsFixed(1)} m × '
          '${params.roomH.toStringAsFixed(1)} m'
          '（グリッド ${params.gridW}×${params.gridH}、セル '
          '${(params.cellSize * 100).round()} cm）')
      ..writeln('- マッピング面積: ${mappedAreaM2.toStringAsFixed(2)} m²'
          '（カバー率 ${(coverage * 100).toStringAsFixed(1)}%）')
      ..writeln('- 記録件数: コマンド $_commandCount 件 / 深度スキャン $_scanCount 件 / '
          '軌跡 ${_trajectory.length} 点')
      ..writeln('- 走行設定: 速度 ${(speedNorm * 100).round()}% '
          '(PWM $currentDrivePwm), 操舵反転 ${invertSteer ? 'ON' : 'OFF'}')
      ..writeln()
      ..writeln('## 出力ファイル')
      ..writeln('- `driving_log_$runId.csv`: 時系列操作コマンド・ARCore自己位置・21本深度レイ')
      ..writeln('- `floor_map_$runId.png`: 占有格子地図と走行軌跡画像')
      ..writeln('- `floor_map_$runId.json`: グリッド数値データ（logOdds配列）および軌跡JSON');

    return RunBundle(
      runId: runId,
      mode: mode,
      startedAt: startedAt,
      endedAt: end,
      csvContent: _csv.toString(),
      jsonContent: jsonEncode(jsonPayload),
      summaryMarkdown: md.toString(),
      mapPngBytes: mapPngBytes,
      commandCount: _commandCount,
      scanCount: _scanCount,
      mappedAreaM2: mappedAreaM2,
      coverage: coverage,
    );
  }
}

/// 占有格子地図と走行軌跡を RGB PNG 画像（外部ライブラリなし・純 Dart）として書き出す。
Uint8List encodeOccupancyGridPng({
  required OccupancyGrid grid,
  required List<TrajectoryPoint> trajectory,
  required MappingParams params,
  Cell? robotCell,
}) {
  final scale = params.gridW <= 120 ? 4 : 2;
  final width = params.gridW * scale;
  final height = params.gridH * scale;
  final pixels = Uint8List(width * height * 3);

  void fillCell(int r, int c, int rr, int gg, int bb) {
    if (r < 0 || r >= params.gridH || c < 0 || c >= params.gridW) return;
    // MapPainter と同様に北（y 増加方向）を画像の上へ配置する。
    final imgY0 = (params.gridH - 1 - r) * scale;
    final imgX0 = c * scale;
    for (var dy = 0; dy < scale; dy++) {
      final rowOffset = ((imgY0 + dy) * width + imgX0) * 3;
      for (var dx = 0; dx < scale; dx++) {
        final idx = rowOffset + dx * 3;
        pixels[idx] = rr;
        pixels[idx + 1] = gg;
        pixels[idx + 2] = bb;
      }
    }
  }

  for (var r = 0; r < params.gridH; r++) {
    for (var c = 0; c < params.gridW; c++) {
      final v = grid.logOdds[r * params.gridW + c];
      if (v.abs() <= 1e-6) {
        fillCell(r, c, 229, 231, 235);
      } else if (v < 0) {
        fillCell(r, c, 255, 255, 255);
      } else {
        fillCell(r, c, 17, 24, 39);
      }
    }
  }

  if (trajectory.isNotEmpty) {
    Cell? prev;
    for (final pt in trajectory) {
      final cur = cellOf(params, pt.x, pt.y);
      if (prev != null) {
        final steps = math.max(
          (cur.r - prev.r).abs(),
          (cur.c - prev.c).abs(),
        );
        if (steps > 1) {
          for (var s = 1; s < steps; s++) {
            final t = s / steps;
            final ir = (prev.r + (cur.r - prev.r) * t).round();
            final ic = (prev.c + (cur.c - prev.c) * t).round();
            fillCell(ir, ic, 37, 99, 235);
          }
        }
      }
      fillCell(cur.r, cur.c, 37, 99, 235);
      prev = cur;
    }

    final startCell = cellOf(params, trajectory.first.x, trajectory.first.y);
    for (var dr = -1; dr <= 1; dr++) {
      for (var dc = -1; dc <= 1; dc++) {
        fillCell(startCell.r + dr, startCell.c + dc, 22, 163, 74);
      }
    }
  }

  final endCell = robotCell ??
      (trajectory.isNotEmpty
          ? cellOf(params, trajectory.last.x, trajectory.last.y)
          : null);
  if (endCell != null) {
    for (var dr = -1; dr <= 1; dr++) {
      for (var dc = -1; dc <= 1; dc++) {
        fillCell(endCell.r + dr, endCell.c + dc, 220, 38, 38);
      }
    }
  }

  return _encodeRgbPng(width, height, pixels);
}

Uint8List _encodeRgbPng(int width, int height, Uint8List rgbPixels) {
  final stride = width * 3;
  final raw = Uint8List(height * (1 + stride));
  for (var y = 0; y < height; y++) {
    final dstRow = y * (1 + stride);
    raw[dstRow] = 0; // filter type None
    final srcRow = y * stride;
    raw.setRange(dstRow + 1, dstRow + 1 + stride, rgbPixels, srcRow);
  }
  final compressed = Uint8List.fromList(zlib.encode(raw));

  final ihdr = ByteData(13)
    ..setUint32(0, width, Endian.big)
    ..setUint32(4, height, Endian.big)
    ..setUint8(8, 8) // bit depth
    ..setUint8(9, 2) // color type RGB
    ..setUint8(10, 0) // compression
    ..setUint8(11, 0) // filter
    ..setUint8(12, 0); // interlace

  final out = BytesBuilder(copy: false)
    ..add(const [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    ..add(_pngChunk('IHDR', ihdr.buffer.asUint8List()))
    ..add(_pngChunk('IDAT', compressed))
    ..add(_pngChunk('IEND', Uint8List(0)));
  return out.takeBytes();
}

Uint8List _pngChunk(String type, Uint8List data) {
  final typeBytes = ascii.encode(type);
  final header = ByteData(4)..setUint32(0, data.length, Endian.big);

  final crcInput = Uint8List(4 + data.length)
    ..setRange(0, 4, typeBytes)
    ..setRange(4, 4 + data.length, data);
  final crc = ByteData(4)..setUint32(0, _crc32(crcInput), Endian.big);

  return (BytesBuilder(copy: false)
        ..add(header.buffer.asUint8List())
        ..add(crcInput)
        ..add(crc.buffer.asUint8List()))
      .takeBytes();
}

final List<int> _crcTable = List<int>.generate(256, (n) {
  var c = n;
  for (var k = 0; k < 8; k++) {
    c = (c & 1) != 0 ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
  }
  return c;
});

int _crc32(Uint8List bytes) {
  var c = 0xFFFFFFFF;
  for (final b in bytes) {
    c = _crcTable[(c ^ b) & 0xFF] ^ (c >>> 8);
  }
  return (c ^ 0xFFFFFFFF) & 0xFFFFFFFF;
}

/// 走行記録保存・Google ドライブ連携の保存結果。
class SaveRunResult {
  final String savedDir;
  final String driveAction;
  final bool ok;

  const SaveRunResult({
    required this.savedDir,
    required this.driveAction,
    required this.ok,
  });
}

/// 走行記録を端末領域および Google ドライブへ保存するストレージ抽象。
abstract class RunStorage {
  Future<SaveRunResult> saveRunBundle(
    RunBundle bundle, {
    required bool autoDriveSync,
    required bool openDriveSheet,
  });

  Future<bool> shareRunToDrive({String? runId});

  Future<String> getWebhookUrl();

  Future<bool> setWebhookUrl(String url);
}

/// Android ネイティブ実装（[LogStorageManager]）を呼び出す MethodChannel ラッパ。
class MethodChannelRunStorage implements RunStorage {
  static const MethodChannel _channel = MethodChannel('phone_robot/storage');

  @override
  Future<SaveRunResult> saveRunBundle(
    RunBundle bundle, {
    required bool autoDriveSync,
    required bool openDriveSheet,
  }) async {
    try {
      final raw = await _channel.invokeMethod<Map<dynamic, dynamic>>(
        'saveRunBundle',
        {
          'runId': bundle.runId,
          'files': bundle.toChannelFiles(),
          'autoDriveSync': autoDriveSync,
          'openDriveSheet': openDriveSheet,
        },
      );
      if (raw == null) {
        return const SaveRunResult(
          savedDir: '',
          driveAction: 'error',
          ok: false,
        );
      }
      return SaveRunResult(
        savedDir: raw['savedDir']?.toString() ?? '',
        driveAction: raw['driveAction']?.toString() ?? 'local_only',
        ok: raw['ok'] == true,
      );
    } on MissingPluginException {
      return const SaveRunResult(
        savedDir: '',
        driveAction: 'unavailable',
        ok: false,
      );
    }
  }

  @override
  Future<bool> shareRunToDrive({String? runId}) async {
    try {
      final ok = await _channel.invokeMethod<bool>(
        'shareRunToDrive',
        {'runId': runId},
      );
      return ok ?? false;
    } on MissingPluginException {
      return false;
    }
  }

  @override
  Future<String> getWebhookUrl() async {
    try {
      final url = await _channel.invokeMethod<String>('getWebhookUrl');
      return url ?? '';
    } on MissingPluginException {
      return '';
    }
  }

  @override
  Future<bool> setWebhookUrl(String url) async {
    try {
      final ok = await _channel.invokeMethod<bool>(
        'setWebhookUrl',
        {'url': url},
      );
      return ok ?? false;
    } on MissingPluginException {
      return false;
    }
  }
}
