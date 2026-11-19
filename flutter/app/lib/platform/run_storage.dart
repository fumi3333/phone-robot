import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

/// 走行軌跡の1点（ARCore グリッド座標）。
typedef TrajectoryPoint = ({int elapsedMs, double x, double y, double heading});

/// 送信した1つの命令（または同一指令の連続区間）に対する実車・センサー変化の要約。
class CommandEffectSegment {
  final int index;
  final int startElapsedMs;
  final int endElapsedMs;
  final int drivePwm;
  final int steerPwm;
  final String actionLabel;
  final double? startX;
  final double? startY;
  final double? startHeadingDeg;
  final double? endX;
  final double? endY;
  final double? endHeadingDeg;
  final double? dxM;
  final double? dyM;
  final double? distanceMovedM;
  final double? dHeadingDeg;
  final double? startFrontMinM;
  final double? endFrontMinM;
  final double? dFrontMinM;

  const CommandEffectSegment({
    required this.index,
    required this.startElapsedMs,
    required this.endElapsedMs,
    required this.drivePwm,
    required this.steerPwm,
    required this.actionLabel,
    this.startX,
    this.startY,
    this.startHeadingDeg,
    this.endX,
    this.endY,
    this.endHeadingDeg,
    this.dxM,
    this.dyM,
    this.distanceMovedM,
    this.dHeadingDeg,
    this.startFrontMinM,
    this.endFrontMinM,
    this.dFrontMinM,
  });

  int get durationMs => math.max(0, endElapsedMs - startElapsedMs);

  Map<String, Object?> toJson() => {
    'index': index,
    'startElapsedMs': startElapsedMs,
    'endElapsedMs': endElapsedMs,
    'durationMs': durationMs,
    'drivePwm': drivePwm,
    'steerPwm': steerPwm,
    'actionLabel': actionLabel,
    'startX': _roundOpt(startX, 4),
    'startY': _roundOpt(startY, 4),
    'startHeadingDeg': _roundOpt(startHeadingDeg, 2),
    'endX': _roundOpt(endX, 4),
    'endY': _roundOpt(endY, 4),
    'endHeadingDeg': _roundOpt(endHeadingDeg, 2),
    'dxM': _roundOpt(dxM, 4),
    'dyM': _roundOpt(dyM, 4),
    'distanceMovedM': _roundOpt(distanceMovedM, 4),
    'dHeadingDeg': _roundOpt(dHeadingDeg, 2),
    'startFrontMinM': _roundOpt(startFrontMinM, 3),
    'endFrontMinM': _roundOpt(endFrontMinM, 3),
    'dFrontMinM': _roundOpt(dFrontMinM, 3),
  };

  static double? _roundOpt(double? v, int digits) =>
      v == null ? null : double.parse(v.toStringAsFixed(digits));
}

String classifyCommandAction(int drivePwm, int steerPwm) {
  if (drivePwm == 0 && steerPwm == 0) return '停止';
  if (drivePwm == 0 && steerPwm < 0) return '先当て左';
  if (drivePwm == 0 && steerPwm > 0) return '先当て右';
  if (drivePwm > 0 && steerPwm < 0) return '左旋回';
  if (drivePwm > 0 && steerPwm > 0) return '右旋回';
  if (drivePwm > 0) return '前進';
  if (drivePwm < 0 && steerPwm < 0) return '後退左';
  if (drivePwm < 0 && steerPwm > 0) return '後退右';
  return '後退';
}

/// 走行終了時（または手動記録停止時）に出力される記録ファイル一式。
class RunBundle {
  final String runId;
  final String mode;
  final DateTime startedAt;
  final DateTime endedAt;
  final String csvContent;
  final String commandEffectsCsvContent;
  final List<CommandEffectSegment> commandEffects;
  final String jsonContent;
  final String summaryMarkdown;
  final Uint8List mapPngBytes;
  final int commandCount;
  final int scanCount;
  final double mappedAreaM2;
  final double coverage;
  final bool hasMp4Video;

  const RunBundle({
    required this.runId,
    required this.mode,
    required this.startedAt,
    required this.endedAt,
    required this.csvContent,
    required this.commandEffectsCsvContent,
    required this.commandEffects,
    required this.jsonContent,
    required this.summaryMarkdown,
    required this.mapPngBytes,
    required this.commandCount,
    required this.scanCount,
    required this.mappedAreaM2,
    required this.coverage,
    this.hasMp4Video = false,
  });

  /// MethodChannel 経由で Kotlin 側へ渡すファイルマップ。
  Map<String, Object> toChannelFiles() => {
    'driving_log_$runId.csv': csvContent,
    'command_effects_$runId.csv': commandEffectsCsvContent,
    'floor_map_$runId.png': mapPngBytes,
    'floor_map_$runId.json': jsonContent,
    'run_summary_$runId.md': summaryMarkdown,
  };
}

class _OpenCommandSegment {
  final int index;
  final int startElapsedMs;
  final int drivePwm;
  final int steerPwm;
  final Pose? startPose;
  final double? startFrontMinM;

  _OpenCommandSegment({
    required this.index,
    required this.startElapsedMs,
    required this.drivePwm,
    required this.steerPwm,
    required this.startPose,
    required this.startFrontMinM,
  });
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
  final List<CommandEffectSegment> _segments = [];
  _OpenCommandSegment? _openSegment;
  Pose? _latestKnownPose;
  double? _latestKnownFrontMinM;

  int _commandCount = 0;
  int _scanCount = 0;
  int _lastDrivePwm = 0;
  int _lastSteerPwm = 0;
  int _lastScanRecordedMs = -minScanIntervalMs;
  int _lastTrajectoryMs = -1000;
  double? _lastTrajectoryX;
  double? _lastTrajectoryY;

  RunRecorder({String? runId, required this.mode, DateTime? startedAt})
    : startedAt = startedAt ?? DateTime.now(),
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
  List<CommandEffectSegment> get commandEffects => List.unmodifiable(_segments);

  static double? frontMinDepthM(Scan? scan) {
    if (scan == null || scan.ranges.isEmpty) return null;
    final n = scan.ranges.length;
    final cStart = (n * 0.33).floor();
    final cEnd = (n * 0.67).ceil().clamp(cStart, n - 1);
    final vals = <double>[];
    for (var i = cStart; i <= cEnd; i++) {
      final r = scan.ranges[i];
      if (r.isFinite && r > 0) {
        vals.add(r);
      }
    }
    if (vals.isEmpty) return null;
    vals.sort();
    // 1本だけの反射スパイクや床ノイズによる誤停止を防ぐため、2番目に近い値を採用する
    return vals.length >= 2 ? vals[1] : vals[0];
  }

  static double _wrapDegDiff(double radDiff) {
    var r = radDiff % (2 * math.pi);
    if (r > math.pi) r -= 2 * math.pi;
    if (r <= -math.pi) r += 2 * math.pi;
    return r * 180.0 / math.pi;
  }

  static double _wrapDeg(double rad) => _wrapDegDiff(rad);

  void _closeOpenSegment(
    int endElapsedMs,
    Pose? endPose,
    double? endFrontMinM,
  ) {
    final seg = _openSegment;
    if (seg == null) return;
    _openSegment = null;
    final sp = seg.startPose;
    final ep = endPose ?? sp;
    final sf = seg.startFrontMinM;
    final ef = endFrontMinM ?? sf;

    final double? dxM = (sp != null && ep != null) ? ep.x - sp.x : null;
    final double? dyM = (sp != null && ep != null) ? ep.y - sp.y : null;
    final double? distM = (dxM != null && dyM != null)
        ? math.sqrt(dxM * dxM + dyM * dyM)
        : null;
    final double? dHeadingDeg = (sp != null && ep != null)
        ? _wrapDegDiff(ep.heading - sp.heading)
        : null;
    final double? dFrontM = (sf != null && ef != null) ? ef - sf : null;

    _segments.add(
      CommandEffectSegment(
        index: seg.index,
        startElapsedMs: seg.startElapsedMs,
        endElapsedMs: math.max(seg.startElapsedMs, endElapsedMs),
        drivePwm: seg.drivePwm,
        steerPwm: seg.steerPwm,
        actionLabel: classifyCommandAction(seg.drivePwm, seg.steerPwm),
        startX: sp?.x,
        startY: sp?.y,
        startHeadingDeg: sp == null ? null : _wrapDeg(sp.heading),
        endX: ep?.x,
        endY: ep?.y,
        endHeadingDeg: ep == null ? null : _wrapDeg(ep.heading),
        dxM: dxM,
        dyM: dyM,
        distanceMovedM: distM,
        dHeadingDeg: dHeadingDeg,
        startFrontMinM: sf,
        endFrontMinM: ef,
        dFrontMinM: dFrontM,
      ),
    );
  }

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

    final now = at ?? DateTime.now();
    final elapsedMs = now.difference(startedAt).inMilliseconds;
    final effPose = pose ?? latestScan?.pose ?? _latestKnownPose;
    final effFrontMin = frontMinDepthM(latestScan) ?? _latestKnownFrontMinM;
    if (effPose != null) _latestKnownPose = effPose;
    if (effFrontMin != null) _latestKnownFrontMinM = effFrontMin;

    final open = _openSegment;
    if (open == null ||
        open.drivePwm != drivePwm ||
        open.steerPwm != steerPwm) {
      if (open != null) {
        _closeOpenSegment(elapsedMs, effPose, effFrontMin);
      }
      _openSegment = _OpenCommandSegment(
        index: _segments.length + 1,
        startElapsedMs: elapsedMs,
        drivePwm: drivePwm,
        steerPwm: steerPwm,
        startPose: effPose,
        startFrontMinM: effFrontMin,
      );
    }

    if (rowCount >= maxRows) return;

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

  void recordScan(Scan scan, {required String trackingState, DateTime? at}) {
    final now = at ?? DateTime.now();
    final elapsedMs = now.difference(startedAt).inMilliseconds;
    _latestKnownPose = scan.pose;
    final fMin = frontMinDepthM(scan);
    if (fMin != null) _latestKnownFrontMinM = fMin;
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
    final distSq =
        (pose.x - lx) * (pose.x - lx) + (pose.y - ly) * (pose.y - ly);
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

  static String _fmtOpt(double? v, int digits) =>
      v == null ? '' : v.toStringAsFixed(digits);

  static String _buildCommandEffectsCsv(List<CommandEffectSegment> segments) {
    final buf = StringBuffer()
      ..writeln(
        'index,start_elapsed_ms,end_elapsed_ms,duration_ms,'
        'drive_pwm,steer_pwm,action_label,'
        'start_x_m,start_y_m,start_heading_deg,'
        'end_x_m,end_y_m,end_heading_deg,'
        'dx_m,dy_m,distance_moved_m,d_heading_deg,'
        'start_front_min_m,end_front_min_m,d_front_min_m',
      );
    for (final s in segments) {
      buf.writeln(
        '${s.index},${s.startElapsedMs},${s.endElapsedMs},${s.durationMs},'
        '${s.drivePwm},${s.steerPwm},${_csvCell(s.actionLabel)},'
        '${_fmtOpt(s.startX, 4)},${_fmtOpt(s.startY, 4)},${_fmtOpt(s.startHeadingDeg, 2)},'
        '${_fmtOpt(s.endX, 4)},${_fmtOpt(s.endY, 4)},${_fmtOpt(s.endHeadingDeg, 2)},'
        '${_fmtOpt(s.dxM, 4)},${_fmtOpt(s.dyM, 4)},${_fmtOpt(s.distanceMovedM, 4)},'
        '${_fmtOpt(s.dHeadingDeg, 2)},'
        '${_fmtOpt(s.startFrontMinM, 3)},${_fmtOpt(s.endFrontMinM, 3)},'
        '${_fmtOpt(s.dFrontMinM, 3)}',
      );
    }
    return buf.toString();
  }

  RunBundle buildBundle({
    required OccupancyGrid grid,
    required MappingParams params,
    required double speedNorm,
    required int currentDrivePwm,
    required bool invertSteer,
    Cell? robotCell,
    Pose? latestPose,
    Scan? latestScan,
    bool hasMp4Video = false,
    DateTime? endedAt,
  }) {
    final end = endedAt ?? DateTime.now();
    final endElapsedMs = math.max(0, end.difference(startedAt).inMilliseconds);
    final closingPose = latestPose ?? latestScan?.pose ?? _latestKnownPose;
    final closingFrontMin = frontMinDepthM(latestScan) ?? _latestKnownFrontMinM;
    if (_openSegment != null) {
      _closeOpenSegment(endElapsedMs, closingPose, closingFrontMin);
    }
    final segments = List<CommandEffectSegment>.unmodifiable(_segments);
    final effectsCsv = _buildCommandEffectsCsv(segments);

    final durationSec = math.max(0.0, endElapsedMs / 1000.0);
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
      'hasMp4Video': hasMp4Video,
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
        'commandSegmentCount': segments.length,
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
      'commandEffects': [for (final seg in segments) seg.toJson()],
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
      ..writeln(
        '- 終了時刻: ${end.toIso8601String()}（${durationSec.toStringAsFixed(1)} 秒）',
      )
      ..writeln(
        '- 対象スケール: ${params.roomW.toStringAsFixed(1)} m × '
        '${params.roomH.toStringAsFixed(1)} m'
        '（グリッド ${params.gridW}×${params.gridH}、セル '
        '${(params.cellSize * 100).round()} cm）',
      )
      ..writeln(
        '- マッピング面積: ${mappedAreaM2.toStringAsFixed(2)} m²'
        '（カバー率 ${(coverage * 100).toStringAsFixed(1)}%）',
      )
      ..writeln(
        '- 記録件数: コマンド $_commandCount 件（命令区間 ${segments.length} 件） / '
        '深度スキャン $_scanCount 件 / 軌跡 ${_trajectory.length} 点',
      )
      ..writeln(
        '- 走行設定: 速度 ${(speedNorm * 100).round()}% '
        '(PWM $currentDrivePwm), 操舵反転 ${invertSteer ? 'ON' : 'OFF'}',
      )
      ..writeln(
        '- カメラMP4録画: ${hasMp4Video ? 'camera_recording_$runId.mp4 保存済み' : 'なし'}',
      )
      ..writeln()
      ..writeln('## 出力ファイル')
      ..writeln('- `driving_log_$runId.csv`: 時系列操作コマンド・ARCore自己位置・21本深度レイ')
      ..writeln('- `command_effects_$runId.csv`: 命令送信ごとの位置・向き・前方距離の変化一覧')
      ..writeln('- `floor_map_$runId.png`: 占有格子地図と走行軌跡画像')
      ..writeln('- `floor_map_$runId.json`: グリッド数値データ・軌跡・命令効果JSON');

    if (hasMp4Video) {
      md.writeln('- `camera_recording_$runId.mp4`: ARCore カメラ映像＋センサー動画');
    }
    if (segments.isNotEmpty) {
      md
        ..writeln()
        ..writeln('## コマンド別の車体・センサー変化一覧（命令→結果）')
        ..writeln()
        ..writeln(
          '| # | 時間 [ms] | 操作 | PWM (駆動,操舵) | 移動距離 [m] | 向き変化 [°] | 前方距離変化 [m] |',
        )
        ..writeln('|---:|---:|---|---|---:|---:|---:|');
      for (final s in segments) {
        final mv = s.distanceMovedM == null
            ? '—'
            : s.distanceMovedM!.toStringAsFixed(2);
        final dh = s.dHeadingDeg == null
            ? '—'
            : s.dHeadingDeg!.toStringAsFixed(1);
        final df = s.dFrontMinM == null
            ? '—'
            : s.dFrontMinM!.toStringAsFixed(2);
        md.writeln(
          '| ${s.index} | ${s.durationMs} | ${s.actionLabel} | '
          '(${s.drivePwm}, ${s.steerPwm}) | $mv | $dh | $df |',
        );
      }
    }

    return RunBundle(
      runId: runId,
      mode: mode,
      startedAt: startedAt,
      endedAt: end,
      csvContent: _csv.toString(),
      commandEffectsCsvContent: effectsCsv,
      commandEffects: segments,
      jsonContent: jsonEncode(jsonPayload),
      summaryMarkdown: md.toString(),
      mapPngBytes: mapPngBytes,
      commandCount: _commandCount,
      scanCount: _scanCount,
      mappedAreaM2: mappedAreaM2,
      coverage: coverage,
      hasMp4Video: hasMp4Video,
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
        final steps = math.max((cur.r - prev.r).abs(), (cur.c - prev.c).abs());
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

  final endCell =
      robotCell ??
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
      final raw = await _channel
          .invokeMethod<Map<dynamic, dynamic>>('saveRunBundle', {
            'runId': bundle.runId,
            'files': bundle.toChannelFiles(),
            'autoDriveSync': autoDriveSync,
            'openDriveSheet': openDriveSheet,
          });
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
      final ok = await _channel.invokeMethod<bool>('shareRunToDrive', {
        'runId': runId,
      });
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
      final ok = await _channel.invokeMethod<bool>('setWebhookUrl', {
        'url': url,
      });
      return ok ?? false;
    } on MissingPluginException {
      return false;
    }
  }
}
