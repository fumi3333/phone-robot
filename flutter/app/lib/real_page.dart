import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';

import 'map_painter.dart';
import 'platform/serial_link.dart';
import 'real/real_session.dart';

/// 実機モードの画面。
///
/// ARCore（自己位置＋深度）と ESP32（USB シリアル）の状態を表示し、
/// VFH 自律クルーズ／探索／ゴール指定ナビを操作する。
/// 安全のため、実機走行は必ず監視下・低速で行うこと（README 参照）。
class RealPage extends StatefulWidget {
  const RealPage({super.key});

  @override
  State<RealPage> createState() => _RealPageState();
}

class _RealPageState extends State<RealPage> with WidgetsBindingObserver {
  bool _floorScale = false;
  late RealSession _session;
  StreamSubscription<void>? _sub;
  Timer? _jogTimer;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _initSession(autoStart: true);
  }

  void _initSession({
    required bool autoStart,
    double? speedNorm,
    bool? invertSteer,
    bool? autoDriveSync,
  }) {
    final params =
        _floorScale ? RealSession.floorParams : RealSession.roomParams;
    _session = RealSession(customParams: params);
    if (speedNorm != null) _session.setSpeedNorm(speedNorm);
    if (invertSteer != null) _session.setInvertSteer(invertSteer);
    if (autoDriveSync != null) _session.setAutoDriveSync(autoDriveSync);
    _sub = _session.updates.listen((_) {
      if (mounted) setState(() {});
    });
    if (autoStart) {
      unawaited(_session.autoSetup());
    }
  }

  Future<void> _switchScale(bool floorMode) async {
    if (_session.busy || _floorScale == floorMode) return;
    final prevSpeed = _session.speedNorm;
    final prevInvert = _session.invertSteer;
    final prevAutoDrive = _session.autoDriveSync;
    _stopJog(sendStop: false);
    await _sub?.cancel();
    _sub = null;
    try {
      await _session.disconnectSerial();
      await _session.stopSensors();
    } catch (_) {
    }
    _session.dispose();
    if (!mounted) return;
    setState(() {
      _floorScale = floorMode;
      _initSession(
        autoStart: true,
        speedNorm: prevSpeed,
        invertSteer: prevInvert,
        autoDriveSync: prevAutoDrive,
      );
    });
  }

  @override
  void dispose() {
    _stopJog(sendStop: false);
    WidgetsBinding.instance.removeObserver(this);
    _sub?.cancel();
    _session.dispose();
    super.dispose();
  }

  /// R4: 背面に移ったら走行を止め、復帰したら状態の再評価を促す。
  /// 走行の自動再開はしない（再開時はドライバが現在位置から再計画）。
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.hidden) {
      _stopJog(sendStop: _session.linkState == Esp32LinkState.connected);
      _session.handleAppPaused();
    } else if (state == AppLifecycleState.resumed) {
      _session.handleAppResumed();
    }
  }

  void _onTapMap(TapDownDetails details, Size size) {
    if (!_session.explored || _session.busy) return;
    final params = _session.params;
    final cw = size.width / params.gridW;
    final ch = size.height / params.gridH;
    final c = (details.localPosition.dx / cw).floor();
    final rFromTop = (details.localPosition.dy / ch).floor();
    final r = params.gridH - 1 - rFromTop;
    if (r < 0 || r >= params.gridH || c < 0 || c >= params.gridW) return;
    _session.selectGoal((r: r, c: c));
  }

  String get _linkLabel => switch (_session.linkState) {
        Esp32LinkState.connected => '接続済み',
        Esp32LinkState.connecting => '接続中…',
        Esp32LinkState.error => 'エラー',
        Esp32LinkState.disconnected => '未接続',
      };

  String get _poseLabel {
    final p = _session.latestPose;
    if (p == null) return '—';
    final deg = p.heading * 180 / math.pi;
    return 'x=${p.x.toStringAsFixed(2)}m y=${p.y.toStringAsFixed(2)}m '
        '${deg.toStringAsFixed(0)}°';
  }

  void _startJog(String ch) {
    _jogTimer?.cancel();
    unawaited(_session.jog(ch));
    if (ch == 's') return;
    _jogTimer = Timer.periodic(
      const Duration(milliseconds: 100),
      (_) => unawaited(_session.jog(ch)),
    );
  }

  void _stopJog({bool sendStop = true}) {
    final hadTimer = _jogTimer != null;
    _jogTimer?.cancel();
    _jogTimer = null;
    if (sendStop && hadTimer) {
      unawaited(_session.jog('s'));
    }
  }

  Widget _smallToggle(String label, bool active, VoidCallback? onPressed) {
    final bg = active ? const Color(0xFF111827) : Colors.white;
    final fg = active ? Colors.white : const Color(0xFF374151);
    return OutlinedButton(
      onPressed: onPressed,
      style: OutlinedButton.styleFrom(
        backgroundColor: bg,
        foregroundColor: fg,
        side: BorderSide(
          color: active ? const Color(0xFF111827) : const Color(0xFFD1D5DB),
        ),
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
        minimumSize: const Size(0, 32),
        tapTargetSize: MaterialTapTargetSize.shrinkWrap,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
        textStyle: const TextStyle(fontSize: 12, fontWeight: FontWeight.w500),
      ),
      child: Text(label),
    );
  }

  Widget _jogButton(String label, String ch) {
    final connected = _session.linkState == Esp32LinkState.connected;
    final buttonStyle = OutlinedButton.styleFrom(
      foregroundColor: const Color(0xFF111827),
      side: const BorderSide(color: Color(0xFFD1D5DB)),
      padding: const EdgeInsets.symmetric(vertical: 10),
      minimumSize: const Size(0, 42),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(6)),
      textStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600),
    );
    if (!connected) {
      return OutlinedButton(
        onPressed: null,
        style: buttonStyle,
        child: Text(label),
      );
    }
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTapDown: (_) => _startJog(ch),
      onTapUp: (_) => _stopJog(sendStop: ch != 's'),
      onTapCancel: () => _stopJog(sendStop: ch != 's'),
      child: IgnorePointer(
        child: OutlinedButton(
          onPressed: () {},
          style: buttonStyle,
          child: Text(label),
        ),
      ),
    );
  }

  String get _recordLabel {
    if (_session.isRecording) {
      return '記録中 (${_session.recordedRowCount}件)';
    }
    final latest = _session.lastSavedRunId;
    return latest == null
        ? '待機中 (${_session.lastSaveStatus})'
        : '待機中 (最新: $latest / ${_session.lastSaveStatus})';
  }

  Future<void> _editWebhookUrl() async {
    final controller = TextEditingController(text: _session.driveWebhookUrl);
    final result = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('自動ドライブ転送 URL', style: TextStyle(fontSize: 16)),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'scripts/google_drive_webhook.gs をデプロイした URL を設定すると、'
              '走行終了時に画面操作なしでドライブへ自動保存します。空欄時は共有画面を開きます。',
              style: TextStyle(fontSize: 12, color: Color(0xFF4B5563)),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: controller,
              decoration: const InputDecoration(
                hintText: 'https://script.google.com/macros/s/.../exec',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              style: const TextStyle(fontSize: 12),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(''),
            child: const Text('クリア'),
          ),
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(),
            child: const Text('キャンセル'),
          ),
          FilledButton(
            onPressed: () => Navigator.of(ctx).pop(controller.text),
            child: const Text('保存'),
          ),
        ],
      ),
    );
    controller.dispose();
    if (result != null && mounted) {
      await _session.updateDriveWebhookUrl(result);
    }
  }

  Widget _buildInfoBox({required EdgeInsetsGeometry margin}) {
    return Container(
      margin: margin,
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
      decoration: BoxDecoration(
        color: const Color(0xFFF9FAFB),
        border: Border.all(color: const Color(0xFFE5E7EB)),
        borderRadius: BorderRadius.circular(6),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(
            'ARCore: ${_session.trackingState}  |  '
            'センサー: ${_session.sensorHealth}  |  ESP32: $_linkLabel',
            style: const TextStyle(
              fontSize: 12,
              color: Color(0xFF374151),
            ),
          ),
          const SizedBox(height: 2),
          Text(
            '位置: $_poseLabel  |  '
            '面積: ${_session.mappedAreaM2.toStringAsFixed(1)}m² '
            '(${(_session.grid.coverage * 100).toStringAsFixed(1)}%)  |  '
            '指令: ${_session.link.lastCommand}',
            style: const TextStyle(
              fontSize: 12,
              color: Color(0xFF374151),
            ),
          ),
          const SizedBox(height: 2),
          Text(
            '記録: $_recordLabel',
            style: const TextStyle(
              fontSize: 12,
              color: Color(0xFF374151),
            ),
          ),
          const SizedBox(height: 4),
          Text(
            _session.status,
            style: const TextStyle(
              fontSize: 13,
              fontWeight: FontWeight.w600,
              color: Color(0xFF111827),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapWidget() {
    final params = _session.params;
    final robot =
        _session.robotCell ?? (r: params.gridH ~/ 2, c: params.gridW ~/ 2);
    return LayoutBuilder(
      builder: (context, constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        return GestureDetector(
          onTapDown: (d) => _onTapMap(d, size),
          child: CustomPaint(
            size: size,
            painter: MapPainter(
              logOdds: _session.grid.logOdds,
              gridW: params.gridW,
              gridH: params.gridH,
              robot: robot,
              goal: _session.goal,
              path: _session.path,
              robotPose: _session.latestPose,
              latestScan: _session.latestScan,
              targetHeading: _session.hudTargetHeading,
              targetDistM: _session.hudTargetDistM,
              mappingParams: params,
            ),
          ),
        );
      },
    );
  }

  Widget _buildControlPanel(
    BuildContext context, {
    required EdgeInsetsGeometry padding,
  }) {
    final compactShape =
        RoundedRectangleBorder(borderRadius: BorderRadius.circular(6));
    return Padding(
      padding: padding,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Wrap(
            spacing: 4,
            runSpacing: 4,
            alignment: WrapAlignment.spaceBetween,
            children: [
              Wrap(
                spacing: 4,
                runSpacing: 4,
                children: [
                  _smallToggle(
                    '6×5m',
                    !_floorScale,
                    _session.busy ? null : () => unawaited(_switchScale(false)),
                  ),
                  _smallToggle(
                    '24×24m',
                    _floorScale,
                    _session.busy ? null : () => unawaited(_switchScale(true)),
                  ),
                  _smallToggle(
                    '操舵反転',
                    _session.invertSteer,
                    () => _session.setInvertSteer(!_session.invertSteer),
                  ),
                  _smallToggle(
                    _session.isRecording ? '記録保存' : '手動記録',
                    _session.isRecording,
                    _session.busy
                        ? null
                        : () => unawaited(_session.toggleManualRecording()),
                  ),
                  _smallToggle(
                    '自動ドライブ',
                    _session.autoDriveSync,
                    () => _session.setAutoDriveSync(!_session.autoDriveSync),
                  ),
                  _smallToggle(
                    'ドライブ送信',
                    false,
                    _session.lastSavedRunId == null
                        ? null
                        : () => unawaited(_session.shareLatestToDrive()),
                  ),
                  _smallToggle(
                    'URL設定',
                    _session.driveWebhookUrl.isNotEmpty,
                    () => unawaited(_editWebhookUrl()),
                  ),
                ],
              ),
              Wrap(
                spacing: 4,
                runSpacing: 4,
                children: [
                  _smallToggle(
                    'センサー',
                    _session.sensorsOn,
                    _session.sensorsOn
                        ? () => _session.stopSensors()
                        : () => _session.startSensors(),
                  ),
                  _smallToggle(
                    'ESP32',
                    _session.linkState == Esp32LinkState.connected,
                    _session.linkState == Esp32LinkState.connected
                        ? () => _session.disconnectSerial()
                        : () => _session.connectSerial(),
                  ),
                ],
              ),
            ],
          ),
          Row(
            children: [
              SizedBox(
                width: 128,
                child: Text(
                  '速度 ${(_session.speedNorm * 100).round()}% '
                  '(PWM ${_session.currentDrivePwm})',
                  style: const TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w600,
                    color: Color(0xFF111827),
                  ),
                ),
              ),
              Expanded(
                child: SliderTheme(
                  data: SliderTheme.of(context).copyWith(
                    trackHeight: 3,
                    thumbShape: const RoundSliderThumbShape(
                      enabledThumbRadius: 7,
                    ),
                    overlayShape: const RoundSliderOverlayShape(
                      overlayRadius: 14,
                    ),
                  ),
                  child: Slider(
                    value: _session.speedNorm.clamp(0.10, 1.0),
                    min: 0.10,
                    max: 1.0,
                    divisions: 18,
                    activeColor: const Color(0xFF111827),
                    inactiveColor: const Color(0xFFD1D5DB),
                    onChanged: (v) => _session.setSpeedNorm(v),
                  ),
                ),
              ),
            ],
          ),
          Row(
            children: [
              Expanded(
                child: FilledButton(
                  style: FilledButton.styleFrom(
                    backgroundColor: const Color(0xFF111827),
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(horizontal: 4),
                    minimumSize: const Size(0, 40),
                    shape: compactShape,
                    textStyle: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                  onPressed:
                      _session.busy ? null : () => _session.runAutoCruise(),
                  child: const FittedBox(
                    fit: BoxFit.scaleDown,
                    child: Text('マップ作成', maxLines: 1),
                  ),
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: OutlinedButton(
                  style: OutlinedButton.styleFrom(
                    foregroundColor: const Color(0xFF111827),
                    side: const BorderSide(color: Color(0xFFD1D5DB)),
                    padding: const EdgeInsets.symmetric(horizontal: 4),
                    minimumSize: const Size(0, 40),
                    shape: compactShape,
                    textStyle: const TextStyle(fontSize: 12),
                  ),
                  onPressed:
                      _session.busy ? null : () => _session.runExploration(),
                  child: const FittedBox(
                    fit: BoxFit.scaleDown,
                    child: Text('A*探索', maxLines: 1),
                  ),
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: OutlinedButton(
                  style: OutlinedButton.styleFrom(
                    foregroundColor: const Color(0xFF111827),
                    side: const BorderSide(color: Color(0xFFD1D5DB)),
                    padding: const EdgeInsets.symmetric(horizontal: 4),
                    minimumSize: const Size(0, 40),
                    shape: compactShape,
                    textStyle: const TextStyle(fontSize: 12),
                  ),
                  onPressed: _session.path == null || _session.busy
                      ? null
                      : () => _session.goToGoal(),
                  child: const FittedBox(
                    fit: BoxFit.scaleDown,
                    child: Text('ゴール移動', maxLines: 1),
                  ),
                ),
              ),
              const SizedBox(width: 6),
              Expanded(
                child: FilledButton(
                  style: FilledButton.styleFrom(
                    backgroundColor: const Color(0xFFDC2626),
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(horizontal: 4),
                    minimumSize: const Size(0, 40),
                    shape: compactShape,
                    textStyle: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                  onPressed: () => _session.emergencyStop(),
                  child: const FittedBox(
                    fit: BoxFit.scaleDown,
                    child: Text('停止', maxLines: 1),
                  ),
                ),
              ),
            ],
          ),
          const SizedBox(height: 6),
          Row(
            children: [
              Expanded(child: _jogButton('前', 'f')),
              const SizedBox(width: 6),
              Expanded(child: _jogButton('左', 'l')),
              const SizedBox(width: 6),
              Expanded(child: _jogButton('止', 's')),
              const SizedBox(width: 6),
              Expanded(child: _jogButton('右', 'r')),
              const SizedBox(width: 6),
              Expanded(child: _jogButton('後', 'b')),
            ],
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        top: false,
        child: LayoutBuilder(
          builder: (context, constraints) {
            final isLandscape = constraints.maxWidth > constraints.maxHeight;
            if (isLandscape) {
              return Row(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Expanded(child: _buildMapWidget()),
                  const VerticalDivider(width: 1, color: Color(0xFFE5E7EB)),
                  SizedBox(
                    width: 336,
                    child: SingleChildScrollView(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.stretch,
                        children: [
                          _buildInfoBox(
                            margin: const EdgeInsets.fromLTRB(10, 8, 10, 4),
                          ),
                          _buildControlPanel(
                            context,
                            padding: const EdgeInsets.fromLTRB(10, 4, 10, 10),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              );
            }
            return Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                _buildInfoBox(margin: const EdgeInsets.fromLTRB(12, 4, 12, 6)),
                Expanded(child: _buildMapWidget()),
                const Divider(height: 1, color: Color(0xFFE5E7EB)),
                _buildControlPanel(
                  context,
                  padding: const EdgeInsets.fromLTRB(12, 8, 12, 10),
                ),
              ],
            );
          },
        ),
      ),
    );
  }
}
