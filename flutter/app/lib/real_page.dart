import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';

import 'map_painter.dart';
import 'platform/serial_link.dart';
import 'real/real_session.dart';

/// 実機モードの画面。
///
/// ARCore（自己位置＋深度）と ESP32（USB シリアル）の状態を表示し、
/// RealDriver による実機探索／ゴール指定ナビを操作する。
/// 安全のため、実機走行は必ず監視下・低速で行うこと（README 参照）。
class RealPage extends StatefulWidget {
  const RealPage({super.key});

  @override
  State<RealPage> createState() => _RealPageState();
}

class _RealPageState extends State<RealPage> with WidgetsBindingObserver {
  final RealSession _session = RealSession();
  StreamSubscription<void>? _sub;

  static const _params = RealSession.params;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _sub = _session.updates.listen((_) {
      if (mounted) setState(() {});
    });
  }

  @override
  void dispose() {
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
      _session.handleAppPaused();
    } else if (state == AppLifecycleState.resumed) {
      _session.handleAppResumed();
    }
  }

  void _onTapMap(TapDownDetails details, Size size) {
    if (!_session.explored || _session.busy) return;
    final cw = size.width / _params.gridW;
    final ch = size.height / _params.gridH;
    final c = (details.localPosition.dx / cw).floor();
    final rFromTop = (details.localPosition.dy / ch).floor();
    final r = _params.gridH - 1 - rFromTop;
    if (r < 0 || r >= _params.gridH || c < 0 || c >= _params.gridW) return;
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
    if (p == null) return '（なし）';
    final deg = p.heading * 180 / math.pi;
    return 'x ${p.x.toStringAsFixed(2)} m / y ${p.y.toStringAsFixed(2)} m / '
        '${deg.toStringAsFixed(0)}°';
  }

  Widget _statusRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 2),
      child: Row(
        children: [
          SizedBox(
            width: 92,
            child: Text(label,
                style: const TextStyle(fontWeight: FontWeight.bold)),
          ),
          Expanded(child: Text(value)),
        ],
      ),
    );
  }

  Widget _jogButton(String label, String ch) {
    return OutlinedButton(
      onPressed: _session.linkState == Esp32LinkState.connected
          ? () => _session.jog(ch)
          : null,
      child: Text(label),
    );
  }

  @override
  Widget build(BuildContext context) {
    final robot = _session.robotCell ?? (r: 16, c: 16);
    return Scaffold(
      appBar: AppBar(title: const Text('実機モード（ARCore＋実車）')),
      body: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          _statusRow('ARCore', _session.trackingState),
          _statusRow('センサー', _session.sensorHealth),
          _statusRow('ESP32', _linkLabel),
          _statusRow('自己位置', _poseLabel),
          _statusRow('現在の指令', _session.link.lastCommand),
          _statusRow('カバー率',
              '${(_session.grid.coverage * 100).toStringAsFixed(1)}%'),
          Padding(
            padding: const EdgeInsets.all(12),
            child: Text(_session.status),
          ),
          Expanded(
            child: LayoutBuilder(
              builder: (context, constraints) {
                final size = Size(constraints.maxWidth, constraints.maxHeight);
                return GestureDetector(
                  onTapDown: (d) => _onTapMap(d, size),
                  child: CustomPaint(
                    size: size,
                    painter: MapPainter(
                      logOdds: _session.grid.logOdds,
                      gridW: _params.gridW,
                      gridH: _params.gridH,
                      robot: robot,
                      goal: _session.goal,
                      path: _session.path,
                    ),
                  ),
                );
              },
            ),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(12, 4, 12, 0),
            child: Wrap(
              spacing: 8,
              runSpacing: 4,
              alignment: WrapAlignment.center,
              children: [
                FilledButton.tonal(
                  onPressed: _session.sensorsOn
                      ? () => _session.stopSensors()
                      : () => _session.startSensors(),
                  child: Text(_session.sensorsOn ? 'センサー停止' : 'センサー開始'),
                ),
                FilledButton.tonal(
                  onPressed: _session.linkState == Esp32LinkState.connected
                      ? () => _session.disconnectSerial()
                      : () => _session.connectSerial(),
                  child: Text(
                      _session.linkState == Esp32LinkState.connected
                          ? 'ESP32 切断'
                          : 'ESP32 接続'),
                ),
                FilledButton(
                  onPressed: _session.busy
                      ? null
                      : () => _session.runExploration(),
                  child: const Text('探索開始（実機）'),
                ),
                FilledButton.tonal(
                  onPressed: _session.path == null || _session.busy
                      ? null
                      : () => _session.goToGoal(),
                  child: const Text('ゴールへ移動'),
                ),
                FilledButton(
                  style: FilledButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                  ),
                  onPressed: () => _session.emergencyStop(),
                  child: const Text('停止'),
                ),
              ],
            ),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(12, 4, 12, 12),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                const Text('手動: '),
                _jogButton('前', 'f'),
                const SizedBox(width: 4),
                _jogButton('左', 'l'),
                const SizedBox(width: 4),
                _jogButton('止', 's'),
                const SizedBox(width: 4),
                _jogButton('右', 'r'),
                const SizedBox(width: 4),
                _jogButton('後', 'b'),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
