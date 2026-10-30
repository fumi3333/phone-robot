/// シミュレーションモードの画面（従来の骨格そのまま）。
/// 中核の Explorer / GoalNavigator を SimRobot で駆動し、地図が
/// 塗り上がる様子とタップでのゴール指定を確認できる。
import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:phone_robot_core/phone_robot_core.dart';

import 'map_painter.dart';
import 'sim_session.dart';

class SimPage extends StatefulWidget {
  const SimPage({super.key});

  @override
  State<SimPage> createState() => _SimPageState();
}

class _SimPageState extends State<SimPage> {
  final SimSession _session = SimSession();
  Timer? _timer;

  /// 探索アニメーション再生中のフレーム番号（再生中でなければ null）。
  int? _frameIndex;

  /// ゴールアニメーション中の表示ロボット位置。
  Cell? _animRobot;

  Cell? _goal;
  List<Cell>? _path;
  String _status = '「探索開始」を押すとシミュレーション探索が始まります';

  static const _params = SimSession.params;

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  Float32List get _displayGrid {
    final fi = _frameIndex;
    if (fi != null && _session.frames.isNotEmpty) {
      return _session.frames[fi.clamp(0, _session.frames.length - 1)];
    }
    return _session.grid.logOdds;
  }

  Cell get _displayRobot {
    final fi = _frameIndex;
    if (fi != null && _session.frameRobots.isNotEmpty) {
      return _session.frameRobots[fi.clamp(0, _session.frameRobots.length - 1)];
    }
    return _animRobot ?? _session.robot.cell;
  }

  void _startExploration() {
    _timer?.cancel();
    final result = _session.runExploration();
    setState(() {
      _goal = null;
      _path = null;
      _animRobot = null;
      _frameIndex = 0;
      _status = '探索中…（地図が塗り上がっていきます）';
    });
    _timer = Timer.periodic(const Duration(milliseconds: 90), (t) {
      final next = (_frameIndex ?? 0) + 1;
      if (next >= _session.frames.length) {
        t.cancel();
        setState(() {
          _frameIndex = null;
          _status =
              '探索完了: カバー率 '
              '${(_session.grid.coverage * 100).toStringAsFixed(1)}%'
              '（${result.steps} ステップ）。地図をタップするとゴール指定できます';
        });
      } else {
        setState(() => _frameIndex = next);
      }
    });
  }

  void _onTapMap(TapDownDetails details, Size size) {
    if (!_session.explored || _frameIndex != null) return;
    final cw = size.width / _params.gridW;
    final ch = size.height / _params.gridH;
    final c = (details.localPosition.dx / cw).floor();
    final rFromTop = (details.localPosition.dy / ch).floor();
    final r = _params.gridH - 1 - rFromTop;
    if (r < 0 || r >= _params.gridH || c < 0 || c >= _params.gridW) return;
    final goal = (r: r, c: c);
    final plan = _session.planTo(goal);
    setState(() {
      _goal = goal;
      _path = plan;
      _status = plan == null
          ? 'その場所は既知の空間内に経路がありません（未探索・障害物・到達不能）'
          : '経路を表示しました（${plan.length} セル）。「ゴールへ移動」で走行します';
    });
  }

  void _goToGoal() {
    final goal = _goal;
    if (goal == null || _path == null) return;
    _timer?.cancel();
    // 中核の GoalNavigator で実際に走行させ、通ったセル列をアニメーションする
    final result = _session.goTo(goal);
    final traversed = result.traversed;
    var i = 0;
    setState(() {
      _animRobot = traversed.first;
      _status = '移動中…';
    });
    _timer = Timer.periodic(const Duration(milliseconds: 60), (t) {
      i++;
      if (i >= traversed.length) {
        t.cancel();
        setState(() {
          _animRobot = traversed.last;
          _path = null;
          _status = result.success
              ? 'ゴールに到達しました（${result.steps} ステップ, '
                    '${result.pathLengthM.toStringAsFixed(2)} m）'
              : '到達できませんでした: ${result.failureReason}';
        });
      } else {
        setState(() => _animRobot = traversed[i]);
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('シミュレーションモード')),
      body: Column(
        children: [
          Padding(padding: const EdgeInsets.all(12), child: Text(_status)),
          Expanded(
            child: LayoutBuilder(
              builder: (context, constraints) {
                final size = Size(constraints.maxWidth, constraints.maxHeight);
                return GestureDetector(
                  onTapDown: (d) => _onTapMap(d, size),
                  child: CustomPaint(
                    size: size,
                    painter: MapPainter(
                      logOdds: _displayGrid,
                      gridW: _params.gridW,
                      gridH: _params.gridH,
                      robot: _displayRobot,
                      goal: _goal,
                      path: _path,
                    ),
                  ),
                );
              },
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(12),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                FilledButton(
                  onPressed: _startExploration,
                  child: const Text('探索開始'),
                ),
                const SizedBox(width: 12),
                FilledButton.tonal(
                  onPressed: _path == null ? null : _goToGoal,
                  child: const Text('ゴールへ移動'),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
