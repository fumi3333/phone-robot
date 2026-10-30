import 'dart:typed_data';

import 'package:phone_robot_core/phone_robot_core.dart';

/// アプリ骨格用のシミュレーションセッション。
///
/// 中核（phone_robot_core）の Explorer / GoalNavigator を SimRobot で
/// 駆動する。実機では SimRobot の代わりに、platform/ の PoseSource /
/// DepthSource 経由で ARCore + ToF を使う MotionHooks 実装へ差し替える。
class SimSession {
  static const MappingParams params = MappingParams();

  final OccupancyGrid grid;
  final SimRobot robot;

  /// 探索中の地図スナップショット（アニメーション再生用）。
  final List<Float32List> frames = [];

  /// 各スナップショット時点のロボット位置。
  final List<Cell> frameRobots = [];

  bool explored = false;

  SimSession._(this.grid, this.robot);

  factory SimSession() {
    final grid = OccupancyGrid(params);
    final robot = SimRobot(
      params: params,
      groundTruth: buildGroundTruth(params),
      grid: grid,
      rng: GaussianRandom(7),
      drift: false,
      start: cellOf(params, 0.80, 0.80),
    );
    return SimSession._(grid, robot);
  }

  /// 探索を最後まで実行し、途中のスナップショットを貯める。
  ExploreResult runExploration() {
    frames.clear();
    frameRobots.clear();
    final result = Explorer(params).explore(
      grid: grid,
      start: robot.cell,
      hooks: robot,
      onFrame: (snapshot, at) {
        if (frames.length < 400) {
          frames.add(snapshot);
          frameRobots.add(at);
        }
      },
    );
    frames.add(Float32List.fromList(grid.logOdds));
    frameRobots.add(robot.cell);
    explored = true;
    return result;
  }

  /// ゴールまでの計画経路（表示用）。到達不能なら null。
  List<Cell>? planTo(Cell goal) {
    if (!grid.inBounds(goal.r, goal.c)) return null;
    final gi = grid.index(goal.r, goal.c);
    if (grid.logOdds[gi] == 0 || !grid.isPlanFree(gi)) return null;
    final mask = grid.planMask(forcedFree: robot.cell);
    return findPath(
      gridW: params.gridW,
      gridH: params.gridH,
      isFree: (i) => mask[i] == 1,
      start: robot.cell,
      goal: goal,
    );
  }

  /// ゴール指定ナビゲーションを実行する（中核の GoalNavigator を使用）。
  GoToResult goTo(Cell goal) {
    return GoalNavigator(params)
        .goTo(grid: grid, start: robot.cell, goal: goal, hooks: robot);
  }
}
