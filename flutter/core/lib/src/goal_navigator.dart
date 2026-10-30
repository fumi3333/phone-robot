import 'dart:math' as math;
import 'dart:typed_data';

import 'astar.dart';
import 'explorer.dart';
import 'occupancy_grid.dart';
import 'params.dart';
import 'pose.dart';

/// ゴール指定ナビゲーションの結果。
class GoToResult {
  final bool success;

  /// 実際に移動したステップ数（セル数）。
  final int steps;

  /// 辿った経路の長さ [m]（斜めは √2 セル分）。
  final double pathLengthM;

  /// 塞がれて再計画した回数。
  final int replans;

  /// 失敗理由（成功時は null）。
  /// 'goal-out-of-bounds' / 'goal-unknown' / 'goal-occupied' /
  /// 'unreachable' / 'blocked-no-progress' / 'too-many-replans' /
  /// 'step-limit'
  final String? failureReason;

  /// 実際に通ったセル列（start を含む）。
  final List<Cell> traversed;

  const GoToResult({
    required this.success,
    required this.steps,
    required this.pathLengthM,
    required this.replans,
    required this.failureReason,
    required this.traversed,
  });
}

/// ゴール指定ナビゲーション。
///
/// 探索で作った地図の上で、指定した任意の点まで「既知の空間内だけ」を
/// 通って移動する。A* のゴールをフロンティアから指定点に差し替えたもの。
/// 未知セル・占有セルは通らない（安全側）。塞がれた場合は再計画し、
/// それでも進めなければ失敗を正直に返す（空転防止ガード付き）。
class GoalNavigator {
  final MappingParams params;

  const GoalNavigator(this.params);

  GoToResult goTo({
    required OccupancyGrid grid,
    required Cell start,
    required Cell goal,
    required MotionHooks hooks,
    int maxSteps = 2000,
    void Function(Float32List snapshot, Cell robot)? onFrame,
  }) {
    final p = params;
    final traversed = <Cell>[start];
    var steps = 0;
    var replans = 0;
    var lengthM = 0.0;

    GoToResult fail(String reason) => GoToResult(
          success: false,
          steps: steps,
          pathLengthM: lengthM,
          replans: replans,
          failureReason: reason,
          traversed: traversed,
        );

    if (!grid.inBounds(goal.r, goal.c)) return fail('goal-out-of-bounds');
    final goalIdx = grid.index(goal.r, goal.c);
    if (grid.logOdds[goalIdx] == 0) {
      // 未探索の場所は「行ける」と言えないので失敗として返す（安全側）
      return fail('goal-unknown');
    }
    if (!grid.isPlanFree(goalIdx)) return fail('goal-occupied');

    var current = start;
    var noProgressPlans = 0;

    while (current != goal) {
      if (steps >= maxSteps) return fail('step-limit');
      final mask = grid.planMask(forcedFree: current);
      final path = findPath(
        gridW: p.gridW,
        gridH: p.gridH,
        isFree: (i) => mask[i] == 1,
        start: current,
        goal: goal,
      );
      if (path == null || path.length < 2) {
        return fail('unreachable');
      }

      var movedThisPlan = 0;
      var blocked = false;
      for (var pi = 1; pi < path.length; pi++) {
        if (steps >= maxSteps) break;
        final next = path[pi];
        final diagonal = next.r != current.r && next.c != current.c;
        if (!hooks.moveTo(next)) {
          // フェイルセーフで拒否（セルは占有として記録済み）。再計画へ。
          blocked = true;
          break;
        }
        lengthM += (diagonal ? math.sqrt2 : 1.0) * p.cellSize;
        current = next;
        traversed.add(current);
        steps++;
        movedThisPlan++;
        hooks.scan(hooks.heading);
        if (onFrame != null && steps % 10 == 0) {
          onFrame(Float32List.fromList(grid.logOdds), current);
        }
        if (current == goal) break;
      }

      if (current == goal) break;
      if (steps >= maxSteps) return fail('step-limit');
      if (blocked) {
        replans++;
        if (movedThisPlan == 0) {
          // 1歩も進めない計画が2回続いたら空転とみなして失敗を返す
          noProgressPlans++;
          if (noProgressPlans >= 2) return fail('blocked-no-progress');
        } else {
          noProgressPlans = 0;
        }
        if (replans > 20) return fail('too-many-replans');
      }
    }

    return GoToResult(
      success: true,
      steps: steps,
      pathLengthM: lengthM,
      replans: replans,
      failureReason: null,
      traversed: traversed,
    );
  }
}
