import 'dart:math' as math;
import 'dart:typed_data';

import 'astar.dart';
import 'occupancy_grid.dart';
import 'params.dart';
import 'pose.dart';

/// 探索・ナビゲーションが車両側に要求する動作の抽象（SPEC §5 の差し替え点）。
///
/// シミュレーションでは SimRobot がこれを実装する。実機では
/// 「ARCore の自己位置でスキャンを地図へ登録する」「USB シリアル経由で
/// ESP32 へ操舵指令を送る」実装に差し替える。
abstract class MotionHooks {
  /// 現在の物理的な向き [rad]（スキャンの基準方向）。
  double get heading;

  /// 指定した向きでスキャンし、結果を believed pose で地図へ登録する。
  void scan(double heading);

  /// 次セルへの1ステップ移動を試みる。
  ///
  /// 物理的に進入できない場合（実機では近接 ToF / バンパーの
  /// フェイルセーフ）は false を返す。その場合、そのセルは強い占有証拠
  /// として地図に記録済みであること（参照実装のフェイルセーフと同じ
  /// 意味づけ: SPEC §5.3）。
  bool moveTo(Cell next);
}

/// 探索の結果。
class ExploreResult {
  final int steps;
  final int scans;
  final double coverage;
  final bool targetReached;

  /// 'target' / 'no-frontier' / 'no-progress' / 'step-limit'
  final String endReason;

  const ExploreResult({
    required this.steps,
    required this.scans,
    required this.coverage,
    required this.targetReached,
    required this.endReason,
  });
}

/// フロンティア探索（SPEC §3.4）。参照実装 sim_mapping.py と同一の手順。
class Explorer {
  final MappingParams params;

  const Explorer(this.params);

  ExploreResult explore({
    required OccupancyGrid grid,
    required Cell start,
    required MotionHooks hooks,
    void Function(Float32List snapshot, Cell robot)? onFrame,
  }) {
    final p = params;
    var robot = start;
    // 立っているセルは空間として初期化する（参照実装と同じ）
    grid.logOdds[grid.index(robot.r, robot.c)] = p.lFree;

    var steps = 0;
    var scans = 0;
    var failedPlans = 0;

    void doScan(double heading) {
      hooks.scan(heading);
      scans++;
    }

    void rotationScan() {
      final base = hooks.heading;
      for (var k = 0; k < 12; k++) {
        doScan(base + k * math.pi / 6);
      }
    }

    doScan(hooks.heading);
    rotationScan();

    var cov = grid.coverage;
    var endReason = 'step-limit';

    while (cov < p.targetCoverage && steps < p.maxSteps) {
      final mask = grid.planMask(forcedFree: robot);
      final fronts = grid.frontierCells(mask);
      if (fronts.isEmpty) {
        endReason = 'no-frontier';
        break;
      }
      // ユークリッド距離の近い順に候補を並べ、上限60候補まで試す
      fronts.sort((a, b) {
        final da = (a.r - robot.r) * (a.r - robot.r) +
            (a.c - robot.c) * (a.c - robot.c);
        final db = (b.r - robot.r) * (b.r - robot.r) +
            (b.c - robot.c) * (b.c - robot.c);
        return da.compareTo(db);
      });
      List<Cell>? path;
      for (final goal in fronts.take(60)) {
        if (goal == robot) continue; // 既に立っているフロンティアは除く
        path = findPath(
          gridW: p.gridW,
          gridH: p.gridH,
          isFree: (i) => mask[i] == 1,
          start: robot,
          goal: goal,
        );
        if (path != null) break;
      }

      if (path == null || path.length < 2) {
        // 進展のない計画。到達不能候補への空転を防ぐため、
        // 2回連続で打ち切る（参照実装の運用ガード: SPEC §3.4）。
        failedPlans++;
        if (failedPlans >= 2) {
          endReason = 'no-progress';
          break;
        }
        rotationScan();
        if (grid.coverage <= cov) {
          endReason = 'no-progress';
          break;
        }
        cov = grid.coverage;
        continue;
      }
      failedPlans = 0;

      // 経路追従: 1セル進むごとにスキャン
      for (var pi = 1; pi < path.length; pi++) {
        if (steps >= p.maxSteps) break;
        final next = path[pi];
        if (!hooks.moveTo(next)) {
          // フェイルセーフで拒否された。セルは占有として記録済み。
          // この経路は破棄し、外側のループで再計画する。
          break;
        }
        robot = next;
        doScan(hooks.heading);
        steps++;
        if (steps % 10 == 0) {
          onFrame?.call(Float32List.fromList(grid.logOdds), robot);
        }
        if (steps % 50 == 0 && grid.coverage >= p.targetCoverage) {
          break;
        }
      }
      rotationScan();
      cov = grid.coverage;
    }

    final targetReached = cov >= p.targetCoverage;
    if (targetReached) endReason = 'target';
    return ExploreResult(
      steps: steps,
      scans: scans,
      coverage: cov,
      targetReached: targetReached,
      endReason: endReason,
    );
  }
}
