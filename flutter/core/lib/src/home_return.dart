import 'explorer.dart';
import 'goal_navigator.dart';
import 'keepout.dart';
import 'occupancy_grid.dart';
import 'params.dart';
import 'pose.dart';

/// 帰還（return-to-home）。
///
/// 探索開始地点など「ホーム」を記録しておき、任意の場所から既知の
/// 空間だけを通って戻る。実体はゴール指定ナビゲーションの応用で、
/// ゴールがホームセルに固定されているだけだが、「帰れたか」を巡回
/// レポートに残す単位として独立させてある。
class HomeReturn {
  final MappingParams params;

  /// 記録されたホームセル。
  final Cell homeCell;

  const HomeReturn(this.params, this.homeCell);

  /// 現在地 [current] からホームへ戻る。
  /// 既にホームにいる場合は steps=0 で成功を返す。
  GoToResult goHome({
    required OccupancyGrid grid,
    required Cell current,
    required MotionHooks hooks,
    KeepOutZones? keepOut,
    int maxSteps = 2000,
  }) {
    return GoalNavigator(params).goTo(
      grid: grid,
      start: current,
      goal: homeCell,
      hooks: hooks,
      maxSteps: maxSteps,
      keepOut: keepOut,
    );
  }
}
