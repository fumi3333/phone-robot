import 'dart:convert';

import 'explorer.dart';
import 'goal_navigator.dart';
import 'keepout.dart';
import 'occupancy_grid.dart';
import 'params.dart';
import 'pose.dart';

/// 巡回ルートの1地点。世界座標 [m] と表示名で保持する
/// （セル座標はグリッド解像度に依存するため、保存形式にはしない）。
class RouteWaypoint {
  final String name;
  final double x;
  final double y;

  const RouteWaypoint(this.name, this.x, this.y);

  Map<String, dynamic> toJson() => {'name': name, 'x': x, 'y': y};

  factory RouteWaypoint.fromJson(Map<String, dynamic> json) => RouteWaypoint(
        json['name'] as String,
        (json['x'] as num).toDouble(),
        (json['y'] as num).toDouble(),
      );
}

/// 巡回ルート（ウェイポイント列）。データとして保存・復元できる。
class PatrolRoute {
  final List<RouteWaypoint> waypoints;

  const PatrolRoute(this.waypoints);

  Map<String, dynamic> toJson() => {
        'waypoints': [for (final w in waypoints) w.toJson()],
      };

  String toJsonString() => jsonEncode(toJson());

  factory PatrolRoute.fromJson(Map<String, dynamic> json) => PatrolRoute([
        for (final w in json['waypoints'] as List<dynamic>)
          RouteWaypoint.fromJson(w as Map<String, dynamic>),
      ]);

  factory PatrolRoute.fromJsonString(String source) =>
      PatrolRoute.fromJson(jsonDecode(source) as Map<String, dynamic>);
}

/// 巡回の1区間（現在地 → ウェイポイント）の結果。
class PatrolLegResult {
  final int waypointIndex;
  final RouteWaypoint waypoint;
  final Cell goalCell;
  final bool reached;

  /// この区間の移動ステップ数・経路長・再計画回数（goTo の結果そのまま）。
  final int steps;
  final double pathLengthM;
  final int replans;

  /// 到達できなかった理由（到達時は null）。goTo の failureReason。
  final String? failureReason;

  const PatrolLegResult({
    required this.waypointIndex,
    required this.waypoint,
    required this.goalCell,
    required this.reached,
    required this.steps,
    required this.pathLengthM,
    required this.replans,
    required this.failureReason,
  });
}

/// 巡回1回分の結果。
class PatrolRunResult {
  final List<PatrolLegResult> legs;

  const PatrolRunResult(this.legs);

  int get totalSteps => legs.fold(0, (s, l) => s + l.steps);
  double get totalPathLengthM => legs.fold(0.0, (s, l) => s + l.pathLengthM);
  int get reachedCount => legs.where((l) => l.reached).length;
  bool get allReached => legs.every((l) => l.reached);
}

/// 巡回ルートの再生（保存したウェイポイント列を順に回る）。
///
/// 各区間は [GoalNavigator] に委譲する。到達できないウェイポイントが
/// あっても巡回自体は止めず、その区間を「未到達（理由付き）」として
/// 記録して次へ進む（警備巡回では「行けなかった地点がある」こと自体が
/// 報告対象のため。ただし現在地が動いていない以上、次の区間も同じ
/// 現在地から計画される）。
class PatrolRunner {
  final MappingParams params;

  const PatrolRunner(this.params);

  PatrolRunResult run({
    required PatrolRoute route,
    required OccupancyGrid grid,
    required MotionHooks hooks,

    /// 現在地セルの取得（SimRobot.cell、実機では believed pose 由来）。
    required Cell Function() currentCell,

    /// 立ち入り禁止エリア（全区間に適用）。
    KeepOutZones? keepOut,

    /// ウェイポイント到達時に呼ばれる（定点スナップショット取得など）。
    void Function(int index, RouteWaypoint waypoint, Cell cell)? onArrive,
    int maxStepsPerLeg = 2000,
  }) {
    final nav = GoalNavigator(params);
    final legs = <PatrolLegResult>[];
    for (var i = 0; i < route.waypoints.length; i++) {
      final wp = route.waypoints[i];
      final goal = cellOf(params, wp.x, wp.y);
      final res = nav.goTo(
        grid: grid,
        start: currentCell(),
        goal: goal,
        hooks: hooks,
        maxSteps: maxStepsPerLeg,
        keepOut: keepOut,
      );
      legs.add(PatrolLegResult(
        waypointIndex: i,
        waypoint: wp,
        goalCell: goal,
        reached: res.success,
        steps: res.steps,
        pathLengthM: res.pathLengthM,
        replans: res.replans,
        failureReason: res.failureReason,
      ));
      if (res.success) {
        onArrive?.call(i, wp, currentCell());
      }
    }
    return PatrolRunResult(legs);
  }
}
