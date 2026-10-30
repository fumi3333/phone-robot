/// 巡回機能の一括検証シナリオ（Dart 版）。
///
/// 使い方: dart run bin/patrol_scenario.dart [--seed N] [--out DIR]
///
/// 通しで実行する流れ:
///   1. 探索で地図作成
///   2. 巡回ルートを定義し、JSON として保存（patrol_route.json）→ 復元
///   3. ルート巡回 1回目（各地点で定点スナップショット撮影）
///   4. 帰還（スタート地点へ）
///   5. 立ち入り禁止エリア検証（回避走行＋禁止内ゴールの拒否）
///   6. 世界を変更（箱を1個追加）
///   7. ルート巡回 2回目（スナップショット再撮影→変化検出）
///   8. 巡回レポートを Markdown で出力（patrol_report.md）
///
/// 各段の成否・ステップ数・検出結果はコンソールにも表示する。
library;

import 'dart:io';

import 'package:phone_robot_core/phone_robot_core.dart';

void main(List<String> args) {
  var seed = 7;
  var outDir = '.';
  for (var i = 0; i < args.length; i++) {
    switch (args[i]) {
      case '--seed':
        seed = int.parse(args[++i]);
      case '--out':
        outDir = args[++i];
      default:
        throw ArgumentError('unknown argument: ${args[i]}');
    }
  }

  const params = MappingParams();
  final gt = buildGroundTruth(params);
  final grid = OccupancyGrid(params);
  final rng = GaussianRandom(seed);
  final start = cellOf(params, 0.80, 0.80);
  final robot = SimRobot(
    params: params,
    groundTruth: gt,
    grid: grid,
    rng: rng,
    drift: false,
    start: start,
  );
  final home = HomeReturn(params, start);

  print('=== patrol scenario (seed=$seed) ===');

  // ---- 1. 探索で地図作成
  final exploreResult =
      Explorer(params).explore(grid: grid, start: start, hooks: robot);
  print('[1] explore: steps=${exploreResult.steps} '
      'scans=${exploreResult.scans} '
      'coverage=${(exploreResult.coverage * 100).toStringAsFixed(2)}% '
      'end=${exploreResult.endReason}');

  // ---- 2. ルート定義 → JSON 保存 → 復元（保存と再生がデータで往復できること）
  const routeDef = PatrolRoute([
    RouteWaypoint('中央', 3.00, 2.50),
    RouteWaypoint('北東', 5.30, 4.30),
    RouteWaypoint('北西', 0.60, 4.30),
    RouteWaypoint('東', 5.50, 1.00),
  ]);
  final routeFile = File('$outDir/patrol_route.json');
  routeFile.writeAsStringSync(routeDef.toJsonString());
  final route = PatrolRoute.fromJsonString(routeFile.readAsStringSync());
  print('[2] route saved/restored: ${route.waypoints.length} waypoints '
      '-> ${routeFile.path}');

  // ---- 3. 巡回 1回目（到達地点でスナップショット）
  final snapshots1 = <String, ObservationSnapshot>{};
  final runner = PatrolRunner(params);
  final run1 = runner.run(
    route: route,
    grid: grid,
    hooks: robot,
    currentCell: () => robot.cell,
    onArrive: (i, wp, cell) {
      snapshots1[wp.name] = ObservationSnapshot.capture(robot, wp.name);
    },
  );
  for (final leg in run1.legs) {
    print('[3] patrol#1 leg ${leg.waypointIndex + 1} ${leg.waypoint.name}: '
        '${leg.reached ? 'REACHED' : 'MISSED(${leg.failureReason})'} '
        'steps=${leg.steps} path=${leg.pathLengthM.toStringAsFixed(2)}m '
        'replans=${leg.replans}');
  }
  print('[3] patrol#1: ${run1.reachedCount}/${run1.legs.length} reached, '
      'total steps=${run1.totalSteps}, snapshots=${snapshots1.length}');

  // ---- 4. 帰還
  final homeRes = home.goHome(grid: grid, current: robot.cell, hooks: robot);
  print('[4] return-home: '
      '${homeRes.success ? 'SUCCESS' : 'FAIL(${homeRes.failureReason})'} '
      'steps=${homeRes.steps} path=${homeRes.pathLengthM.toStringAsFixed(2)}m '
      'at_home=${robot.cell == start}');

  // ---- 5. 立ち入り禁止エリア検証
  // (a) 中央付近を禁止にし、西→東の横断が禁止セルを踏まずに成功するか。
  // (b) 禁止エリア内のゴールを指定し、正直に失敗するか。
  final keepOut = KeepOutZones.fromWorldRects(params, [
    (x0: 2.40, y0: 2.10, x1: 3.60, y1: 2.90),
  ]);
  final nav = GoalNavigator(params);
  final eastGoal = cellOf(params, 5.50, 1.00);
  final crossRes = nav.goTo(
    grid: grid,
    start: robot.cell,
    goal: eastGoal,
    hooks: robot,
    keepOut: keepOut,
  );
  var crossedIntoKeepOut = 0;
  for (final cell in crossRes.traversed) {
    if (keepOut.containsCell(params, cell)) crossedIntoKeepOut++;
  }
  final crossOk = crossRes.success && crossedIntoKeepOut == 0;
  print('[5a] keep-out crossing: '
      '${crossRes.success ? 'SUCCESS' : 'FAIL(${crossRes.failureReason})'} '
      'steps=${crossRes.steps} keepout_cells_traversed=$crossedIntoKeepOut '
      'zones=${keepOut.size} cells -> ${crossOk ? 'OK' : 'NG'}');

  final insideGoal = cellOf(params, 3.00, 2.50); // 禁止矩形の中心
  final insideRes = nav.goTo(
    grid: grid,
    start: robot.cell,
    goal: insideGoal,
    hooks: robot,
    keepOut: keepOut,
  );
  final insideOk =
      !insideRes.success && insideRes.failureReason == 'goal-keepout';
  print('[5b] keep-out goal rejection: '
      '${insideRes.success ? 'SUCCESS(unexpected)' : 'FAIL(${insideRes.failureReason})'} '
      '-> ${insideOk ? 'OK' : 'NG'}');

  // 帰還（禁止エリア検証で東側にいるため）
  final homeRes2 = home.goHome(grid: grid, current: robot.cell, hooks: robot);
  print('[5c] return-home after keep-out tests: '
      '${homeRes2.success ? 'SUCCESS' : 'FAIL(${homeRes2.failureReason})'} '
      'steps=${homeRes2.steps} at_home=${robot.cell == start}');

  // ---- 6. 世界を変更: 箱を1個追加（中央の南、2.70-3.10 × 1.90-2.25 m）
  const addedBox = (x0: 2.70, y0: 1.90, x1: 3.10, y1: 2.25);
  final bc0 = (addedBox.x0 / params.cellSize).floor();
  final bc1 = (addedBox.x1 / params.cellSize).ceil();
  final br0 = (addedBox.y0 / params.cellSize).floor();
  final br1 = (addedBox.y1 / params.cellSize).ceil();
  for (var r = br0; r < br1; r++) {
    for (var c = bc0; c < bc1; c++) {
      gt[r * params.gridW + c] = 1;
    }
  }
  print('[6] world changed: box added '
      '(${addedBox.x0}, ${addedBox.y0})-(${addedBox.x1}, ${addedBox.y1})');

  // ---- 7. 巡回 2回目＋変化検出
  const detector = ChangeDetector();
  final changeChecks = <({String waypointName, ChangeAssessment assessment})>[];
  final run2 = runner.run(
    route: route,
    grid: grid,
    hooks: robot,
    currentCell: () => robot.cell,
    onArrive: (i, wp, cell) {
      final before = snapshots1[wp.name];
      if (before == null) return;
      final after = ObservationSnapshot.capture(robot, wp.name);
      changeChecks.add((
        waypointName: wp.name,
        assessment: detector.compare(before, after),
      ));
    },
  );
  for (final leg in run2.legs) {
    print('[7] patrol#2 leg ${leg.waypointIndex + 1} ${leg.waypoint.name}: '
        '${leg.reached ? 'REACHED' : 'MISSED(${leg.failureReason})'} '
        'steps=${leg.steps} path=${leg.pathLengthM.toStringAsFixed(2)}m '
        'replans=${leg.replans}');
  }
  for (final c in changeChecks) {
    final a = c.assessment;
    print('[7] change @${c.waypointName}: '
        '${a.changed ? 'CHANGED' : 'no-change'} '
        'changed_rays=${a.changedRayCount}/${a.totalRays} '
        'mean_diff=${a.meanAbsDiff.toStringAsFixed(3)}m '
        'max_diff=${a.maxAbsDiff.toStringAsFixed(2)}m');
  }

  // ---- 8. レポート出力
  final report = buildPatrolReport(PatrolReportData(
    when: DateTime.now(),
    params: params,
    exploration: (
      coverage: exploreResult.coverage,
      steps: exploreResult.steps,
      scans: exploreResult.scans,
      endReason: exploreResult.endReason,
    ),
    patrolRuns: [
      (label: 'ルート巡回（1回目）', result: run1),
      (label: 'ルート巡回（2回目・変更後）', result: run2),
    ],
    homeReturn: (result: homeRes, homeLabel: 'スタート地点'),
    keepOutChecks: [
      (
        label: '禁止エリアを避けた横断（西→東）',
        ok: crossOk,
        detail: '${crossRes.steps} ステップ、禁止セル踏破 '
            '$crossedIntoKeepOut セル',
      ),
      (
        label: '禁止エリア内のゴールの拒否',
        ok: insideOk,
        detail: 'failureReason=${insideRes.failureReason}',
      ),
    ],
    changeChecks: changeChecks,
    detector: detector,
    notes: [
      '変化シナリオ: 1回目と2回目の巡回の間に箱を1個追加'
          '（${addedBox.x0}-${addedBox.x1} m × ${addedBox.y0}-${addedBox.y1} m）。',
      '地図カバー率（最終）: ${(grid.coverage * 100).toStringAsFixed(2)}%。',
      '本レポートはシミュレーション検証の自動生成例。実機の巡回報告では'
          'スナップショットを Depth API の実測に差し替える。',
    ],
  ));
  final reportFile = File('$outDir/patrol_report.md');
  reportFile.writeAsStringSync(report);
  print('[8] report written -> ${reportFile.path}');
  print('=== scenario done ===');
}
