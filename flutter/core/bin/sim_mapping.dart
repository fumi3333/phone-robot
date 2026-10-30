/// Dart 版マッピングシミュレーション（参照実装 sim_mapping.py と同一条件）。
///
/// 使い方: dart run bin/sim_mapping.dart [--drift] [--seed N] [--skip-goals]
///
/// 探索（占有格子＋フロンティア探索）を最後まで走らせ、カバー率・
/// ステップ数・既知セルの正解率を表示する。その後、探索で作った地図の
/// 上でゴール指定ナビゲーションを複数点で検証する。
library;

import 'package:phone_robot_core/phone_robot_core.dart';

void main(List<String> args) {
  var drift = false;
  var seed = 7;
  var skipGoals = false;
  for (var i = 0; i < args.length; i++) {
    switch (args[i]) {
      case '--drift':
        drift = true;
      case '--seed':
        seed = int.parse(args[++i]);
      case '--skip-goals':
        skipGoals = true;
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
    drift: drift,
    start: start,
  );

  // ---- 探索フェーズ
  final explorer = Explorer(params);
  final result = explorer.explore(grid: grid, start: start, hooks: robot);

  var known = 0;
  var correct = 0;
  for (var i = 0; i < gt.length; i++) {
    if (grid.logOdds[i] != 0) {
      known++;
      if ((grid.logOdds[i] > 0) == (gt[i] != 0)) correct++;
    }
  }
  final acc = known == 0 ? 0.0 : correct / known * 100;

  print('mode=${drift ? 'drift' : 'normal'} seed=$seed');
  print('steps=${result.steps} scans=${result.scans}');
  print('coverage=${(grid.coverage * 100).toStringAsFixed(2)}%  '
      '(known $known/${gt.length} cells)');
  print('map_accuracy_on_known=${acc.toStringAsFixed(2)}%');
  print('explore_end=${result.endReason}');

  if (skipGoals) return;

  // ---- ゴール指定ナビゲーションフェーズ（探索後の地図を使う）
  print('--- goal navigation (map built by exploration above) ---');
  const goals = <(String, double, double)>[
    ('NE corner', 5.30, 4.30),
    ('NW corner', 0.60, 4.30),
    ('east of box3', 5.50, 1.00),
    ('room center', 3.00, 2.50),
    ('west-north', 1.05, 3.90),
    // 負例: 箱1の内部。観測不能で未探索のはず → 正直に失敗するのが正しい。
    ('inside box1 (negative)', 1.80, 1.45),
  ];
  final nav = GoalNavigator(params);
  for (final (name, gx, gy) in goals) {
    final goal = cellOf(params, gx, gy);
    final res =
        nav.goTo(grid: grid, start: robot.cell, goal: goal, hooks: robot);
    final status = res.success ? 'SUCCESS' : 'FAIL(${res.failureReason})';
    print('goal $name (${gx.toStringAsFixed(2)}, ${gy.toStringAsFixed(2)}) '
        'cell=(${goal.r}, ${goal.c}): $status steps=${res.steps} '
        'path=${res.pathLengthM.toStringAsFixed(2)}m replans=${res.replans}');
  }
  print('coverage_after_goals=${(grid.coverage * 100).toStringAsFixed(2)}%');
}
