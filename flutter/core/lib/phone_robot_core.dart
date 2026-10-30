/// phone-robot マッピング中核ライブラリ（Flutter 非依存の純 Dart）。
///
/// 占有格子地図（ログオッズ）＋フロンティア探索＋A* 経路計画＋
/// ゴール指定ナビゲーション。移植仕様は phone-robot-mapping/SPEC.md。
library;

export 'src/astar.dart';
export 'src/explorer.dart';
export 'src/goal_navigator.dart';
export 'src/home_return.dart';
export 'src/keepout.dart';
export 'src/occupancy_grid.dart';
export 'src/params.dart';
export 'src/patrol.dart';
export 'src/pose.dart';
export 'src/report.dart';
export 'src/scan.dart';
export 'src/sim_world.dart';
export 'src/snapshots.dart';
