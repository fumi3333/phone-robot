import 'goal_navigator.dart';
import 'params.dart';
import 'patrol.dart';
import 'snapshots.dart';

/// 巡回レポートの入力データ一式。
/// 各セクションは任意で、入っているものだけが出力される。
class PatrolReportData {
  /// 報告日時。
  final DateTime when;

  final MappingParams params;

  /// 探索フェーズの結果（カバー率・ステップ数・スキャン数・終了理由）。
  final ({
    double coverage,
    int steps,
    int scans,
    String endReason
  })? exploration;

  /// ルート巡回の結果（ラベル付き、実施順）。
  final List<({String label, PatrolRunResult result})> patrolRuns;

  /// 帰還の結果。
  final ({GoToResult result, String homeLabel})? homeReturn;

  /// 立ち入り禁止エリア検証の結果（ラベル・期待通りか・詳細）。
  final List<({String label, bool ok, String detail})> keepOutChecks;

  /// 変化検出の結果（地点名つき、実施順）。
  final List<({String waypointName, ChangeAssessment assessment})> changeChecks;

  /// 変化検出に使った指標の説明。
  final ChangeDetector? detector;

  /// 備考（変化シナリオの説明など）。
  final List<String> notes;

  const PatrolReportData({
    required this.when,
    required this.params,
    this.exploration,
    this.patrolRuns = const [],
    this.homeReturn,
    this.keepOutChecks = const [],
    this.changeChecks = const [],
    this.detector,
    this.notes = const [],
  });
}

String _f2(double v) => v.toStringAsFixed(2);
String _pct(double v) => '${(v * 100).toStringAsFixed(2)}%';

/// 警備の巡回報告を意識した Markdown レポートを生成する。
///
/// 構成: 日時・対象エリア → 巡回結果サマリ → 異常（変化あり）地点の
/// 一覧 → 巡回詳細（区間ごと）→ 禁止エリア検証 → 備考。
String buildPatrolReport(PatrolReportData data) {
  final p = data.params;
  final b = StringBuffer();
  final changedPoints = [
    for (final c in data.changeChecks)
      if (c.assessment.changed) c,
  ];

  b.writeln('# 巡回報告書');
  b.writeln();
  b.writeln('- 報告日時: ${data.when.toIso8601String()}');
  b.writeln('- 対象エリア: ${_f2(p.roomW)} m × ${_f2(p.roomH)} m'
      '（グリッド ${p.gridW}×${p.gridH}、セル ${(p.cellSize * 100).toStringAsFixed(0)} cm）');
  b.writeln('- 巡回方式: 自律探索で地図を作成し、保存ルートを再生'
      '（シミュレーション検証）');
  b.writeln();

  // ---- サマリ
  b.writeln('## 巡回結果サマリ');
  b.writeln();
  b.writeln('| 項目 | 結果 |');
  b.writeln('|---|---|');
  final ex = data.exploration;
  if (ex != null) {
    b.writeln('| 地図カバー率 | ${_pct(ex.coverage)} |');
    b.writeln('| 探索ステップ数 | ${ex.steps}（スキャン ${ex.scans} 回） |');
  }
  for (final run in data.patrolRuns) {
    b.writeln('| ${run.label} | ${run.result.reachedCount}/'
        '${run.result.legs.length} 地点到達'
        '（合計 ${run.result.totalSteps} ステップ、'
        '${_f2(run.result.totalPathLengthM)} m） |');
  }
  final home = data.homeReturn;
  if (home != null) {
    final r = home.result;
    b.writeln('| 帰還（${home.homeLabel}） | '
        '${r.success ? '成功' : '失敗: ${r.failureReason}'}'
        '（${r.steps} ステップ、${_f2(r.pathLengthM)} m） |');
  }
  if (data.keepOutChecks.isNotEmpty) {
    final okCount = data.keepOutChecks.where((c) => c.ok).length;
    b.writeln('| 立ち入り禁止エリア検証 | $okCount/'
        '${data.keepOutChecks.length} 件が期待通り |');
  }
  if (data.changeChecks.isNotEmpty) {
    b.writeln('| 変化検出 | ${changedPoints.length} 地点で変化あり'
        ' / 全 ${data.changeChecks.length} 地点 |');
  }
  b.writeln();

  // ---- 異常（変化あり）地点
  b.writeln('## 異常（変化あり）地点');
  b.writeln();
  if (data.changeChecks.isEmpty) {
    b.writeln('変化検出は実施していません。');
  } else if (changedPoints.isEmpty) {
    b.writeln('異常なし（全地点で変化は検出されませんでした）。');
  } else {
    b.writeln('| 地点 | 変化レイ数 | 平均差分 [m] | 最大差分 [m] |');
    b.writeln('|---|---|---|---|');
    for (final c in changedPoints) {
      final a = c.assessment;
      b.writeln('| ${c.waypointName} | ${a.changedRayCount}/${a.totalRays} '
          '| ${_f2(a.meanAbsDiff)} | ${_f2(a.maxAbsDiff)} |');
    }
  }
  b.writeln();

  // ---- 巡回詳細
  b.writeln('## 巡回詳細');
  b.writeln();
  for (final run in data.patrolRuns) {
    b.writeln('### ${run.label}');
    b.writeln();
    b.writeln('| # | 地点 | 結果 | ステップ | 経路長 [m] | 再計画 |');
    b.writeln('|---|---|---|---|---|---|');
    for (final leg in run.result.legs) {
      b.writeln('| ${leg.waypointIndex + 1} | ${leg.waypoint.name} '
          '| ${leg.reached ? '到達' : '未到達: ${leg.failureReason}'} '
          '| ${leg.steps} | ${_f2(leg.pathLengthM)} | ${leg.replans} |');
    }
    b.writeln();
  }

  // ---- 禁止エリア検証
  if (data.keepOutChecks.isNotEmpty) {
    b.writeln('## 立ち入り禁止エリア検証');
    b.writeln();
    for (final c in data.keepOutChecks) {
      b.writeln('- ${c.label}: ${c.ok ? '期待通り' : '想定外'}（${c.detail}）');
    }
    b.writeln();
  }

  // ---- 変化検出の全地点表
  if (data.changeChecks.isNotEmpty) {
    b.writeln('## 変化検出（全地点）');
    b.writeln();
    b.writeln('| 地点 | 判定 | 変化レイ数 | 平均差分 [m] | 最大差分 [m] |');
    b.writeln('|---|---|---|---|---|');
    for (final c in data.changeChecks) {
      final a = c.assessment;
      b.writeln('| ${c.waypointName} | ${a.changed ? '変化あり' : '変化なし'} '
          '| ${a.changedRayCount}/${a.totalRays} '
          '| ${_f2(a.meanAbsDiff)} | ${_f2(a.maxAbsDiff)} |');
    }
    b.writeln();
  }

  // ---- 備考
  if (data.notes.isNotEmpty || data.detector != null) {
    b.writeln('## 備考');
    b.writeln();
    final d = data.detector;
    if (d != null) {
      b.writeln('- 変化検出の指標: レイごとの距離差が '
          '${_f2(d.rayDiffThresholdM)} m を超えたレイを「変化レイ」とし、'
          '${d.changedRayThreshold} 本以上でその地点を「変化あり」と判定。'
          '片方だけ計測できなかったレイは ${_f2(d.noMeasureDiffM)} m の差として数える。');
    }
    for (final n in data.notes) {
      b.writeln('- $n');
    }
    b.writeln();
  }

  return b.toString();
}
