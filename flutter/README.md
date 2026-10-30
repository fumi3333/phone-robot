# phone-robot マッピング（Flutter/Dart 移植）

作成: 2026-10-30
仕様: `~/workspace/phone-robot-mapping/SPEC.md`（参照実装 `sim_mapping.py`）

Python で検証済みのマッピング中核（占有格子＋フロンティア探索＋A*）を
Flutter/Dart へ移植したもの。加えて、本人の希望で「できたマップの中で
『ここに行って』と指定できる」ゴール指定ナビゲーションを中核に追加した。

## 構成

```
core/                    Flutter 非依存の純 Dart パッケージ (phone_robot_core)
  lib/src/params.dart          主要パラメータ（SPEC §4）
  lib/src/pose.dart            Pose / Cell / 座標変換
  lib/src/scan.dart            Scan（21本レイ）と角度オフセット表
  lib/src/occupancy_grid.dart  占有格子（ログオッズ Float32List、§2.1/§3.1/§3.2）
  lib/src/astar.dart           A*（8近傍・角抜け防止・octile、§3.3）
  lib/src/explorer.dart        MotionHooks 抽象＋フロンティア探索ループ（§3.4）
  lib/src/goal_navigator.dart  ゴール指定ナビゲーション（追加機能）
  lib/src/home_return.dart     帰還（ゴール指定ナビの応用）
  lib/src/keepout.dart         立ち入り禁止エリア（計画への反映用データ構造）
  lib/src/patrol.dart          巡回ルートの保存（JSON）・再生
  lib/src/snapshots.dart       定点スナップショット＋変化検出
  lib/src/report.dart          巡回レポート（Markdown）自動生成
  lib/src/sim_world.dart       シミュレーション専用: 真値の部屋・レイキャスト・SimRobot
  bin/sim_mapping.dart         Dart シミュレーション実行ファイル
  bin/patrol_scenario.dart     巡回機能の一括検証シナリオ（下記の実測の再現手順）
app/                     Flutter アプリ（シミュレーション＋実機モード完成形）
  lib/main.dart                モード切替（シミュレーション / 実機）
  lib/sim_page.dart            シミュレーション画面（探索アニメ＋タップでゴール指定）
  lib/map_painter.dart         占有格子の描画（ゴールマーカー・経路表示）
  lib/sim_session.dart         アプリ用シミュレーション駆動
  lib/real_page.dart           実機モード画面（状態表示・地図・ジョグパッド・探索/移動操作）
  lib/real/real_session.dart   実機セッション（姿勢/深度/シリアルの集約）
  lib/real/real_driver.dart    実機ドライバ（中核の計画部品を共有し、走行だけ非同期化。
                               近接フェイルセーフ＝最小距離<0.22mで即停止→占有記録→再計画）
  lib/platform/pose_source.dart   自己位置の差し替え点（SPEC §5.1、ARCore 想定）
  lib/platform/depth_source.dart  距離の差し替え点（SPEC §5.2、Depth API 想定）
  lib/platform/serial_link.dart   ESP32 リンクの差し替え点（ESP32 GPIO16-19、
                               115200bps、"c,drive,steer\n"＋単文字 f/b/l/r/s）
  android/                     Android ネイティブ（Kotlin、自前プラットフォームチャネル）
    .../MainActivity.kt        チャネルの配線
    .../ArCoreManager.kt       ARCore セッション（描画なしEGLでポーズ＋Raw Depth→21本レイ）
    .../SerialManager.kt       usb-serial-for-android で ESP32 接続（USB権限フロー含む）
```

## アプリ完成形の追加（2026-10-30）

骨格から完成形のソース一式へ仕上げた。範囲:

- Dart 側: モード切替、実機モードの状態表示（ARCore トラッキング状態 /
  ESP32 リンク状態 / 自己位置 / 現在の指令 / カバー率）、実機ドライバ
  （経路→シリアル指令変換、近接フェイルセーフ、回転スキャン、タイムアウト時の
  障害物記録＋再計画）。中核（core）の API と検証済みロジックは無変更。
  MotionHooks が同期型のため、実機走行は同じ手順・同じガード条件を
  非同期ドライバとして再実装し、計画部品（格子・フロンティア・A*）は
  中核をそのまま共有している。
- ネイティブ側（Kotlin）: ARCore セッション管理と SPEC §5.1 の座標変換
  （開始時ポーズを原点に固定、四元数の自前計算で `inverse/compose` 相当）、
  Raw Depth の21本レイ化（信頼度 >=128 のみ採用、中央帯の中央値）、
  USB シリアル（115200bps、権限要求・切断検知含む）。Manifest は
  カメラ権限・ARCore メタデータ（required）・USB ホスト宣言・
  USB 接続時の起動フィルタ（CH340 等の VID）を追加。build.gradle は
  minSdk 24（ARCore 要件）、ARCore 1.44.0、usb-serial-for-android 3.7.0。
- 参考資料（REFERENCES.md）の反映:
  - **深度の 1/cos(θ) 補正を実装**。ARCore の深度値は光線の実距離ではなく
    光軸への射影（z値）のため、レイ実距離 = z / cos(θ) で補正する
    （端のレイで最大約 +15%。放置すると壁が内側に湾曲する）。
  - **深度フレームの新旧判定を実装**。深度画像のタイムスタンプが前回
    配信分と同一なら配信しない（公式手順。スキャンの二重更新防止）。
  - ハートビート指令（OpenBot の `h<ms>` 方式）は**要検討**（下記）。

## 実行方法（Dart シミュレーション）

```sh
cd core
dart pub get
dart run bin/sim_mapping.dart            # 通常モード＋ゴール検証
dart run bin/sim_mapping.dart --drift    # 自己位置ドリフトモード
dart run bin/sim_mapping.dart --seed 3   # シード変更
```

## 検証済みのこと（この VM で実測）

### 探索（通常モード、seed=7）: Python 版との比較

| 項目 | Dart 版（実測） | Python 版（実測） |
|---|---|---|
| 移動ステップ数 | 214 | 132 |
| スキャン回数 | 395 | 265 |
| 最終カバー率 | 97.63% | 96.89% |
| 既知セルの正解率 | 95.70% | 95.56% |

- 条件は同一（部屋 6m×5m、箱3個、扇形レイ21本・視野角60°・最大5m・
  距離ノイズ σ3cm、セル5cm、開始 (0.80, 0.80)）。
- 乱数系列が異なる（numpy PCG64 と Dart Random＋Box-Muller）ため
  ステップ数などの細部は一致しないが、カバー率・正解率は同水準で
  再現した。他シードでも seed=3: 95.11%（158ステップ）、
  seed=21: 95.03%（191ステップ）と安定して目標 95% に到達。
- 参考: カバー率の理論上限 ≒97.6% は「登録が完全な場合」の近似。
  seed=7 では自己位置ノイズにより箱内部のセルが数セルだけ登録され、
  上限をわずかに超えた。

### ゴール指定ナビゲーション（通常モード、探索完了後の地図で実測）

A* のゴールをフロンティアから指定点へ差し替え、既知の空間内だけを
通る。未知セルは通らない（安全側）。

| ゴール | 結果 | ステップ | 経路長 | 再計画 |
|---|---|---|---|---|
| (5.30, 4.30) 北東 | 到達 | 57 | 2.91 m | 0 |
| (0.60, 4.30) 北西 | 到達 | 94 | 4.74 m | 0 |
| (5.50, 1.00) 箱3の東 | 到達 | 102 | 6.38 m | 0 |
| (3.00, 2.50) 中央 | 到達 | 53 | 3.21 m | 0 |
| (1.05, 3.90) 西北寄り | 到達 | 39 | 2.53 m | 0 |
| (1.80, 1.45) 箱1内部（負例） | 失敗 `goal-unknown` | 0 | — | 0 |

- 負例は「未探索の場所へは行けると言わない」という安全側の失敗で、
  期待通りの挙動。
- ゴール走行中のスキャンで地図がさらに育ち、カバー率は
  97.63% → 98.47% に増えた。

### ドリフトモード（参考実測）

- Dart seed=7: 128ステップ、カバー率 95.32%、正解率 95.36%。
  Python 版（1186ステップ、正解率 84.01%）ほど悪化しなかった。
  ランダムウォークの実現値は乱数系列に強く依存するため、この差は
  系列差の範囲とみている（「補正なしでは歪む」という結論自体は
  DRIFT_MEMO.md の Python 実験が示している）。
- それでも歪んだ地図の影響はゴール検証に出た: 到達可能だったはずの
  5点中3点が `unreachable`（幻の壁が経路を塞いだ）。うち1点では
  物理フェイルセーフ発動 → 再計画（replans=1）→ それでも到達不能で
  正直に失敗を返した。再計画の発動はこのケースで確認できた。

### コード品質

- `dart analyze`（core パッケージ）: No issues found。
- アプリ側を含む全 Dart ファイルは `dart format` でパース成功
  （構文エラーのないことのみ確認。意味解析は Flutter SDK が必要なため
  未実施）。

### 巡回機能の一括追加（帰還・ルート保存/再生・禁止エリア・変化検出・レポート）

「今の土台（地図＋ゴール指定移動）で作れる機能」として中核に追加し、
`bin/patrol_scenario.dart` で通し検証した。流れは
探索 → ルートを JSON 保存/復元 → 巡回1回目（定点スナップショット撮影）
→ 帰還 → 禁止エリア検証 → 世界を変更（箱1個追加）→ 巡回2回目で
変化検出 → レポート出力。実行方法:

```sh
cd core
dart run bin/patrol_scenario.dart --out ..   # patrol_report.md / patrol_route.json を出力
```

実測（seed=7、通常モード。seed=3 でも同構造の結果を再現）:

| 段 | 結果 |
|---|---|
| 探索 | カバー率 97.63%（214ステップ、既存検証と不変） |
| 巡回1回目 | 4/4 地点到達（合計 295 ステップ、16.97 m） |
| 帰還 | 成功（94ステップ。禁止エリア検証後の帰還も成功） |
| 禁止エリア回避 | 中央 2.4-3.6 m × 2.1-2.9 m（400セル）を禁止にし西→東を横断: 成功し、禁止セルの踏破 0 セル |
| 禁止内ゴールの拒否 | 禁止矩形中心を指定 → `goal-keepout` で正直に失敗 |
| 巡回2回目（箱追加後） | 4/4 地点到達（合計 296 ステップ、17.29 m） |
| 変化検出 | 追加した箱に最も近い「中央」だけを検出: 変化レイ 15/84、平均差 0.43 m、最大差 2.46 m。他3地点は変化レイ 0（平均差 約0.03 m = ノイズ水準）で誤検出なし |

- 変化検出の指標: 定点で固定4方位 × 21本レイの距離列をスナップ
  ショットとして保存し、再巡回時に比較する。レイ差 0.30 m 超を
  「変化レイ」とし、4本以上でその地点を「変化あり」と判定する
  （ノイズ σ=3cm に対し 0.30 m は約10σ）。指標は
  `core/lib/src/snapshots.dart` の ChangeDetector に明記。
- 禁止エリアは地図のログオッズを書き換えず、計画時にだけマスクへ
  反映する（運用上の制約 ≠ 観測結果。解除すれば元の地図で通れる）。
- レポート生成例: `patrol_report.md`（本ディレクトリ）。日時・
  サマリ・異常（変化あり）地点一覧・区間別詳細を含む警備巡回報告式。
  ルートの保存例は `patrol_route.json`。

### 今回対象外（やっていないこと）

- 物体検出との連携（探し物モード等）: 検出モデルが要るため対象外。
- 音声入力による指示: 対象外（入り口は地図タップのみ）。
- 通信・遠隔操作（離れた場所からの指定・映像確認）: 対象外。
- これらはいずれも中核ではなく「入力/出力の差し替え点」の拡張で、
  地図・移動・巡回の中核 API は変えずに足せる設計にしてある。

## テスト結果（2026-10-30、この VM で実測）

### 静的解析

- `dart analyze`（core）: **No issues found**
- `flutter analyze`（app）: **No issues found**（未使用 import 1件を修正済み）

### core 回帰確認（アプリ完成化の後も中核が崩れていないこと）

- `dart run bin/sim_mapping.dart`（seed=7）: 探索 **coverage 97.63% /
  既知セル正解率 95.70%**、ゴール 5/5 到達（最長 6.38 m / 102ステップ）、
  箱内部ゴールは `goal-unknown` で正直に失敗 — 従来の検証値と完全一致。
- `dart run bin/patrol_scenario.dart`（seed=7）: 巡回1回目 4/4 到達
  （295ステップ）、帰還成功、禁止エリア踏破 0セル＋ `goal-keepout` 拒否、
  箱追加後の巡回2回目で**「中央」だけ変化検出**（変化レイ 15/84、
  平均差 0.43 m）、他3地点は誤検出 0 — 従来通り。

### APK ビルド挑戦: 停止点（環境の権限でブロック）

ツールチェーン（Flutter 3.24.5 / JDK 17 / Android SDK 34）を
`~/workspace/toolchain/` に導入し、`flutter pub get` までは成功した。
しかし `flutter build apk --debug` は **Gradle デーモンとの通信で停止**した。

- 症状: Gradle クライアントがデーモンからメッセージを読めない
  （`MessageIOException: Unexpected type tag 109`）。
- 原因の切り分け（実測）: 最小の Java ループバックプログラムでも、
  接続先に届かず実行環境の遮断メッセージを受け取る
  （先頭の 'm' = タグ109 の正体）:
  > `muse: Other TCP connections is turned off for this assistant. To
  > allow it, ask the user to open Muse settings -> Permissions ->
  > Direct network protocols and switch other_tcp from Deny to Ask.`
  つまり **JVM の TCP 接続が Muse の権限設定（other_tcp = Deny）で
  遮断**されており、Gradle デーモン（127.0.0.1 のソケット必須）が
  通信できない。Python のループバックは遮断されなかった。
  同じ理由で Gradle 本体や sdkmanager の Java 製ダウンローダも使えず、
  Gradle 8.3 ディストリビューションは curl で取得して手動配置した
  （配置自体は正常で、デーモン起動までは到達した）。
- **対処（ユーザーの操作が1つ必要）**: Muse の設定 →
  Permissions → Direct network protocols で **other_tcp を Deny から
  Ask に変更**し、接続の承認を出す。その後に下記のコマンドを再実行すれば
  ビルドが進む見込み（依存ダウンロードも Java のプロキシ経由接続が
  必要で、同じ遮断の対象だった）。

  ```sh
  export JAVA_HOME=$HOME/workspace/toolchain/jdk17
  export ANDROID_HOME=$HOME/workspace/toolchain/android-sdk
  export GRADLE_USER_HOME=/home/hatch/.gradle   # Java の user.home は /root のため必須
  export PATH=$JAVA_HOME/bin:$HOME/workspace/toolchain/flutter/bin:$PATH
  cd app && flutter build apk --debug
  # 成功時の配布用コピー:
  #   mkdir -p ../release
  #   cp build/app/outputs/flutter-apk/app-debug.apk ../release/phone-robot-v0.1.0.apk
  ```
- そのため **APK はまだ生成できていない**。APK の中身確認
  （アプリ名・パーミッション・ARCore メタデータの構造チェック）は
  未実施で、ビルド成功後に行う。
- 環境メモ: プロキシ設定は `~/.gradle/gradle.properties` に systemProp で
  入れてある（この VM 専用、リポジトリには入れていない）。

## 要検討（勝手に決めていないこと）

- **ハートビート指令の導入可否**: OpenBot（同型の先行実装）は
  `h<time_ms>` というハートビート指令でファーム側ウォッチドッグを回す
  実績パターンがある。私たちのファームにも「400ms 通信断で停止」は
  入っているが、アプリ→ファームの定期ハートビート送信は現行プロトコルに
  無い。導入するならファーム側の仕様変更が要るため、今回は変更せず
  要検討として残す（現行でも走行中は指令が継続送信されるため、実害は
  「指令間隔が空くケースの検出速度」に限られる見込み）。
- **指令スケールの確定**: Dart 側 `SerialCommandScale.maxValue = 100` は
  仮置き。ファーム（ESP32 GPIO16-19 版）の drive/steer 受け入れ範囲は
  このワークスペースにソースが無く未確認。実機接続時に確定する。

## 実機でしか確認できないこと（実機テストは今回対象外）

- Note 10+ での ARCore トラッキング挙動（描画なし EGL 方式での
  `session.update()` の安定性、原点固定の効き、ドリフト量）。
- ARCore Depth の実出（Note 10+ の ToF 由来 Raw Depth の解像度・
  信頼度の出方、中央帯サンプリングと 1/cos(θ) 補正の妥当性）。
  深度が取れない環境では全方向「計測なし」のフォールバックになる
  （その場合、実機探索は近接停止が効かず回転スキャン頼みになる）。
- OTG 経由の ESP32 接続（CH340 の列挙、USB 権限ダイアログ、切断検知）。
- 実走そのもの: 前進/後進の符号、旋回方向と ste­er の符号、ステアリングが
  比例かオンオフか（最大の未知数）、回転スキャンの実挙動（車はその場で
  回る前提 — SPEC §7）。スマホのカメラ前方 ≒ 車の前方という取り付け前提も
  実機で合わせる。
- ファームの指令範囲（上記「指令スケール」）と 400ms ウォッチドッグの実挙動。

## 実機待ち・未検証のこと（旧記述の更新）

- Flutter のビルド確認は上記「APK ビルド挑戦」の通り、環境の権限で
  停止中（ソース側の analyze はクリーン）。
- ARCore / Depth API / USB シリアルのネイティブ実装は今回**実装済み**
  （Kotlin）。実機での動作確認だけが残る（上記リスト）。

## 設計メモ

- アルゴリズムは参照実装と同一にし、差し替えるのは「自己位置の出所」と
  「距離データの出所」だけ（SPEC の設計方針通り）。中核は MotionHooks
  インターフェース越しに車両を動かすため、シミュレーションと実機で
  同じ探索・ナビゲーションコードが使える。
- 空転防止（進展のない計画が2回連続したら終了/失敗）は、探索と
  ゴール指定ナビの両方に入れてある（Python 版で踏んだ知見の踏襲）。

## 第一段リスク R1〜R4 の修正（2026-10-30、NOTE10PLUS_RISKS 対応）

対象は `app/` のみ（core の API は無変更）。要点:

- **R1 トラッキング喪失への無自覚** → ポーズの受け渡しを `PoseFeed`
  （ポーズ＋受信時刻＋トラッキング状態）にし、RealDriver の駆動・回転の
  各周期でゲートを確認する。TRACKING でない／ポーズが 500 ms より古い
  間は駆動指令を出さず停止して復帰を待ち（上限 5 秒）、復帰したら
  `DriveOutcome.replan` で外側が現在位置から再計画する。復帰しなければ
  `sensorLost('tracking-lost')` で走行全体を止める。ポーズが不健全な
  スキャンは地図へ登録しない。
- **R2 近接フェイルセーフのフェイルオープン反転** → 駆動中はスキャンの
  鮮度ゲート（ScanFeed.isFresh、600 ms）を通った時だけ進む。古ければ
  停止して新しいスキャンを待ち（上限 2 秒）、来なければ
  `sensorLost('scan-stale')`。「データなし＝障害物なし」には倒れない。
- **R3 USB 切断の障害物誤認** → ドライバがリンク状態と送信結果
  （sendDriveSteer の成否）を各周期で見る。切断・送信失敗は
  `DriveOutcome.linkLost` で即停止し、地図には何も書かない。
  `_markBlockedAhead` は「リンク正常＋ポーズ健全＋スキャン新鮮」の
  ときだけ実行されるようガードし、「指令は届いたが進めなかった」
  （近接停止・タイムアウト）専用にした。自動再接続はしない
  （状態表示＋停止まで。接続し直しは手動）。
- **R4 画面消灯でセッションが黙って死ぬ** → センサー開始中は
  KeepAwake（新チャネル `phone_robot/system` → MainActivity で
  FLAG_KEEP_SCREEN_ON）で画面を常時点灯。RealSession に 500 ms 周期の
  ウォッチドッグを入れ、ポーズ／深度の無音（3 秒超）を検出して
  ドライバ停止＋状態表示（「センサー」行に無音秒数を表示）。
  配信が戻れば stall 判定は自動解除するが走行は自動再開しない。
  RealPage がライフサイクルを見て、背面遷移で走行を止め、復帰時は
  無音評価をやり直して「再開は手動・開始時に現在位置から再計画」と
  案内する。フォアグラウンドサービスは導入していない（リスク文書の
  段階付け通り後段。常時点灯＋前面運用で第一段は足りる判断）。

### 検証（この VM で実測）

- 単体テスト新規 11 件（`app/test/real_driver_test.dart` 7 件、
  `app/test/real_session_test.dart` 4 件）、全合格。フェイクの
  リンク／ポーズ／スキャンで、喪失時に止まる・古いスキャンで進まない・
  切断で壁を書かない（対照: 健全なタイムアウトでは書く）・復帰で
  replan になる、等を確認。
- `flutter analyze`（app、テスト含む）／`dart analyze`（core）: 両方
  No issues found。
- core 回帰: sim_mapping（coverage 97.63%、正解率 95.70%、ゴール
  5/5、goal-unknown の正直な失敗）、patrol_scenario（巡回 4/4 ×2、
  帰還成功、禁止エリア踏破 0、変化検出は「中央」のみ 15/84）—
  いずれも従来値と完全一致。

### 実機でしか最終確認できない点（R4 中心）

- 画面タイムアウト相当の状況で FLAG_KEEP_SCREEN_ON が実際に効くか、
  消灯・プロセス凍結が起きたときのウォッチドッグの見え方。
- ライフサイクル復帰後に ARCore／深度の EventChannel が配信を
  再開するか（再開しない場合は「センサー停止→開始」の手動手順が
  復帰手順になる。その判定材料として無音表示は出る）。
- ゲートの閾値（ポーズ 500 ms、復帰待ち 5 秒／2 秒）が実機の
  ARCore 配信間隔・深度更新間隔（公称 150 ms）と釣り合うか。

