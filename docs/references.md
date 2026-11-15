# REFERENCES — phone-robot Flutter アプリ開発の参考資料

2026-10-30 調査。すべて実際に開いて確認した一次資料（公式ドキュメント・公式/当事者リポジトリ）のみ。
各資料について「私たちのどの差し替えポイントに対応するか」「借りられる具体」「注意点」を書く。

## 対応表（先に結論）

| 私たちの課題 | 第一参照 |
|---|---|
| serial_link（USB 115200、ESP32 の CH340C） | usb-serial-for-android、OpenBot firmware |
| MotionHooks（経路→モーター指令、ウォッチドッグ） | OpenBot のシリアルプロトコル設計 |
| depth_source（Raw Depth＋信頼度→21本レイ） | ARCore 公式 raw_depth_java、Raw Depth ガイド |
| pose_source（ARCore ポーズ） | ARCore 公式 hello_ar_java、OpenBot の Point Goal Navigation |
| テスト（実車なしで pose/depth を再生） | ARCore Recording and Playback API |
| 変更検出（ARCore 中の静止画取得） | ARCore 公式 shared_camera_java |
| 意味層の端末内 VLM | Gemma-3n（LiteRT-LM 系） |
| Flutter 側の受け方 | Flutter 公式 platform-channels、EventChannel API |

---

## 1. OpenBot（Intel Labs 発祥）★最重要参照

- リポジトリ: https://github.com/ob-f/OpenBot
- Robot App 説明: https://github.com/ob-f/OpenBot/blob/master/robot/android/app/README.md
- Firmware 説明（プロトコル全文）: https://github.com/ob-f/OpenBot/blob/master/firmware/README.md
- Policy 学習: https://github.com/ob-f/OpenBot/blob/master/policy/README.md

私たちとほぼ同じ設計（スマホが頭脳、MCU がモーター変換だけ、USB シリアル）の先行実装。MIT ライセンス、スター約3,600、2026年10月時点でもコミットが続いている活きたプロジェクト。

### 対応する課題
serial_link、MotionHooks、アプリ全体構成、将来の学習ポリシー（段階2）。

### 借りられる具体
- **シリアルプロトコル設計**: 1文字プレフィックス＋カンマ区切りの書式は私たちの `"c,drive,steer"` と同型。モーター指令は `c<left>,<right>`（各 -255〜255）。**ハートビート指令 `h<time_ms>`（一定時間指令が来なければファーム側が停止）** があり、私たちの「400ms 通信断ウォッチドッグ」と同じ安全設計の実績例になる。
- **センサー報告は間隔指定式**: ソナー `s<ms>`、オドメトリ `w<ms>`、電圧 `v<ms>` のように「報告間隔を指令で変える」形。ESP32 側から電池電圧や速度を返す拡張をする際の書式の手本。
- **接続まわり**: USB はデフォルト 115200 baud・自動接続。私たちの serial_link のデフォルト値と一致する。ESP32 では Bluetooth 接続も併用可能にしている（将来ワイヤレス化の参照）。
- **端末内推論の実測値**: 物体検出は量子化済み TFLite（SSD MobileNetV1-300 がプリインストール、MobileNetV3-320、YOLOv4-tiny）。CPU/GPU/NNAPI のデリゲート切替 UI がある。**Snapdragon 855 機（Xiaomi Mi9、Pixel 4XL）の fps 実測表** があり、Note 10+（同じ SD855）の推論性能見積もりにそのまま使える。
- **Point Goal Navigation**: ARCore のポーズ＋学習ポリシーで「相対座標のゴールへ移動」を実装している。私たちの pose_source＋ゴールナビの構成が先行例として成立している証拠。
- **データ収集と学習の型**: 走行ログ（画像＋指令）を zip で保存し、約30分のデータで模倣学習（CIL-Mobile / PilotNet 系）→ `autopilot_float.tflite` をアプリの assets に入れて再配布。私たちの「MP4＋時刻つき CSV」ログ形式と Colab 学習の流れの参照実装になる。

### 注意点
- Robot App はネイティブ（Java/Kotlin）で Flutter ではない。コードの直接流用ではなく、プロトコルと設計判断を借りる使い方になる。なお OpenBot 自身の Flutter アプリは「コントローラー側」（別スマホから操作する側）で、頭脳側はネイティブに作っている。この事実自体が「ARCore はネイティブ実装＋チャネル接続」という私たちの方針の裏付けになる。
- OpenBot の ESP32 ファームをビルドする場合、ESP32 用 Arduino コアは **2.0.17 固定**（3.x 系は動かないと明記）。私たち自身のファームには無関係だが、比較実験で彼らのファームを焼くときに踏む。
- モーター指令が左右輪の差動式（`c<left>,<right>`）で、私たちの drive/steer 式とは車体構造が違う。書式の思想だけ借りて、値の意味は混ぜない。

---

## 2. ARCore 公式サンプルとガイド（Google）

### 2-1. raw_depth_java サンプル ★depth_source の正本
- https://github.com/google-ar/arcore-android-sdk/tree/master/samples/raw_depth_java
- ソースは `app/src/main/java/com/google/ar/core/examples/java/rawdepth/` に **DepthData.java**（生深度＋信頼度から幾何データを作る処理）、RawDepthActivity.java、PointCloudHelper.java、Renderer.java の4ファイル。

借りられる具体:
- Raw Depth の取得→信頼度フィルタ→利用可能な形へ変換、という depth_source ネイティブ側（ArCoreManager）の処理の流れがそのまま読める。21本化の前段処理はこれを土台にする。

### 2-2. Raw Depth ガイド（公式）
- https://developers.google.com/ar/develop/java/depth/raw-depth

借りられる具体（depth_source の閾値設計に直結）:
- `frame.acquireRawDepthImage16Bits()` と `frame.acquireRawDepthConfidenceImage()` の対で取得する。信頼度画像は Y8 形式で **0〜255**。
- Raw Depth は疎（まばら）: 深度のないピクセルは深度 0・信頼度 0。**「計測なし」をどう作るかの仕様**がここに定義されている（私たちの「負値＝計測なし」契約の根拠）。
- **白壁問題の公式見解**: テクスチャのない面は信頼度 0 になりやすい。ただし **ToF 等のハードウェア深度センサーがある機種では、近距離ならテクスチャなしでも信頼度が高く出る**。Note 10+ の ToF が効く範囲では白壁でもレイが立つ可能性がある。
- 毎フレーム新しい深度とは限らない。**タイムスタンプ比較で「新しい深度か」を判定**するのが公式手順。スキャンの二重更新を避けるのに必要。
- 計算コストは通常の Depth API の約半分。

### 2-3. Depth API ガイド（公式）
- https://developers.google.com/ar/develop/java/depth/developer-guide

借りられる具体:
- 有効化手順: `session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)` を確認してから `config.setDepthMode(...)`。depth_source の起動時チェックはこの形にする。
- 16bit 深度画像から mm 値を読むコード（`getMillimetersDepth`: プレーン0のバッファを pixelStride/rowStride で引く）が載っている。
- **⚠️ 重要な落とし穴: 深度値は「光線の長さ」ではなく「カメラ光軸への射影（z 座標）」**。私たちの21本レイは扇形の実距離（シミュレーションの Scan.ranges と同じ意味）なので、端のレイほど `1/cos(θ)` の補正が要る。補正しないと端のレイが系統的に短く出て、地図の壁が内側に湾曲する。
- カメラ画像座標↔深度画像座標の変換は `frame.transformCoordinates2d()` を使う（画角合わせの実装で必要）。

### 2-4. Frame API リファレンス（公式）
- https://developers.google.com/ar/reference/java/com/google/ar/core/Frame

借りられる具体:
- `acquireDepthImage16Bits()` / `acquireRawDepthImage16Bits()` / `acquireRawDepthConfidenceImage()` の3画像は**同一サイズ**。深度画像は通常 約160×120、機種により最大 640×480 まで変わる → 21本サンプリングの座標は固定値でなく画像サイズから計算すること。
- 8bit 版の acquire 系は非推奨（deprecated）。16bit 版を使う。
- `recordTrackData()` でカスタムデータを録画データセットに多重化できる（下記 2-5 と併用可）。

### 2-5. その他の公式サンプル（一覧ページで存在と役割を確認）
- サンプル一覧: https://developers.google.com/ar/discover/use-cases
- **hello_ar_java** — pose_source の基礎。セッションのライフサイクルと `frame.getCamera().getPose()` / トラッキング状態の扱いの最小形。
- **recording_playback_java**（Recording and Playback API）— ★テスト用。ARCore セッションをデータセットとして録画し、後から再生できる。**実車・実走行なしで pose/depth の入力を繰り返し再生して中核ロジックをテストできる**。私たちのログ形式（MP4＋CSV）と思想が近く、テスト基盤の第一候補。
- **shared_camera_java** — ARCore がカメラを使う間に Camera API 側でも画像を取る共有アクセス。変更検出や VLM 用の静止画を、ARCore セッションを止めずに撮るために必要。
- **arcore-ml-sample**（Kotlin）— ARCore のフレームを ML モデルの入力にする公式例。意味層・検出系を ARCore と同居させる際の参照。

注意点（ARCore 全般）:
- Depth 対応は機種依存。アプリ側は `isDepthModeSupported` の結果で「深度なしフォールバック」（私たちの depth_source 契約どおり全レイ計測なし）に落ちる設計を先に持つ。
- 深度は「動きから計算」される部分が大きく、静止直後や特徴の少ない環境では `NotYetAvailableException` が出るのが正常動作。例外＝異常として扱わない。

---

## 3. usb-serial-for-android（mik3y）★serial_link の正本

- https://github.com/mik3y/usb-serial-for-android

MIT ライセンス、スター約5,700、現行バージョン 3.11.0（JitPack 経由で導入）。Android の USB ホスト（OTG）でシリアル変換チップを叩く定番ライブラリ。

### 対応する課題
serial_link のネイティブ側実装そのもの。

### 借りられる具体
- **対応チップに Qinheng CH340 / CH341A が明記**されている。私たちの ESP32 基板（Lzxuan、CH340C 搭載）はこのドライバでカバーされる。CDC/ACM 汎用ドライバもあるので、将来 ESP32 ネイティブ USB の基板に替え
...[truncated 6085 chars]

---

## 付記. UCLA PhoneBot（RoMeLa）— スマホ頭脳の最新先行研究（2026-10-30 追記）

- 論文: arXiv:2610.08737（2026-10-06 公開）、プロジェクトページ: phonebot.dev
- 概要: Android スマホを胴体＝頭脳にする二足歩行ロボット。サーボ13個、機体約400ドル（スマホ除く）。スマホとは Wi-Fi で体側の小型コンピュータに指令を送る構成。

### 対応する課題
「スマホを頭脳にする」設計思想の最新の学術実証。安いスマホでどこまでいけるか、ハードの癖をどう吸収するかの一次データ。

### 借りられる具体（知見として）
- **sim-to-real の学習パイプライン**: MuJoCo で強化学習し、モーターのトルク特性・バックラッシュ（ガタ）までシミュレーションに正直に入れる。学習済み方策は TFLite でスマホ上 50Hz 実行。
- **癖への答えが「学習側」**: 左右の脚の非対称（癖）を、調整ではなく学習時の sagittal mirroring（左右対称化の工夫）で消し、対称誤差 0.254→0.003 rad。私たちの「機種別プロファイル＋テストコース採点」とは別の解き方で、学習層を持つ段階で効く。
- **安いスマホの実証範囲**: 50〜80ドルの機種3台＋2017年製 Honor 9 で歩行と画像認識が動作。唯一の失敗は Honor 9 の SLAM で、原因は性能ではなく米国で Google サービスが使えないこと（＝ソフトウェア環境の癖）。最低ライン議論の実データ。
- **層の分担**: 会話（言語モデル）は近くのノートPC、歩行と視覚はスマホ。意味の層を外に出す分担は私たちの設計と一致。

### 注意点
- 設計資料のライセンスは **CC BY-NC 4.0（非商用）**。ソフトウェアのコード公開は未完了との記録あり（robotworld.top の取り込み時点）。**事業に使う場合は設計・コードの複写は不可**。使えるのは概念と知見のみ。
- 平地のみ・腕なしの研究プラットフォーム。商品化はされていない。
