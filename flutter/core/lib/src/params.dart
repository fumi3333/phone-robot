/// マッピングの主要パラメータ（SPEC §4 の参照値＋シミュレーション用センサー値）。
///
/// 座標系・単位は SPEC §1: 単位は m / rad、世界座標は部屋の南西隅が原点、
/// x = 東、y = 北、heading は 0 = +x で反時計回りが正。
class MappingParams {
  /// セル一辺 [m]
  final double cellSize;

  /// グリッド列数（x 方向）
  final int gridW;

  /// グリッド行数（y 方向）
  final int gridH;

  /// センサー最大距離 [m]
  final double maxRange;

  /// センサー最小距離 [m]
  final double minRange;

  /// 深度カメラの扇形視野 [rad]（60°）
  final double fovRad;

  /// 1 スキャンのレイ本数
  final int nRays;

  /// レイマーチング刻み [m]（シミュレーションの真値レイキャスト用）
  final double rayDs;

  /// 距離計測ノイズ σ [m]（シミュレーション用）
  final double rangeNoise;

  /// 自己位置ノイズ σ [m]（通常モード、スキャン毎）
  final double posePosNoise;

  /// 向きノイズ σ [rad]（通常モード、スキャン毎、1.5°）
  final double poseHeadNoise;

  /// ドリフトモードの位置ランダムウォーク σ [m/移動]
  final double driftPosStep;

  /// ドリフトモードの向きランダムウォーク σ [rad/移動]（0.15°）
  final double driftHeadStep;

  /// ログオッズ増分（命中）
  final double lOcc;

  /// ログオッズ増分（空間）
  final double lFree;

  /// ログオッズのクランプ
  final double lClamp;

  /// 計画用「走行可能」のしきい値（SPEC §3.1 / §6）
  final double planFreeL;

  /// 終端手前で free 加算を止める安全マージン [m]（SPEC §6）
  final double freeMargin;

  /// 探索の終了目安カバー率
  final double targetCoverage;

  /// 探索の移動ステップ上限
  final int maxSteps;

  const MappingParams({
    this.cellSize = 0.05,
    this.gridW = 120,
    this.gridH = 100,
    this.maxRange = 5.0,
    this.minRange = 0.10,
    this.fovRad = 1.0471975511965976, // 60°
    this.nRays = 21,
    this.rayDs = 0.025,
    this.rangeNoise = 0.03,
    this.posePosNoise = 0.02,
    this.poseHeadNoise = 0.026179938779914943, // 1.5°
    this.driftPosStep = 0.012,
    this.driftHeadStep = 0.0026179938779914944, // 0.15°
    this.lOcc = 0.85,
    this.lFree = -0.40,
    this.lClamp = 4.0,
    this.planFreeL = -0.8,
    this.freeMargin = 0.10,
    this.targetCoverage = 0.95,
    this.maxSteps = 20000,
  });

  int get cellCount => gridW * gridH;
  double get roomW => gridW * cellSize;
  double get roomH => gridH * cellSize;
}
