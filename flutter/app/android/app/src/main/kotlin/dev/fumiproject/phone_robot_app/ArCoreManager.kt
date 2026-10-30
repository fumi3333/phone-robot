package dev.fumiproject.phone_robot_app

import android.Manifest
import android.app.Activity
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.GLES20
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.util.Log
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.UnavailableException
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel
import kotlin.math.atan2
import kotlin.math.roundToInt
import kotlin.math.tan

/**
 * ARCore のセッション管理と、Dart 側チャネルへの配信。
 *
 * チャネル契約（Dart: platform/pose_source.dart, depth_source.dart）:
 *  - MethodChannel "phone_robot/arcore": startPoseTracking / stopPoseTracking
 *  - EventChannel "phone_robot/arcore/pose": {x, y, heading}（SPEC §1 の部屋座標）
 *  - EventChannel "phone_robot/arcore/tracking": "TRACKING" / "PAUSED" /
 *    "STOPPED" / "UNSUPPORTED" / "ERROR"
 *  - EventChannel "phone_robot/depth": {ranges: [21本, m]}. 計測なしは -1.0。
 *
 * 座標変換（SPEC §5.1）: セッション開始後、最初に TRACKING になった瞬間の
 * カメラポーズを部屋の原点・方位基準に固定し、以降は相対ポーズから
 *   x = relTx, y = -relTz, heading = atan2(-forwardZ, forwardX)
 * を計算する（ARCore は右手系・-Z がカメラ前方、部屋座標は y-up 前方）。
 *
 * 深度（SPEC §5.2）: Raw Depth（16bit）＋信頼度画像から、画面中央帯の
 * 21 方向（-30°〜+30°、3°刻み）について距離の中央値を取る。
 * 信頼度が低い画素・範囲外は使わない。サンプルが無い方向は -1.0（計測なし）。
 * Depth 非対応の環境では常に全方向 -1.0 を返す（フォールバック）。
 *
 * 描画無しでトラッキングを回すため、オフスクリーンの EGL pbuffer と
 * ダミーの SurfaceTexture を作り、専用スレッドで session.update() を回す。
 */
class ArCoreManager(private val activity: Activity) {

    companion object {
        private const val TAG = "ArCoreManager"
        private const val CAMERA_REQUEST_CODE = 0xA001
        private const val RAY_COUNT = 21
        private const val DEPTH_INTERVAL_MS = 150L
        private const val FALLBACK_INTERVAL_MS = 1000L
    }

    @Volatile private var session: Session? = null
    @Volatile private var running = false
    private var worker: HandlerThread? = null

    @Volatile private var depthSupported = false
    private var originPose: Pose? = null
    private var lastTracking: String = ""
    private var lastDepthAt: Long = 0L

    /** 前回配信した深度画像のタイムスタンプ（新しい深度かの判定用）。 */
    private var lastDepthTimestamp: Long = -1L

    private var poseSink: EventChannel.EventSink? = null
    private var trackingSink: EventChannel.EventSink? = null
    private var depthSink: EventChannel.EventSink? = null
    private val mainHandler = Handler(Looper.getMainLooper())

    private var pendingResult: MethodChannel.Result? = null

    val poseHandler = object : EventChannel.StreamHandler {
        override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
            poseSink = events
        }
        override fun onCancel(arguments: Any?) { poseSink = null }
    }

    val trackingHandler = object : EventChannel.StreamHandler {
        override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
            trackingSink = events
        }
        override fun onCancel(arguments: Any?) { trackingSink = null }
    }

    val depthHandler = object : EventChannel.StreamHandler {
        override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
            depthSink = events
        }
        override fun onCancel(arguments: Any?) { depthSink = null }
    }

    // ---------------------------------------------------------------- 開始
    fun start(result: MethodChannel.Result) {
        if (running) {
            result.success(null)
            return
        }
        if (ContextCompat.checkSelfPermission(activity, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED
        ) {
            pendingResult = result
            ActivityCompat.requestPermissions(
                activity, arrayOf(Manifest.permission.CAMERA), CAMERA_REQUEST_CODE
            )
            return
        }
        startSession(result)
    }

    fun onRequestPermissionsResult(requestCode: Int, grantResults: IntArray) {
        if (requestCode != CAMERA_REQUEST_CODE) return
        val result = pendingResult ?: return
        pendingResult = null
        if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startSession(result)
        } else {
            result.error("camera_permission", "カメラ権限が許可されませんでした", null)
        }
    }

    private fun startSession(result: MethodChannel.Result) {
        try {
            val availability = ArCoreApk.getInstance().checkAvailability(activity)
            if (availability == ArCoreApk.Availability.UNSUPPORTED_DEVICE_NOT_CAPABLE) {
                pushTracking("UNSUPPORTED")
                result.error("arcore_unsupported", "この端末は ARCore に未対応です", null)
                return
            }
            val installStatus = ArCoreApk.getInstance().requestInstall(activity, true)
            if (installStatus == ArCoreApk.InstallStatus.INSTALL_REQUESTED) {
                // インストール画面へ遷移した。戻ってきたらもう一度開始してもらう。
                result.error(
                    "arcore_install",
                    "ARCore（Google Play 開発者サービス）の導入が必要です。完了後にもう一度開始してください",
                    null
                )
                return
            }
            val s = Session(activity)
            val config = Config(s)
            depthSupported = s.isDepthModeSupported(Config.DepthMode.AUTOMATIC)
            config.depthMode =
                if (depthSupported) Config.DepthMode.AUTOMATIC else Config.DepthMode.DISABLED
            config.focusMode = Config.FocusMode.AUTO
            config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            s.configure(config)
            session = s
            originPose = null
            lastTracking = ""
            lastDepthTimestamp = -1L
            running = true
            startWorker(s)
            result.success(null)
        } catch (e: UnavailableException) {
            Log.w(TAG, "ARCore unavailable: ${e.message}")
            pushTracking("UNSUPPORTED")
            result.error("arcore_unavailable", "ARCore を開始できません: ${e.message}", null)
        } catch (e: Exception) {
            Log.w(TAG, "ARCore start failed: ${e.message}")
            pushTracking("ERROR")
            result.error("arcore_error", "ARCore 開始エラー: ${e.message}", null)
        }
    }

    // ---------------------------------------------------------------- 停止
    fun stop() {
        if (!running && session == null) return
        running = false
        try {
            session?.pause()
        } catch (e: Exception) {
            Log.w(TAG, "pause failed: ${e.message}")
        }
        try {
            worker?.quitSafely()
            worker?.join(500)
        } catch (e: Exception) {
            Log.w(TAG, "worker stop failed: ${e.message}")
        }
        worker = null
        try {
            session?.close()
        } catch (e: Exception) {
            Log.w(TAG, "close failed: ${e.message}")
        }
        session = null
        originPose = null
        pushTracking("STOPPED")
    }

    // ------------------------------------------------------- 更新ループ
    private fun startWorker(s: Session) {
        val thread = HandlerThread("arcore-update-loop")
        thread.start()
        worker = thread
        Handler(thread.looper).post { glLoop(s) }
    }

    /** 描画なしで session.update() を回すための最小 EGL 環境でループする。 */
    private fun glLoop(s: Session) {
        val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        val version = IntArray(2)
        EGL14.eglInitialize(display, version, 0, version, 1)
        val configAttribs = intArrayOf(
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_ALPHA_SIZE, 8,
            EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
            EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
            EGL14.EGL_NONE
        )
        val configs = arrayOfNulls<EGLConfig>(1)
        val numConfigs = IntArray(1)
        EGL14.eglChooseConfig(display, configAttribs, 0, configs, 0, 1, numConfigs, 0)
        val contextAttribs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
        val context = EGL14.eglCreateContext(display, configs[0], EGL14.EGL_NO_CONTEXT, contextAttribs, 0)
        val surfaceAttribs = intArrayOf(EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE)
        val surface = EGL14.eglCreatePbufferSurface(display, configs[0], surfaceAttribs, 0)
        EGL14.eglMakeCurrent(display, surface, surface, context)

        val textureIds = IntArray(1)
        GLES20.glGenTextures(1, textureIds, 0)
        val surfaceTexture = SurfaceTexture(textureIds[0])
        try {
            s.setCameraTextureName(textureIds[0])
            s.resume()
        } catch (e: Exception) {
            Log.w(TAG, "session resume failed: ${e.message}")
            pushTracking("ERROR")
            running = false
        }
        while (running) {
            try {
                val frame = s.update()
                try {
                    surfaceTexture.updateTexImage()
                } catch (e: Exception) {
                    // テクスチャ更新失敗は致命的ではない（次フレームで回復する）
                }
                onFrame(frame)
            } catch (e: Exception) {
                if (!running) break
                Log.w(TAG, "update failed: ${e.message}")
                try {
                    Thread.sleep(50)
                } catch (ie: InterruptedException) {
                    break
                }
            }
        }
        surfaceTexture.release()
        EGL14.eglMakeCurrent(
            display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT
        )
        EGL14.eglDestroySurface(display, surface)
        EGL14.eglDestroyContext(display, context)
        EGL14.eglTerminate(display)
    }

    private fun onFrame(frame: Frame) {
        val camera = frame.camera
        val stateName = when (camera.trackingState) {
            TrackingState.TRACKING -> "TRACKING"
            TrackingState.PAUSED -> "PAUSED"
            TrackingState.STOPPED -> "STOPPED"
            else -> "UNKNOWN"
        }
        if (stateName != lastTracking) {
            lastTracking = stateName
            pushTracking(stateName)
        }
        if (camera.trackingState == TrackingState.TRACKING) {
            if (originPose == null) {
                // 最初の TRACKING フレームを部屋の原点・方位基準に固定（SPEC §5.1）
                originPose = camera.pose
            }
            val origin = originPose
            if (origin != null) {
                val (x, y, heading) = relativePose(origin, camera.pose)
                pushPose(x, y, heading)
            }
        }
        val now = System.currentTimeMillis()
        if (depthSupported) {
            if (now - lastDepthAt >= DEPTH_INTERVAL_MS) {
                lastDepthAt = now
                val rays = computeRays(frame)
                if (rays != null) {
                    pushDepth(rays)
                }
            }
        } else if (now - lastDepthAt >= FALLBACK_INTERVAL_MS) {
            // Depth 非対応環境のフォールバック: 常に「計測なし」（SPEC §5.2）
            lastDepthAt = now
            pushDepth(DoubleArray(RAY_COUNT) { -1.0 })
        }
    }

    // ------------------------------------------------------- 座標変換
    /** 四元数（x, y, z, w）。Pose 変換を API 依存なく自前で計算するための最小実装。 */
    private data class Quat(val x: Double, val y: Double, val z: Double, val w: Double) {
        fun conjugate() = Quat(-x, -y, -z, w)

        operator fun times(o: Quat) = Quat(
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w,
            w * o.w - x * o.x - y * o.y - z * o.z
        )

        /** ベクトル (vx, vy, vz) をこの四元数で回転する。 */
        fun rotate(vx: Double, vy: Double, vz: Double): DoubleArray {
            val tx = 2.0 * (y * vz - z * vy)
            val ty = 2.0 * (z * vx - x * vz)
            val tz = 2.0 * (x * vy - y * vx)
            return doubleArrayOf(
                vx + w * tx + (y * tz - z * ty),
                vy + w * ty + (z * tx - x * tz),
                vz + w * tz + (x * ty - y * tx)
            )
        }
    }

    private fun quatOf(pose: Pose): Quat {
        val q = FloatArray(4)
        pose.getRotationQuaternion(q, 0)
        return Quat(q[0].toDouble(), q[1].toDouble(), q[2].toDouble(), q[3].toDouble())
    }

    /** origin を基準にした cam の相対ポーズを SPEC §1 の部屋座標へ変換する。 */
    private fun relativePose(origin: Pose, cam: Pose): Triple<Double, Double, Double> {
        val qoInv = quatOf(origin).conjugate()
        val relQ = qoInv * quatOf(cam)
        val relT = qoInv.rotate(
            (cam.tx() - origin.tx()).toDouble(),
            (cam.ty() - origin.ty()).toDouble(),
            (cam.tz() - origin.tz()).toDouble()
        )
        // カメラ前方 = 回転行列に (0,0,-1) を掛けたもの（第3列の符号反転）
        val r02 = 2.0 * (relQ.x * relQ.z + relQ.w * relQ.y)
        val r22 = 1.0 - 2.0 * (relQ.x * relQ.x + relQ.y * relQ.y)
        val forwardX = -r02
        val forwardZ = -r22
        val x = relT[0]
        val y = -relT[2]
        val heading = atan2(-forwardZ, forwardX)
        return Triple(x, y, heading)
    }

    // ------------------------------------------------------- Depth → 21本レイ
    /**
     * 深度画像から 21 本レイを作る。新しい深度画像でないフレームでは null を
     * 返し、配信しない（タイムスタンプ比較が ARCore 公式の判定手順。
     * スキャンの二重更新を防ぐ）。
     */
    private fun computeRays(frame: Frame): DoubleArray? {
        val rays = DoubleArray(RAY_COUNT) { -1.0 }
        var depthImage: android.media.Image? = null
        var confidenceImage: android.media.Image? = null
        try {
            depthImage = frame.acquireRawDepthImage16Bits()
            if (depthImage.timestamp == lastDepthTimestamp) {
                return null // 前回配信したのと同じ深度: 二重更新を避ける
            }
            lastDepthTimestamp = depthImage.timestamp
            confidenceImage = frame.acquireRawDepthConfidenceImage()
            val w = depthImage.width
            val h = depthImage.height
            val depthPlane = depthImage.planes[0]
            val depthBuffer = depthPlane.buffer
            val depthRowStride = depthPlane.rowStride
            val depthPixelStride = depthPlane.pixelStride
            val confPlane = confidenceImage.planes[0]
            val confBuffer = confPlane.buffer
            val confRowStride = confPlane.rowStride
            val confPixelStride = confPlane.pixelStride

            // 深度画像の水平画角は機種で違うため、テクスチャ側の内部パラメータを
            // 深度画像の解像度へ換算してレイ方向→列の対応を作る（SPEC §5.2-3 の近似。
            // 厳密には機種ごとのキャリブレーションが望ましい）
            val intr = frame.camera.textureIntrinsics
            val texW = intr.imageDimensions[0].toDouble()
            val fxDepth = intr.focalLength[0].toDouble() * w / texW
            val cxDepth = intr.principalPoint[0].toDouble() * w / texW

            val rowFrom = (h * 0.40).toInt()
            val rowTo = (h * 0.60).toInt().coerceAtLeast(rowFrom + 1)
            for (i in 0 until RAY_COUNT) {
                val angleRad = Math.toRadians(-30.0 + 3.0 * i)
                var u = (cxDepth + fxDepth * tan(angleRad)).roundToInt()
                if (u < 0) u = 0
                if (u > w - 1) u = w - 1
                val z = sampleRay(
                    depthBuffer, depthRowStride, depthPixelStride,
                    confBuffer, confRowStride, confPixelStride,
                    w, u, rowFrom, rowTo
                )
                if (z > 0) {
                    // ⚠ ARCore の深度値は「光線の実距離」ではなく「光軸への射影
                    // （z値）」。扇形レイの実距離に直すには 1/cos(θ) の補正が要る
                    // （ARCore 公式 Depth ガイド。放置すると壁が内側に湾曲する）
                    rays[i] = z / kotlin.math.cos(angleRad)
                }
            }
        } catch (e: NotYetAvailableException) {
            // このフレームにはまだ深度が無い: null を返して配信しない
            return null
        } catch (e: Exception) {
            Log.w(TAG, "depth compute failed: ${e.message}")
        } finally {
            try {
                depthImage?.close()
            } catch (e: Exception) {
            }
            try {
                confidenceImage?.close()
            } catch (e: Exception) {
            }
        }
        return rays
    }

    /**
     * 中央列 u（前後 ±2 列まで広げる）× 中央帯の行について、信頼度が高く
     * 範囲内（100〜5000mm）の深度値の中央値をメートルで返す。無ければ -1.0。
     */
    private fun sampleRay(
        depthBuffer: java.nio.ByteBuffer,
        depthRowStride: Int,
        depthPixelStride: Int,
        confBuffer: java.nio.ByteBuffer,
        confRowStride: Int,
        confPixelStride: Int,
        imageWidth: Int,
        centerU: Int,
        rowFrom: Int,
        rowTo: Int
    ): Double {
        val values = ArrayList<Int>()
        for (du in -2..2) {
            val u = centerU + du
            if (u < 0 || u >= imageWidth) continue
            var r = rowFrom
            while (r < rowTo) {
                val dIndex = r * depthRowStride + u * depthPixelStride
                if (dIndex + 1 < depthBuffer.limit()) {
                    val lo = depthBuffer.get(dIndex).toInt() and 0xFF
                    val hi = depthBuffer.get(dIndex + 1).toInt() and 0xFF
                    val mm = lo or (hi shl 8)
                    val cIndex = r * confRowStride + u * confPixelStride
                    val confidence =
                        if (cIndex < confBuffer.limit()) confBuffer.get(cIndex).toInt() and 0xFF else 0
                    // 信頼度 0..255 のうち高信頼（>=128）だけを採用（SPEC §5.2）
                    if (confidence >= 128 && mm in 100..5000) {
                        values.add(mm)
                    }
                }
                r += 2 // 行は間引いてサンプル数を抑える
            }
        }
        if (values.isEmpty()) return -1.0
        values.sort()
        return values[values.size / 2] / 1000.0
    }

    // ------------------------------------------------------- 配信
    private fun pushPose(x: Double, y: Double, heading: Double) {
        mainHandler.post {
            poseSink?.success(mapOf("x" to x, "y" to y, "heading" to heading))
        }
    }

    private fun pushTracking(state: String) {
        mainHandler.post { trackingSink?.success(state) }
    }

    private fun pushDepth(rays: DoubleArray) {
        val list = rays.toList()
        mainHandler.post { depthSink?.success(mapOf("ranges" to list)) }
    }
}
