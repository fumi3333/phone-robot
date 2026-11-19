package dev.fumiproject.phone_robot_app

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.net.Uri
import android.opengl.EGL14
import android.opengl.EGLConfig
import android.opengl.EGLContext
import android.opengl.EGLDisplay
import android.opengl.EGLSurface
import android.opengl.GLES11Ext
import android.opengl.GLES20
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.PowerManager
import android.util.Log
import android.view.Surface
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Coordinates2d
import com.google.ar.core.Frame
import com.google.ar.core.Pose
import com.google.ar.core.RecordingConfig
import com.google.ar.core.RecordingStatus
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import com.google.ar.core.exceptions.UnavailableException
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel
import io.flutter.view.TextureRegistry
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import kotlin.math.atan2
import kotlin.math.hypot

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
class ArCoreManager(
    private val activity: Activity,
    private val textureRegistry: TextureRegistry? = null
) {

    companion object {
        private const val TAG = "ArCoreManager"
        private const val CAMERA_REQUEST_CODE = 0xA001
        private const val RAY_COUNT = 21
        private const val FRAME_INTERVAL_MS = 33L
        private const val POSE_INTERVAL_MS = 33L
        private const val PREVIEW_INTERVAL_MS = 100L
        private const val DEPTH_INTERVAL_MS = 120L
        private const val THERMAL_PREVIEW_INTERVAL_MS = 250L
        private const val THERMAL_DEPTH_INTERVAL_MS = 300L
        private const val FALLBACK_INTERVAL_MS = 1000L
        private const val PREVIEW_WIDTH = 640
        private const val PREVIEW_HEIGHT = 480
    }

    @Volatile private var session: Session? = null
    @Volatile private var running = false
    private var worker: HandlerThread? = null

    @Volatile private var depthSupported = false
    private var originPose: Pose? = null
    private var lastTracking: String = ""
    private var lastDepthAt: Long = 0L
    private var lastPosePushAt: Long = 0L
    private var lastPreviewAt: Long = 0L
    private var lastCameraTimestamp: Long = -1L

    private var flutterTextureEntry: TextureRegistry.SurfaceTextureEntry? = null
    private var activeMp4Path: String? = null

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

    fun getCameraTextureId(): Long {
        if (flutterTextureEntry == null && textureRegistry != null) {
            try {
                flutterTextureEntry = textureRegistry.createSurfaceTexture().also {
                    it.surfaceTexture().setDefaultBufferSize(PREVIEW_WIDTH, PREVIEW_HEIGHT)
                }
            } catch (e: Exception) {
                Log.w(TAG, "createSurfaceTexture failed: ${e.message}")
            }
        }
        return flutterTextureEntry?.id() ?: -1L
    }

    fun startMp4Recording(rawRunId: String): String? {
        val safeRunId = rawRunId.trim().replace(Regex("[^A-Za-z0-9_-]"), "")
        if (safeRunId.isEmpty()) return null
        val s = session ?: return null
        try {
            if (s.recordingStatus == RecordingStatus.OK) {
                s.stopRecording()
            }
            val runsRoot = activity.getExternalFilesDir("runs") ?: return null
            val runDir = File(runsRoot, "run_$safeRunId")
            if (!runDir.exists()) {
                runDir.mkdirs()
            }
            val mp4File = File(runDir, "camera_recording_$safeRunId.mp4")
            val recConfig = RecordingConfig(s)
                .setMp4DatasetUri(Uri.fromFile(mp4File))
                .setAutoStopOnPause(true)
            s.startRecording(recConfig)
            activeMp4Path = mp4File.absolutePath
            return activeMp4Path
        } catch (e: Exception) {
            Log.w(TAG, "startMp4Recording failed: ${e.message}")
            activeMp4Path = null
            return null
        }
    }

    fun stopMp4Recording(): String? {
        val path = activeMp4Path
        activeMp4Path = null
        val s = session
        if (s != null) {
            try {
                if (s.recordingStatus == RecordingStatus.OK) {
                    s.stopRecording()
                }
            } catch (e: Exception) {
                Log.w(TAG, "stopMp4Recording failed: ${e.message}")
            }
        }
        return if (path != null && File(path).exists()) path else null
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
            getCameraTextureId()
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
            lastDepthAt = 0L
            lastPosePushAt = 0L
            lastPreviewAt = 0L
            lastCameraTimestamp = -1L
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
        stopMp4Recording()
        running = false
        try {
            worker?.quitSafely()
            worker?.join(600)
        } catch (e: Exception) {
            Log.w(TAG, "worker stop failed: ${e.message}")
        }
        worker = null
        try {
            session?.pause()
        } catch (e: Exception) {
            Log.w(TAG, "pause failed: ${e.message}")
        }
        try {
            session?.close()
        } catch (e: Exception) {
            Log.w(TAG, "close failed: ${e.message}")
        }
        session = null
        originPose = null
        lastCameraTimestamp = -1L
        val entry = flutterTextureEntry
        flutterTextureEntry = null
        if (entry != null) {
            if (Looper.myLooper() == Looper.getMainLooper()) {
                try {
                    entry.release()
                } catch (_: Exception) {
                }
            } else {
                mainHandler.post {
                    try {
                        entry.release()
                    } catch (_: Exception) {
                    }
                }
            }
        }
        pushTracking("STOPPED")
    }

    // ------------------------------------------------------- 更新ループ
    private fun startWorker(s: Session) {
        val thread = HandlerThread("arcore-update-loop")
        thread.start()
        worker = thread
        Handler(thread.looper).post { glLoop(s) }
    }

    @Suppress("DEPRECATION")
    private fun currentDisplayRotation(): Int {
        return try {
            activity.windowManager.defaultDisplay?.rotation ?: Surface.ROTATION_0
        } catch (_: Exception) {
            Surface.ROTATION_0
        }
    }

    private fun displaySizeForRotation(rot: Int): Pair<Int, Int> {
        return if (rot == Surface.ROTATION_0 || rot == Surface.ROTATION_180) {
            PREVIEW_HEIGHT to PREVIEW_WIDTH
        } else {
            PREVIEW_WIDTH to PREVIEW_HEIGHT
        }
    }

    /** 最小 EGL 環境で session.update() を 30Hz 上限で回し、ライブプレビュー用テクスチャも更新する。 */
    private fun glLoop(s: Session) {
        val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
        val version = IntArray(2)
        EGL14.eglInitialize(display, version, 0, version, 1)

        val configs = arrayOfNulls<EGLConfig>(1)
        val numConfigs = IntArray(1)
        val bothAttribs = intArrayOf(
            EGL14.EGL_RED_SIZE, 8,
            EGL14.EGL_GREEN_SIZE, 8,
            EGL14.EGL_BLUE_SIZE, 8,
            EGL14.EGL_ALPHA_SIZE, 8,
            EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
            EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT or EGL14.EGL_WINDOW_BIT,
            EGL14.EGL_NONE
        )
        EGL14.eglChooseConfig(display, bothAttribs, 0, configs, 0, 1, numConfigs, 0)
        if (numConfigs[0] <= 0 || configs[0] == null) {
            val pbufferOnlyAttribs = intArrayOf(
                EGL14.EGL_RED_SIZE, 8,
                EGL14.EGL_GREEN_SIZE, 8,
                EGL14.EGL_BLUE_SIZE, 8,
                EGL14.EGL_ALPHA_SIZE, 8,
                EGL14.EGL_RENDERABLE_TYPE, EGL14.EGL_OPENGL_ES2_BIT,
                EGL14.EGL_SURFACE_TYPE, EGL14.EGL_PBUFFER_BIT,
                EGL14.EGL_NONE
            )
            EGL14.eglChooseConfig(display, pbufferOnlyAttribs, 0, configs, 0, 1, numConfigs, 0)
        }
        val chosenConfig = configs[0]
        if (chosenConfig == null) {
            Log.w(TAG, "No matching EGLConfig")
            pushTracking("ERROR")
            EGL14.eglTerminate(display)
            return
        }

        val contextAttribs = intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 2, EGL14.EGL_NONE)
        val context = EGL14.eglCreateContext(display, chosenConfig, EGL14.EGL_NO_CONTEXT, contextAttribs, 0)
        val surfaceAttribs = intArrayOf(EGL14.EGL_WIDTH, 1, EGL14.EGL_HEIGHT, 1, EGL14.EGL_NONE)
        val pbufferSurface = EGL14.eglCreatePbufferSurface(display, chosenConfig, surfaceAttribs, 0)
        EGL14.eglMakeCurrent(display, pbufferSurface, pbufferSurface, context)

        val textureIds = IntArray(1)
        GLES20.glGenTextures(1, textureIds, 0)
        GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureIds[0])
        GLES20.glTexParameteri(
            GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_MIN_FILTER,
            GLES20.GL_LINEAR
        )
        GLES20.glTexParameteri(
            GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_MAG_FILTER,
            GLES20.GL_LINEAR
        )
        GLES20.glTexParameteri(
            GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_WRAP_S,
            GLES20.GL_CLAMP_TO_EDGE
        )
        GLES20.glTexParameteri(
            GLES11Ext.GL_TEXTURE_EXTERNAL_OES,
            GLES20.GL_TEXTURE_WRAP_T,
            GLES20.GL_CLAMP_TO_EDGE
        )
        val surfaceTexture = SurfaceTexture(textureIds[0])

        val previewBlitter = PreviewBlitter(
            display = display,
            eglConfig = chosenConfig,
            context = context,
            pbufferSurface = pbufferSurface,
            cameraTextureId = textureIds[0],
            targetTexture = flutterTextureEntry?.surfaceTexture()
        )
        previewBlitter.init()

        var lastRotation = -1
        var lastRotationCheckAt = 0L

        try {
            lastRotation = currentDisplayRotation()
            val (dispW, dispH) = displaySizeForRotation(lastRotation)
            s.setDisplayGeometry(lastRotation, dispW, dispH)
            s.setCameraTextureName(textureIds[0])
            s.resume()
        } catch (e: Exception) {
            Log.w(TAG, "session resume failed: ${e.message}")
            pushTracking("ERROR")
            running = false
        }

        while (running) {
            val loopStart = System.currentTimeMillis()
            try {
                if (loopStart - lastRotationCheckAt >= 500L) {
                    lastRotationCheckAt = loopStart
                    val rot = currentDisplayRotation()
                    if (rot != lastRotation) {
                        lastRotation = rot
                        val (dispW, dispH) = displaySizeForRotation(rot)
                        s.setDisplayGeometry(rot, dispW, dispH)
                    }
                }
                val frame = s.update()
                val camTs = frame.timestamp
                if (camTs == 0L || camTs == lastCameraTimestamp) {
                    // LATEST_CAMERA_IMAGE はノンブロッキングのため、未更新時は待機してバッテリー消耗を防ぐ
                    try {
                        Thread.sleep(8)
                    } catch (_: InterruptedException) {
                        break
                    }
                    continue
                }
                lastCameraTimestamp = camTs
                try {
                    surfaceTexture.updateTexImage()
                } catch (_: Exception) {
                    // テクスチャ更新失敗は次フレームで回復する
                }
                val previewInterval =
                    if (isThermalThrottled(loopStart)) THERMAL_PREVIEW_INTERVAL_MS else PREVIEW_INTERVAL_MS
                if (loopStart - lastPreviewAt >= previewInterval) {
                    lastPreviewAt = loopStart
                    previewBlitter.drawFrame(frame)
                }
                onFrame(frame)
            } catch (e: Exception) {
                if (!running) break
                Log.w(TAG, "update failed: ${e.message}")
                try {
                    Thread.sleep(50)
                } catch (_: InterruptedException) {
                    break
                }
            }
            val elapsed = System.currentTimeMillis() - loopStart
            val remaining = FRAME_INTERVAL_MS - elapsed
            if (remaining > 2L && running) {
                try {
                    Thread.sleep(remaining.coerceAtMost(25L))
                } catch (_: InterruptedException) {
                    break
                }
            }
        }

        previewBlitter.release()
        surfaceTexture.release()
        EGL14.eglMakeCurrent(
            display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT
        )
        EGL14.eglDestroySurface(display, pbufferSurface)
        EGL14.eglDestroyContext(display, context)
        EGL14.eglTerminate(display)
    }

    private fun onFrame(frame: Frame) {
        val now = System.currentTimeMillis()
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
            if (origin != null && now - lastPosePushAt >= POSE_INTERVAL_MS) {
                lastPosePushAt = now
                val (x, y, heading) = relativePose(origin, camera.pose)
                pushPose(x, y, heading)
            }
        }
        if (depthSupported && camera.trackingState == TrackingState.TRACKING) {
            val depthInterval =
                if (isThermalThrottled(now)) THERMAL_DEPTH_INTERVAL_MS else DEPTH_INTERVAL_MS
            if (now - lastDepthAt >= depthInterval) {
                lastDepthAt = now
                val rays = computeRays(frame)
                if (rays != null) {
                    pushDepth(rays)
                }
            }
        } else if (!depthSupported && now - lastDepthAt >= FALLBACK_INTERVAL_MS) {
            // Depth 非対応環境のフォールバック: 常に「計測なし」（SPEC §5.2）
            lastDepthAt = now
            pushDepth(DoubleArray(RAY_COUNT) { -1.0 })
        }
    }

    @Volatile private var lastThermalCheckAt = 0L
    @Volatile private var thermalThrottled = false
    private val powerManager: PowerManager? by lazy {
        activity.getSystemService(Context.POWER_SERVICE) as? PowerManager
    }

    private fun isThermalThrottled(nowMs: Long): Boolean {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) return false
        if (nowMs - lastThermalCheckAt >= 2000L) {
            lastThermalCheckAt = nowMs
            val status = powerManager?.currentThermalStatus ?: PowerManager.THERMAL_STATUS_NONE
            thermalThrottled = status >= PowerManager.THERMAL_STATUS_MODERATE
        }
        return thermalThrottled
    }

    // ------------------------------------------------------- 座標変換
    private data class HorizBasis(
        val fwdX: Double,
        val fwdZ: Double,
        val rightX: Double,
        val rightZ: Double,
        val leftX: Double,
        val leftZ: Double
    )

    /** Pose から重力整合ワールド水平面 (X, Z) 上の前方・右・左単位ベクトルを求める。 */
    private fun horizontalBasis(pose: Pose): HorizBasis {
        val zAxis = pose.zAxis
        var fx = -zAxis[0].toDouble()
        var fz = -zAxis[2].toDouble()
        var len = hypot(fx, fz)
        if (len < 1e-3) {
            // 真下・真上近くを向いた場合はカメラ上辺方向 (+Y) の水平成分を使う
            val yAxis = pose.yAxis
            fx = yAxis[0].toDouble()
            fz = yAxis[2].toDouble()
            len = hypot(fx, fz)
        }
        if (len < 1e-4) {
            fx = 0.0
            fz = -1.0
        } else {
            fx /= len
            fz /= len
        }
        // 右手系ワールド (+Y が鉛直上): forward = (fx, fz) に対して right = (-fz, fx), left = (fz, -fx)
        return HorizBasis(
            fwdX = fx,
            fwdZ = fz,
            rightX = -fz,
            rightZ = fx,
            leftX = fz,
            leftZ = -fx
        )
    }

    /**
     * origin の水平方位を北 (+y / +π/2)、右向きを +x として、cam の位置と方位を
     * SPEC §1 の部屋座標へ射影する（縦持ち・横持ち・斜め傾けに不変）。
     */
    private fun relativePose(origin: Pose, cam: Pose): Triple<Double, Double, Double> {
        val base = horizontalBasis(origin)
        val cur = horizontalBasis(cam)
        val dX = (cam.tx() - origin.tx()).toDouble()
        val dZ = (cam.tz() - origin.tz()).toDouble()
        val x = dX * base.rightX + dZ * base.rightZ
        val y = dX * base.fwdX + dZ * base.fwdZ
        val fRoomX = cur.fwdX * base.rightX + cur.fwdZ * base.rightZ
        val fRoomY = cur.fwdX * base.fwdX + cur.fwdZ * base.fwdZ
        val heading = atan2(fRoomY, fRoomX)
        return Triple(x, y, heading)
    }

    // ------------------------------------------------------- Depth → 21本レイ（3D 逆射影）
    /**
     * 深度画像の各画素を camera.pose を用いて重力整合ワールド空間へ 3D 逆射影し、
     * 床・天井を高さ・俯角フィルタで除外した上で水平 ±30° の 21 本レイへ集計する。
     * 新しい深度画像でないフレームでは null を返し、配信しない。
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

            val intr = frame.camera.textureIntrinsics
            val texW = intr.imageDimensions[0].toDouble()
            val texH = intr.imageDimensions[1].toDouble()
            if (texW <= 0.0 || texH <= 0.0) return null
            val fx = intr.focalLength[0].toDouble() * w / texW
            val fy = intr.focalLength[1].toDouble() * h / texH
            val cx = intr.principalPoint[0].toDouble() * w / texW
            val cy = intr.principalPoint[1].toDouble() * h / texH
            if (fx <= 0.0 || fy <= 0.0) return null

            val camPose = frame.camera.pose
            val xAxis = camPose.xAxis
            val yAxis = camPose.yAxis
            val zAxis = camPose.zAxis
            val ax0 = xAxis[0].toDouble()
            val ax1 = xAxis[1].toDouble()
            val ax2 = xAxis[2].toDouble()
            val ay0 = yAxis[0].toDouble()
            val ay1 = yAxis[1].toDouble()
            val ay2 = yAxis[2].toDouble()
            val az0 = zAxis[0].toDouble()
            val az1 = zAxis[1].toDouble()
            val az2 = zAxis[2].toDouble()

            val basis = horizontalBasis(camPose)
            val bins = Array(RAY_COUNT) { ArrayList<Double>() }

            var v = 0
            while (v < h) {
                var u = 0
                while (u < w) {
                    val dIndex = v * depthRowStride + u * depthPixelStride
                    if (dIndex + 1 < depthBuffer.limit()) {
                        val cIndex = v * confRowStride + u * confPixelStride
                        val confidence =
                            if (cIndex < confBuffer.limit()) confBuffer.get(cIndex).toInt() and 0xFF else 0
                        if (confidence >= 100) {
                            val lo = depthBuffer.get(dIndex).toInt() and 0xFF
                            val hi = depthBuffer.get(dIndex + 1).toInt() and 0xFF
                            val mm = lo or (hi shl 8)
                            if (mm in 150..5000) {
                                val depthM = mm / 1000.0
                                val xCam = (u - cx) * depthM / fx
                                val yCam = (cy - v) * depthM / fy
                                val zCam = -depthM

                                val wx = ax0 * xCam + ay0 * yCam + az0 * zCam
                                val wy = ax1 * xCam + ay1 * yCam + az1 * zCam
                                val wz = ax2 * xCam + ay2 * yCam + az2 * zCam

                                val dForward = wx * basis.fwdX + wz * basis.fwdZ
                                if (dForward > 0.12) {
                                    val dLeft = wx * basis.leftX + wz * basis.leftZ
                                    val horizDist = hypot(dForward, dLeft)
                                    if (horizDist >= 0.20 &&
                                        horizDist <= 5.0 &&
                                        wy >= -0.10 &&
                                        wy <= 0.50
                                    ) {
                                        val elevSlope = wy / horizDist
                                        if (elevSlope >= -0.05 && elevSlope <= 0.35) {
                                            val angleDeg = Math.toDegrees(atan2(dLeft, dForward))
                                            if (angleDeg >= -31.5 && angleDeg <= 31.5) {
                                                val bin = ((angleDeg + 31.5) / 3.0)
                                                    .toInt()
                                                    .coerceIn(0, RAY_COUNT - 1)
                                                bins[bin].add(horizDist)
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    u += 3
                }
                v += 3
            }

            for (i in 0 until RAY_COUNT) {
                val samples = bins[i]
                if (samples.size >= 2) {
                    samples.sort()
                    rays[i] = samples[samples.size / 2]
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
            } catch (_: Exception) {
            }
            try {
                confidenceImage?.close()
            } catch (_: Exception) {
            }
        }
        return rays
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

    /** ARCore の外部 OES カメラ映像を Flutter Texture 用の Window Surface へ描画する。 */
    private inner class PreviewBlitter(
        private val display: EGLDisplay,
        private val eglConfig: EGLConfig,
        private val context: EGLContext,
        private val pbufferSurface: EGLSurface,
        private val cameraTextureId: Int,
        private val targetTexture: SurfaceTexture?
    ) {
        private var windowSurface: EGLSurface = EGL14.EGL_NO_SURFACE
        private var program = 0
        private var posLoc = -1
        private var texLoc = -1
        private var samplerLoc = -1

        private val quadCoordsBuffer: FloatBuffer = ByteBuffer
            .allocateDirect(8 * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply {
                put(floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f))
                position(0)
            }

        private val quadTexBuffer: FloatBuffer = ByteBuffer
            .allocateDirect(8 * 4)
            .order(ByteOrder.nativeOrder())
            .asFloatBuffer()
            .apply {
                put(floatArrayOf(0f, 0f, 1f, 0f, 0f, 1f, 1f, 1f))
                position(0)
            }

        fun init() {
            val st = targetTexture ?: return
            try {
                val attrs = intArrayOf(EGL14.EGL_NONE)
                windowSurface = EGL14.eglCreateWindowSurface(display, eglConfig, st, attrs, 0)
                if (windowSurface == EGL14.EGL_NO_SURFACE) return

                if (!EGL14.eglMakeCurrent(display, windowSurface, windowSurface, context)) {
                    release()
                    return
                }
                val vShader = compileShader(
                    GLES20.GL_VERTEX_SHADER,
                    """
                    attribute vec2 aPosition;
                    attribute vec2 aTexCoord;
                    varying vec2 vTexCoord;
                    void main() {
                      gl_Position = vec4(aPosition, 0.0, 1.0);
                      vTexCoord = aTexCoord;
                    }
                    """.trimIndent()
                )
                val fShader = compileShader(
                    GLES20.GL_FRAGMENT_SHADER,
                    """
                    #extension GL_OES_EGL_image_external : require
                    precision mediump float;
                    varying vec2 vTexCoord;
                    uniform samplerExternalOES sTexture;
                    void main() {
                      gl_FragColor = texture2D(sTexture, vTexCoord);
                    }
                    """.trimIndent()
                )
                if (vShader == 0 || fShader == 0) {
                    if (vShader != 0) GLES20.glDeleteShader(vShader)
                    if (fShader != 0) GLES20.glDeleteShader(fShader)
                    release()
                    return
                }
                val prog = GLES20.glCreateProgram()
                GLES20.glAttachShader(prog, vShader)
                GLES20.glAttachShader(prog, fShader)
                GLES20.glLinkProgram(prog)
                GLES20.glDeleteShader(vShader)
                GLES20.glDeleteShader(fShader)
                val linkStatus = IntArray(1)
                GLES20.glGetProgramiv(prog, GLES20.GL_LINK_STATUS, linkStatus, 0)
                if (linkStatus[0] == 0) {
                    GLES20.glDeleteProgram(prog)
                    release()
                    return
                }
                program = prog
                posLoc = GLES20.glGetAttribLocation(prog, "aPosition")
                texLoc = GLES20.glGetAttribLocation(prog, "aTexCoord")
                samplerLoc = GLES20.glGetUniformLocation(prog, "sTexture")
            } catch (e: Exception) {
                Log.w(TAG, "PreviewBlitter init failed: ${e.message}")
                release()
            } finally {
                EGL14.eglMakeCurrent(display, pbufferSurface, pbufferSurface, context)
            }
        }

        fun drawFrame(frame: Frame) {
            if (windowSurface == EGL14.EGL_NO_SURFACE || program == 0) return
            try {
                quadCoordsBuffer.position(0)
                quadTexBuffer.position(0)
                frame.transformCoordinates2d(
                    Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,
                    quadCoordsBuffer,
                    Coordinates2d.TEXTURE_NORMALIZED,
                    quadTexBuffer
                )
                quadCoordsBuffer.position(0)
                quadTexBuffer.position(0)

                if (!EGL14.eglMakeCurrent(display, windowSurface, windowSurface, context)) {
                    return
                }
                GLES20.glViewport(0, 0, PREVIEW_WIDTH, PREVIEW_HEIGHT)
                GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT)
                GLES20.glUseProgram(program)

                GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
                GLES20.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, cameraTextureId)
                GLES20.glUniform1i(samplerLoc, 0)

                GLES20.glEnableVertexAttribArray(posLoc)
                GLES20.glVertexAttribPointer(posLoc, 2, GLES20.GL_FLOAT, false, 0, quadCoordsBuffer)
                GLES20.glEnableVertexAttribArray(texLoc)
                GLES20.glVertexAttribPointer(texLoc, 2, GLES20.GL_FLOAT, false, 0, quadTexBuffer)

                GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)

                GLES20.glDisableVertexAttribArray(posLoc)
                GLES20.glDisableVertexAttribArray(texLoc)
                EGL14.eglSwapBuffers(display, windowSurface)
            } catch (e: Exception) {
                Log.w(TAG, "PreviewBlitter draw failed: ${e.message}")
            } finally {
                EGL14.eglMakeCurrent(display, pbufferSurface, pbufferSurface, context)
                GLES20.glViewport(0, 0, 1, 1)
            }
        }

        fun release() {
            if (program != 0) {
                try {
                    GLES20.glDeleteProgram(program)
                } catch (_: Exception) {
                }
                program = 0
            }
            if (windowSurface != EGL14.EGL_NO_SURFACE) {
                try {
                    EGL14.eglDestroySurface(display, windowSurface)
                } catch (_: Exception) {
                }
                windowSurface = EGL14.EGL_NO_SURFACE
            }
        }

        private fun compileShader(type: Int, src: String): Int {
            val shader = GLES20.glCreateShader(type)
            if (shader == 0) return 0
            GLES20.glShaderSource(shader, src)
            GLES20.glCompileShader(shader)
            val status = IntArray(1)
            GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, status, 0)
            if (status[0] == 0) {
                GLES20.glDeleteShader(shader)
                return 0
            }
            return shader
        }
    }
}
