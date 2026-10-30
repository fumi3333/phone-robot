package dev.fumiproject.phone_robot_app

import android.view.WindowManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel

/**
 * プラットフォームチャネルの配線だけを行う薄い Activity。
 * 実処理は ArCoreManager（自己位置＋Depth）と SerialManager（ESP32）にある。
 *
 * チャネル一覧:
 *  - MethodChannel "phone_robot/arcore"  : startPoseTracking / stopPoseTracking
 *  - EventChannel  "phone_robot/arcore/pose"     : {x, y, heading}
 *  - EventChannel  "phone_robot/arcore/tracking" : 状態文字列
 *  - EventChannel  "phone_robot/depth"           : {ranges: [21本]}
 *  - MethodChannel "phone_robot/serial"  : listDevices / connect / disconnect / send
 *  - EventChannel  "phone_robot/serial/state"    : リンク状態文字列
 *  - EventChannel  "phone_robot/serial/rx"       : 受信テキスト
 *  - MethodChannel "phone_robot/system"  : setKeepScreenOn（実機モード中の画面常時点灯、R4）
 */
class MainActivity : FlutterActivity() {

    private var arCore: ArCoreManager? = null
    private var serial: SerialManager? = null

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        val messenger = flutterEngine.dartExecutor.binaryMessenger

        val ar = ArCoreManager(this)
        arCore = ar
        val se = SerialManager(this)
        serial = se

        MethodChannel(messenger, "phone_robot/arcore").setMethodCallHandler { call, result ->
            when (call.method) {
                "startPoseTracking" -> ar.start(result)
                "stopPoseTracking" -> {
                    ar.stop()
                    result.success(null)
                }
                else -> result.notImplemented()
            }
        }
        EventChannel(messenger, "phone_robot/arcore/pose").setStreamHandler(ar.poseHandler)
        EventChannel(messenger, "phone_robot/arcore/tracking")
            .setStreamHandler(ar.trackingHandler)
        EventChannel(messenger, "phone_robot/depth").setStreamHandler(ar.depthHandler)

        MethodChannel(messenger, "phone_robot/serial").setMethodCallHandler { call, result ->
            when (call.method) {
                "listDevices" -> result.success(se.listDevices())
                "connect" -> se.connect(
                    call.argument<Int>("deviceId"),
                    call.argument<Int>("baudRate") ?: 115200,
                    result
                )
                "disconnect" -> {
                    se.disconnect()
                    result.success(null)
                }
                "send" -> result.success(se.send(call.argument<String>("text") ?: ""))
                else -> result.notImplemented()
            }
        }
        EventChannel(messenger, "phone_robot/serial/state").setStreamHandler(se.stateHandler)
        EventChannel(messenger, "phone_robot/serial/rx").setStreamHandler(se.rxHandler)

        // 実機モード中の画面常時点灯（R4）。消灯が One UI のプロセス
        // 凍結の引き金になるため、Dart 側の KeepAwake から切り替える。
        MethodChannel(messenger, "phone_robot/system").setMethodCallHandler { call, result ->
            when (call.method) {
                "setKeepScreenOn" -> {
                    val on = call.argument<Boolean>("on") ?: false
                    runOnUiThread {
                        if (on) {
                            window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        } else {
                            window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
                        }
                    }
                    result.success(null)
                }
                else -> result.notImplemented()
            }
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        arCore?.onRequestPermissionsResult(requestCode, grantResults)
    }

    override fun onDestroy() {
        arCore?.stop()
        serial?.close()
        super.onDestroy()
    }
}
