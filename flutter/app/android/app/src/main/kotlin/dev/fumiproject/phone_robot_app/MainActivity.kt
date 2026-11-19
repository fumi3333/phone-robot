package dev.fumiproject.phone_robot_app

import android.content.Context
import android.content.Intent
import android.net.Uri
import android.view.WindowManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel
import org.json.JSONObject
import java.io.File
import java.io.FileOutputStream
import java.net.HttpURLConnection
import java.net.URL

/**
 * プラットフォームチャネルの配線だけを行う薄い Activity。
 * 実処理は ArCoreManager（自己位置＋Depth）と SerialManager（ESP32）にある。
 *
 * チャネル一覧:
 *  - MethodChannel "phone_robot/arcore"  : startPoseTracking / stopPoseTracking / getCameraTextureId / startMp4Recording / stopMp4Recording
 *  - EventChannel  "phone_robot/arcore/pose"     : {x, y, heading}
 *  - EventChannel  "phone_robot/arcore/tracking" : 状態文字列
 *  - EventChannel  "phone_robot/depth"           : {ranges: [21本]}
 *  - MethodChannel "phone_robot/serial"  : listDevices / connect / disconnect / send
 *  - EventChannel  "phone_robot/serial/state"    : リンク状態文字列
 *  - EventChannel  "phone_robot/serial/rx"       : 受信テキスト
 *  - MethodChannel "phone_robot/system"  : setKeepScreenOn（実機モード中の画面常時点灯、R4）
 *  - MethodChannel "phone_robot/storage" : saveRunBundle / shareRunToDrive / getWebhookUrl / setWebhookUrl
 */
class MainActivity : FlutterActivity() {

    private var arCore: ArCoreManager? = null
    private var serial: SerialManager? = null
    private var logStorage: LogStorageManager? = null

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        val messenger = flutterEngine.dartExecutor.binaryMessenger

        val ar = ArCoreManager(this, flutterEngine.renderer)
        arCore = ar
        val se = SerialManager(this)
        serial = se
        val ls = LogStorageManager(this)
        logStorage = ls

        MethodChannel(messenger, "phone_robot/arcore").setMethodCallHandler { call, result ->
            when (call.method) {
                "startPoseTracking" -> ar.start(result)
                "stopPoseTracking" -> {
                    ar.stop()
                    result.success(null)
                }
                "getCameraTextureId" -> result.success(ar.getCameraTextureId())
                "startMp4Recording" -> result.success(
                    ar.startMp4Recording(call.argument<String>("runId") ?: "")
                )
                "stopMp4Recording" -> result.success(ar.stopMp4Recording())
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
                "getVersionName" -> {
                    val v = try {
                        packageManager.getPackageInfo(packageName, 0).versionName ?: "0.0.0"
                    } catch (_: Exception) {
                        "0.0.0"
                    }
                    result.success(v)
                }
                "getAutoStartSensors" -> {
                    val prefs = getSharedPreferences("phone_robot_prefs", Context.MODE_PRIVATE)
                    result.success(prefs.getBoolean("auto_start_sensors", false))
                }
                "setAutoStartSensors" -> {
                    val on = call.argument<Boolean>("on") ?: false
                    getSharedPreferences("phone_robot_prefs", Context.MODE_PRIVATE)
                        .edit()
                        .putBoolean("auto_start_sensors", on)
                        .apply()
                    result.success(true)
                }
                "checkUpdate" -> checkLatestRelease(result)
                "downloadAndInstallApk" -> {
                    val apkUrl = call.argument<String>("url") ?: ""
                    downloadAndInstallApk(apkUrl, result)
                }
                else -> result.notImplemented()
            }
        }

        MethodChannel(messenger, "phone_robot/storage").setMethodCallHandler { call, result ->
            when (call.method) {
                "saveRunBundle" -> ls.saveRunBundle(call, result)
                "shareRunToDrive" -> result.success(
                    ls.shareRunToDriveInternal(call.argument<String>("runId"))
                )
                "getWebhookUrl" -> result.success(ls.getWebhookUrl())
                "setWebhookUrl" -> {
                    ls.setWebhookUrl(call.argument<String>("url") ?: "")
                    result.success(true)
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

    private fun checkLatestRelease(result: MethodChannel.Result) {
        val currentVersion = try {
            packageManager.getPackageInfo(packageName, 0).versionName ?: "0.0.0"
        } catch (_: Exception) {
            "0.0.0"
        }
        Thread {
            var conn: HttpURLConnection? = null
            try {
                val url = URL("https://api.github.com/repos/fumi3333/phone-robot/releases/latest")
                conn = (url.openConnection() as HttpURLConnection).apply {
                    requestMethod = "GET"
                    connectTimeout = 10000
                    readTimeout = 10000
                    setRequestProperty("Accept", "application/vnd.github+json")
                    setRequestProperty("User-Agent", "phone-robot-app")
                }
                if (conn.responseCode !in 200..299) {
                    runOnUiThread { result.success(mapOf("ok" to false, "error" to "HTTP ${conn.responseCode}")) }
                    return@Thread
                }
                val body = conn.inputStream.bufferedReader().use { it.readText() }
                val json = JSONObject(body)
                val tag = json.optString("tag_name", "")
                val latestClean = tag.removePrefix("v")
                val assets = json.optJSONArray("assets")
                var apkUrl = ""
                if (assets != null) {
                    for (i in 0 until assets.length()) {
                        val a = assets.getJSONObject(i)
                        val name = a.optString("name", "")
                        if (name.endsWith(".apk")) {
                            apkUrl = a.optString("browser_download_url", "")
                            if (name.contains("v$latestClean")) break
                        }
                    }
                }
                val hasUpdate = latestClean.isNotEmpty() && compareVersions(latestClean, currentVersion) > 0
                runOnUiThread {
                    result.success(
                        mapOf(
                            "ok" to true,
                            "currentVersion" to currentVersion,
                            "latestVersion" to latestClean,
                            "tag" to tag,
                            "hasUpdate" to hasUpdate,
                            "apkUrl" to apkUrl,
                            "releaseUrl" to json.optString("html_url", "")
                        )
                    )
                }
            } catch (e: Exception) {
                runOnUiThread { result.success(mapOf("ok" to false, "error" to (e.message ?: "error"))) }
            } finally {
                conn?.disconnect()
            }
        }.start()
    }

    private fun downloadAndInstallApk(apkUrl: String, result: MethodChannel.Result) {
        if (apkUrl.isBlank()) {
            result.success(mapOf("ok" to false, "error" to "APK URL が空です"))
            return
        }
        Thread {
            var conn: HttpURLConnection? = null
            try {
                val runsRoot = getExternalFilesDir("runs")
                    ?: throw IllegalStateException("外部キャッシュにアクセスできません")
                val updateDir = File(runsRoot, "update")
                if (!updateDir.exists()) updateDir.mkdirs()
                val apkFile = File(updateDir, "update_latest.apk")

                var currentUrl = apkUrl
                for (redirect in 0..4) {
                    conn = (URL(currentUrl).openConnection() as HttpURLConnection).apply {
                        requestMethod = "GET"
                        connectTimeout = 15000
                        readTimeout = 60000
                        instanceFollowRedirects = false
                        setRequestProperty("User-Agent", "phone-robot-app")
                    }
                    val code = conn.responseCode
                    if (code in 300..399) {
                        val next = conn.getHeaderField("Location")
                        conn.disconnect()
                        if (next.isNullOrBlank()) break
                        currentUrl = next
                    } else {
                        break
                    }
                }
                val activeConn = conn ?: throw IllegalStateException("接続に失敗しました")
                if (activeConn.responseCode !in 200..299) {
                    runOnUiThread {
                        result.success(mapOf("ok" to false, "error" to "HTTP ${activeConn.responseCode}"))
                    }
                    return@Thread
                }
                activeConn.inputStream.use { input ->
                    FileOutputStream(apkFile).use { output ->
                        val buf = ByteArray(16384)
                        while (true) {
                            val n = input.read(buf)
                            if (n <= 0) break
                            output.write(buf, 0, n)
                        }
                    }
                }
                val contentUri = Uri.parse("content://$packageName.logprovider/runs/update/update_latest.apk")
                val installIntent = Intent(Intent.ACTION_VIEW).apply {
                    setDataAndType(contentUri, "application/vnd.android.package-archive")
                    addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                }
                runOnUiThread {
                    startActivity(installIntent)
                    result.success(mapOf("ok" to true))
                }
            } catch (e: Exception) {
                runOnUiThread {
                    result.success(mapOf("ok" to false, "error" to (e.message ?: "install error")))
                }
            } finally {
                conn?.disconnect()
            }
        }.start()
    }

    private fun compareVersions(a: String, b: String): Int {
        val pa = a.split(".").map { it.toIntOrNull() ?: 0 }
        val pb = b.split(".").map { it.toIntOrNull() ?: 0 }
        val maxLen = maxOf(pa.size, pb.size)
        for (i in 0 until maxLen) {
            val va = if (i < pa.size) pa[i] else 0
            val vb = if (i < pb.size) pb[i] else 0
            if (va != vb) return va.compareTo(vb)
        }
        return 0
    }
}
