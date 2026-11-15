package dev.fumiproject.phone_robot_app

import android.app.Activity
import android.content.ContentProvider
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.database.Cursor
import android.database.MatrixCursor
import android.net.Uri
import android.os.Build
import android.os.Environment
import android.os.ParcelFileDescriptor
import android.provider.MediaStore
import android.provider.OpenableColumns
import android.util.Base64
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.FileNotFoundException
import java.net.HttpURLConnection
import java.net.URL

/**
 * externalFilesDir("runs") 配下の走行ログファイルを content:// URI として
 * Google ドライブや他アプリへ安全に渡す ContentProvider（androidx 依存なし）。
 */
class LogFileProvider : ContentProvider() {

    override fun onCreate(): Boolean = true

    override fun query(
        uri: Uri,
        projection: Array<out String>?,
        selection: String?,
        selectionArgs: Array<out String>?,
        sortOrder: String?
    ): Cursor {
        val file = resolveSafeFile(uri)
        val cols = projection ?: arrayOf(OpenableColumns.DISPLAY_NAME, OpenableColumns.SIZE)
        val cursor = MatrixCursor(cols, 1)
        val row = Array<Any?>(cols.size) { i ->
            when (cols[i]) {
                OpenableColumns.DISPLAY_NAME -> file?.name ?: uri.lastPathSegment ?: "file"
                OpenableColumns.SIZE -> file?.length() ?: 0L
                else -> null
            }
        }
        cursor.addRow(row)
        return cursor
    }

    override fun getType(uri: Uri): String {
        val name = (uri.lastPathSegment ?: "").lowercase()
        return LogStorageManager.mimeTypeFor(name)
    }

    override fun openFile(uri: Uri, mode: String): ParcelFileDescriptor {
        val file = resolveSafeFile(uri)
            ?: throw FileNotFoundException("走行記録ファイルが見つかりません: $uri")
        return ParcelFileDescriptor.open(file, ParcelFileDescriptor.MODE_READ_ONLY)
    }

    override fun insert(uri: Uri, values: ContentValues?): Uri? {
        throw UnsupportedOperationException("読み取り専用プロバイダです")
    }

    override fun delete(uri: Uri, selection: String?, selectionArgs: Array<out String>?): Int {
        throw UnsupportedOperationException("読み取り専用プロバイダです")
    }

    override fun update(
        uri: Uri,
        values: ContentValues?,
        selection: String?,
        selectionArgs: Array<out String>?
    ): Int {
        throw UnsupportedOperationException("読み取り専用プロバイダです")
    }

    private fun resolveSafeFile(uri: Uri): File? {
        val ctx = context ?: return null
        val baseDir = ctx.getExternalFilesDir("runs") ?: return null
        val segments = uri.pathSegments
        if (segments.size < 3 || segments[0] != "runs") return null
        val runDir = segments[1]
        val fileName = segments[2]
        if (runDir.contains("..") || fileName.contains("..")) return null
        val target = File(File(baseDir, runDir), fileName)
        val baseCanonical = baseDir.canonicalPath
        val targetCanonical = target.canonicalPath
        if (!targetCanonical.startsWith(baseCanonical + File.separator)) {
            return null
        }
        return if (target.exists() && target.isFile) target else null
    }
}

/**
 * 走行記録（CSV・マップPNG・JSON・サマリMD）の端末保存、Google ドライブ共有、
 * および GAS Webhook によるバックグラウンド自動転送を管理する。
 */
class LogStorageManager(private val activity: Activity) {

    private val prefs by lazy {
        activity.getSharedPreferences("phone_robot_prefs", Context.MODE_PRIVATE)
    }

    fun getWebhookUrl(): String = prefs.getString(KEY_WEBHOOK_URL, "") ?: ""

    fun setWebhookUrl(url: String) {
        prefs.edit().putString(KEY_WEBHOOK_URL, url.trim()).apply()
    }

    fun saveRunBundle(call: MethodCall, result: MethodChannel.Result) {
        val rawRunId = call.argument<String>("runId")?.trim().orEmpty()
        val safeRunId = rawRunId.replace(Regex("[^A-Za-z0-9_-]"), "")
        if (safeRunId.isEmpty()) {
            result.error("INVALID_ARGS", "runId が空または不正です", null)
            return
        }
        val rawFiles = call.argument<Map<*, *>>("files")
        if (rawFiles == null || rawFiles.isEmpty()) {
            result.error("INVALID_ARGS", "保存対象ファイルがありません", null)
            return
        }
        val autoDriveSync = call.argument<Boolean>("autoDriveSync") ?: true
        val openDriveSheet = call.argument<Boolean>("openDriveSheet") ?: true

        val runsRoot = activity.getExternalFilesDir("runs")
        if (runsRoot == null) {
            result.error("IO_ERROR", "外部ストレージのアプリ領域を取得できません", null)
            return
        }
        val runDir = File(runsRoot, "run_$safeRunId")
        if (!runDir.exists()) {
            runDir.mkdirs()
        }

        val byteFiles = LinkedHashMap<String, ByteArray>()
        try {
            for ((k, v) in rawFiles) {
                val rawName = k?.toString()?.trim().orEmpty()
                val safeName = File(rawName).name
                if (safeName.isEmpty() || safeName.contains("..")) continue
                val bytes = when (v) {
                    is ByteArray -> v
                    is String -> v.toByteArray(Charsets.UTF_8)
                    else -> null
                } ?: continue
                File(runDir, safeName).writeBytes(bytes)
                byteFiles[safeName] = bytes
            }
        } catch (e: Exception) {
            result.error("IO_ERROR", "端末保存に失敗しました: ${e.message}", null)
            return
        }

        // Android 10+ 端末からファイルアプリで見やすい Download/PhoneRobot/ にも複製する。
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            for ((name, bytes) in byteFiles) {
                saveToMediaStoreDownloads(safeRunId, name, bytes)
            }
        }

        val webhookUrl = getWebhookUrl().trim()
        if (autoDriveSync && webhookUrl.startsWith("https://")) {
            Thread {
                val uploaded = postToWebhook(webhookUrl, safeRunId, byteFiles)
                activity.runOnUiThread {
                    result.success(
                        mapOf(
                            "savedDir" to runDir.absolutePath,
                            "driveAction" to "webhook_sent",
                            "ok" to uploaded
                        )
                    )
                }
            }.start()
            return
        }

        if (autoDriveSync && openDriveSheet) {
            val shared = shareRunToDriveInternal(safeRunId)
            result.success(
                mapOf(
                    "savedDir" to runDir.absolutePath,
                    "driveAction" to "intent_opened",
                    "ok" to shared
                )
            )
            return
        }

        result.success(
            mapOf(
                "savedDir" to runDir.absolutePath,
                "driveAction" to "local_only",
                "ok" to true
            )
        )
    }

    fun shareRunToDriveInternal(runId: String?): Boolean {
        val runsRoot = activity.getExternalFilesDir("runs") ?: return false
        val targetDir = if (!runId.isNullOrBlank()) {
            val safe = runId.trim().replace(Regex("[^A-Za-z0-9_-]"), "")
            val candidate = File(runsRoot, "run_$safe")
            if (candidate.exists() && candidate.isDirectory) candidate else null
        } else {
            runsRoot.listFiles()
                ?.filter { it.isDirectory && it.name.startsWith("run_") }
                ?.maxByOrNull { it.lastModified() }
        } ?: return false

        val files = targetDir.listFiles()?.filter { it.isFile }?.sortedBy { it.name }.orEmpty()
        if (files.isEmpty()) return false

        val authority = "${activity.packageName}.logprovider"
        val uris = ArrayList<Uri>(files.size)
        for (file in files) {
            uris.add(Uri.parse("content://$authority/runs/${targetDir.name}/${Uri.encode(file.name)}"))
        }

        val baseIntent = Intent(Intent.ACTION_SEND_MULTIPLE).apply {
            type = "*/*"
            putParcelableArrayListExtra(Intent.EXTRA_STREAM, uris)
            putExtra(Intent.EXTRA_SUBJECT, "phone-robot ${targetDir.name}")
            putExtra(Intent.EXTRA_TEXT, "phone-robot 走行記録 (${targetDir.name})")
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }

        val drivePkg = "com.google.android.apps.docs"
        val driveIntent = Intent(baseIntent).setPackage(drivePkg)
        val pm = activity.packageManager
        return try {
            if (driveIntent.resolveActivity(pm) != null) {
                for (u in uris) {
                    activity.grantUriPermission(drivePkg, u, Intent.FLAG_GRANT_READ_URI_PERMISSION)
                }
                activity.startActivity(driveIntent)
            } else {
                activity.startActivity(
                    Intent.createChooser(baseIntent, "走行記録を Google ドライブへ保存")
                )
            }
            true
        } catch (_: Exception) {
            false
        }
    }

    private fun saveToMediaStoreDownloads(runId: String, fileName: String, bytes: ByteArray) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) return
        try {
            val resolver = activity.contentResolver
            val relPath = "${Environment.DIRECTORY_DOWNLOADS}/PhoneRobot/run_$runId"
            val values = ContentValues().apply {
                put(MediaStore.MediaColumns.DISPLAY_NAME, fileName)
                put(MediaStore.MediaColumns.MIME_TYPE, mimeTypeFor(fileName))
                put(MediaStore.MediaColumns.RELATIVE_PATH, relPath)
            }
            val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values) ?: return
            resolver.openOutputStream(uri)?.use { out ->
                out.write(bytes)
            }
        } catch (_: Exception) {
            // Download 領域への複製失敗時でもアプリ領域への保存は生かす。
        }
    }

    private fun postToWebhook(
        webhookUrl: String,
        runId: String,
        files: Map<String, ByteArray>
    ): Boolean {
        val fileArray = JSONArray()
        for ((name, bytes) in files) {
            fileArray.put(
                JSONObject().apply {
                    put("name", name)
                    put("mimeType", mimeTypeFor(name))
                    put("base64", Base64.encodeToString(bytes, Base64.NO_WRAP))
                }
            )
        }
        val payload = JSONObject().apply {
            put("runId", runId)
            put("files", fileArray)
        }.toString().toByteArray(Charsets.UTF_8)

        var conn: HttpURLConnection? = null
        return try {
            val url = URL(webhookUrl)
            conn = (url.openConnection() as HttpURLConnection).apply {
                requestMethod = "POST"
                connectTimeout = 15000
                readTimeout = 30000
                doOutput = true
                instanceFollowRedirects = false
                setRequestProperty("Content-Type", "application/json; charset=UTF-8")
            }
            conn.outputStream.use { it.write(payload) }
            val code = conn.responseCode
            // GAS WebApp は doPost 実行後に 302 で結果 URL へ転送するため 200..399 を成功とする。
            if (code in 300..399) {
                val redirectUrl = conn.getHeaderField("Location")
                if (!redirectUrl.isNullOrBlank()) {
                    followRedirectGet(redirectUrl)
                } else {
                    true
                }
            } else {
                code in 200..299
            }
        } catch (_: Exception) {
            false
        } finally {
            conn?.disconnect()
        }
    }

    private fun followRedirectGet(redirectUrl: String): Boolean {
        var conn: HttpURLConnection? = null
        return try {
            conn = (URL(redirectUrl).openConnection() as HttpURLConnection).apply {
                requestMethod = "GET"
                connectTimeout = 15000
                readTimeout = 20000
            }
            conn.responseCode in 200..399
        } catch (_: Exception) {
            // 最初の POST 時点ですでに doPost が実行されているためフォールバックは true を返さない。
            false
        } finally {
            conn?.disconnect()
        }
    }

    companion object {
        private const val KEY_WEBHOOK_URL = "gas_webhook_url"

        fun mimeTypeFor(fileName: String): String {
            val lower = fileName.lowercase()
            return when {
                lower.endsWith(".csv") -> "text/csv"
                lower.endsWith(".png") -> "image/png"
                lower.endsWith(".json") -> "application/json"
                lower.endsWith(".md") -> "text/markdown"
                else -> "application/octet-stream"
            }
        }
    }
}
