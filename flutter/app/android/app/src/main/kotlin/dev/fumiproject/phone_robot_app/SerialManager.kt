package dev.fumiproject.phone_robot_app

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodChannel
import java.util.concurrent.Executors

/**
 * USB シリアル（usb-serial-for-android）で ESP32 とつなぐ管理クラス。
 *
 * チャネル契約（Dart: platform/serial_link.dart）:
 *  - MethodChannel "phone_robot/serial":
 *      listDevices -> [{deviceId, vendorId, productId, name}]
 *      connect {deviceId?, baudRate} -> bool
 *      disconnect -> null
 *      send {text} -> bool
 *  - EventChannel "phone_robot/serial/state": "connected" / "connecting" /
 *    "disconnected" / "error"
 *  - EventChannel "phone_robot/serial/rx": 受信テキスト（表示用）
 *
 * 対応チップはライブラリのデフォルトプローバ依存（CH340 / CP210x / FTDI /
 * CDC 等）。ESP32 基板（Lzxuan, CH340C）は VID 0x1A86。
 */
class SerialManager(private val context: Context) {

    companion object {
        private const val TAG = "SerialManager"
        private const val ACTION_USB_PERMISSION =
            "dev.fumiproject.phone_robot_app.USB_PERMISSION"
    }

    private val usbManager = context.getSystemService(Context.USB_SERVICE) as UsbManager

    @Volatile private var port: UsbSerialPort? = null
    @Volatile private var ioManager: SerialInputOutputManager? = null
    private var stateSink: EventChannel.EventSink? = null
    private var rxSink: EventChannel.EventSink? = null
    private val mainHandler = Handler(Looper.getMainLooper())

    private var pendingResult: MethodChannel.Result? = null
    private var pendingDevice: UsbDevice? = null
    private var pendingBaud: Int = 115200

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, intent: Intent) {
            if (intent.action != ACTION_USB_PERMISSION) return
            val granted =
                intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
            val result = pendingResult
            val device = pendingDevice
            pendingResult = null
            pendingDevice = null
            if (result == null) return
            if (granted && device != null) {
                result.success(openPort(device, pendingBaud))
            } else {
                pushState("error")
                result.success(false)
            }
        }
    }

    private val detachReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, intent: Intent) {
            if (intent.action == UsbManager.ACTION_USB_DEVICE_DETACHED) {
                disconnect()
            }
        }
    }

    init {
        ContextCompat.registerReceiver(
            context,
            permissionReceiver,
            IntentFilter(ACTION_USB_PERMISSION),
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
        ContextCompat.registerReceiver(
            context,
            detachReceiver,
            IntentFilter(UsbManager.ACTION_USB_DEVICE_DETACHED),
            ContextCompat.RECEIVER_NOT_EXPORTED
        )
    }

    val stateHandler = object : EventChannel.StreamHandler {
        override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
            stateSink = events
        }
        override fun onCancel(arguments: Any?) { stateSink = null }
    }

    val rxHandler = object : EventChannel.StreamHandler {
        override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
            rxSink = events
        }
        override fun onCancel(arguments: Any?) { rxSink = null }
    }

    fun listDevices(): List<Map<String, Any>> {
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        return drivers.map { driver ->
            mapOf(
                "deviceId" to driver.device.deviceId,
                "vendorId" to driver.device.vendorId,
                "productId" to driver.device.productId,
                "name" to (driver.device.productName ?: driver.device.deviceName ?: "USB serial")
            )
        }
    }

    fun connect(deviceId: Int?, baudRate: Int, result: MethodChannel.Result) {
        pushState("connecting")
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        val driver = drivers.firstOrNull { deviceId == null || it.device.deviceId == deviceId }
        if (driver == null) {
            pushState("error")
            result.success(false)
            return
        }
        val device = driver.device
        if (!usbManager.hasPermission(device)) {
            pendingResult = result
            pendingDevice = device
            pendingBaud = baudRate
            val intent = Intent(ACTION_USB_PERMISSION).apply {
                setPackage(context.packageName)
            }
            val flags =
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                    PendingIntent.FLAG_MUTABLE
                } else {
                    0
                }
            val pendingIntent = PendingIntent.getBroadcast(context, 0, intent, flags)
            usbManager.requestPermission(device, pendingIntent)
            return
        }
        result.success(openPort(device, baudRate))
    }

    private fun openPort(device: UsbDevice, baudRate: Int): Boolean {
        val drivers = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager)
        val driver = drivers.firstOrNull { it.device.deviceId == device.deviceId }
            ?: run {
                pushState("error")
                return false
            }
        val connection = usbManager.openDevice(device)
            ?: run {
                pushState("error")
                return false
            }
        val p = driver.ports[0]
        return try {
            p.open(connection)
            p.setParameters(baudRate, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            port = p
            val io = SerialInputOutputManager(p, object : SerialInputOutputManager.Listener {
                override fun onNewData(data: ByteArray) {
                    val text = String(data, Charsets.UTF_8)
                    mainHandler.post { rxSink?.success(text) }
                }

                override fun onRunError(e: Exception) {
                    Log.w(TAG, "serial io error: ${e.message}")
                    pushState("error")
                }
            })
            ioManager = io
            Executors.newSingleThreadExecutor().submit(io)
            pushState("connected")
            true
        } catch (e: Exception) {
            Log.w(TAG, "open failed: ${e.message}")
            pushState("error")
            false
        }
    }

    fun send(text: String): Boolean {
        val p = port ?: return false
        return try {
            p.write(text.toByteArray(Charsets.UTF_8), 500)
            true
        } catch (e: Exception) {
            Log.w(TAG, "write failed: ${e.message}")
            false
        }
    }

    fun disconnect() {
        try {
            ioManager?.stop()
        } catch (e: Exception) {
        }
        ioManager = null
        try {
            port?.close()
        } catch (e: Exception) {
        }
        port = null
        pushState("disconnected")
    }

    fun close() {
        disconnect()
        try {
            context.unregisterReceiver(permissionReceiver)
        } catch (e: Exception) {
        }
        try {
            context.unregisterReceiver(detachReceiver)
        } catch (e: Exception) {
        }
    }

    private fun pushState(state: String) {
        mainHandler.post { stateSink?.success(state) }
    }
}
