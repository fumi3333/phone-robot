import 'dart:async';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'map_painter.dart';
import 'platform/serial_link.dart';
import 'real/real_session.dart';

enum _WorkMode { walkScan, rcControl }

enum _RcInputType { buttons, flick }

/// 実機アプリ画面。
///
/// 問題を2つに分割し、
/// 1) 歩いてカメラ＋深度マップを測定・録画保存する「歩いてマップ測定」モード
/// 2) ESP32 とつないで前後左右・旋回の反応を記録テストする「ラジコン操縦テスト」モード
/// を上部のタブで切り替える。シミュレーションや詳細設定は「⚙ 設定」内に退避する。
class RealPage extends StatefulWidget {
  final RealSession? session;

  const RealPage({super.key, this.session});

  @override
  State<RealPage> createState() => _RealPageState();
}

class _RealPageState extends State<RealPage> with WidgetsBindingObserver {
  static const MethodChannel _systemChannel = MethodChannel('phone_robot/system');

  bool _floorScale = false;
  bool _followRobot = true;
  double _mapViewRadiusM = 2.5;
  Offset? _cameraCenterWorld;
  double _scaleStartRadiusM = 2.5;
  Offset? _lastFocalPoint;
  bool _cameraMain = false;
  bool _autoStartSensors = false;
  String _appVersion = '0.9.0';
  bool _checkingUpdate = false;
  String? _availableUpdateVersion;
  String? _availableApkUrl;
  bool _downloadingUpdate = false;
  _WorkMode _workMode = _WorkMode.walkScan;
  _RcInputType _rcInputType = _RcInputType.buttons;

  late RealSession _session;
  late bool _ownsSession;
  StreamSubscription<void>? _sub;
  Timer? _jogTimer;
  String? _activeFlickDir;
  Offset _flickOffset = Offset.zero;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    final injected = widget.session;
    if (injected != null) {
      _session = injected;
      _ownsSession = false;
      _sub = _session.updates.listen((_) {
        if (mounted) setState(() {});
      });
    } else {
      _ownsSession = true;
      _initSession(autoStart: false);
      unawaited(_loadAppPrefs());
    }
  }

  Future<void> _loadAppPrefs() async {
    try {
      final ver = await _systemChannel.invokeMethod<String>('getVersionName');
      final auto = await _systemChannel.invokeMethod<bool>('getAutoStartSensors');
      if (!mounted) return;
      setState(() {
        if (ver != null && ver.isNotEmpty) _appVersion = ver;
        _autoStartSensors = auto ?? false;
      });
      if (_autoStartSensors && !_session.sensorsOn) {
        unawaited(_session.autoSetup());
      }
      unawaited(_silentCheckUpdate());
    } catch (_) {
      // テスト環境等で未実装の場合は起動 OFF のまま
    }
  }

  Future<void> _silentCheckUpdate() async {
    try {
      final res = await _systemChannel.invokeMapMethod<String, dynamic>('checkUpdate');
      if (!mounted || res == null || res['ok'] != true) return;
      if (res['hasUpdate'] == true) {
        final latest = res['latestVersion']?.toString();
        final apkUrl = res['apkUrl']?.toString();
        if (latest != null && apkUrl != null && apkUrl.isNotEmpty) {
          setState(() {
            _availableUpdateVersion = latest;
            _availableApkUrl = apkUrl;
          });
        }
      }
    } catch (_) {}
  }

  Future<void> _installUpdateDirect(String apkUrl) async {
    if (_downloadingUpdate) return;
    setState(() => _downloadingUpdate = true);
    try {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('APK をダウンロード中… しばらくお待ちください')),
        );
      }
      final dl = await _systemChannel.invokeMapMethod<String, dynamic>(
        'downloadAndInstallApk',
        {'url': apkUrl},
      );
      if (!mounted) return;
      if (dl == null || dl['ok'] != true) {
        final err = dl?['error'] ?? 'ダウンロード失敗';
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('更新に失敗しました: $err')),
        );
      }
    } finally {
      if (mounted) setState(() => _downloadingUpdate = false);
    }
  }

  void _initSession({
    required bool autoStart,
    double? speedNorm,
    bool? invertSteer,
    bool? autoDriveSync,
  }) {
    final params = _floorScale
        ? RealSession.floorParams
        : RealSession.roomParams;
    _session = RealSession(customParams: params);
    if (speedNorm != null) _session.setSpeedNorm(speedNorm);
    if (invertSteer != null) _session.setInvertSteer(invertSteer);
    if (autoDriveSync != null) _session.setAutoDriveSync(autoDriveSync);
    _sub = _session.updates.listen((_) {
      if (mounted) setState(() {});
    });
    if (autoStart) {
      unawaited(_session.autoSetup());
    }
  }

  Future<void> _switchScale(bool floorMode) async {
    if (_session.busy || _floorScale == floorMode) return;
    final prevSpeed = _session.speedNorm;
    final prevInvert = _session.invertSteer;
    final prevAutoDrive = _session.autoDriveSync;
    _stopJog(sendStop: false);
    await _sub?.cancel();
    _sub = null;
    if (_ownsSession) {
      try {
        await _session.disconnectSerial();
        await _session.stopSensors();
      } catch (_) {}
      _session.dispose();
    }
    if (!mounted) return;
    setState(() {
      _floorScale = floorMode;
      _ownsSession = true;
      _initSession(
        autoStart: true,
        speedNorm: prevSpeed,
        invertSteer: prevInvert,
        autoDriveSync: prevAutoDrive,
      );
    });
  }

  @override
  void dispose() {
    _stopJog(sendStop: false);
    WidgetsBinding.instance.removeObserver(this);
    _sub?.cancel();
    if (_ownsSession) {
      _session.dispose();
    }
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    if (state == AppLifecycleState.paused ||
        state == AppLifecycleState.hidden) {
      _stopJog(sendStop: _session.linkState == Esp32LinkState.connected);
      _session.handleAppPaused();
    } else if (state == AppLifecycleState.resumed) {
      _session.handleAppResumed();
    }
  }

  void _onTapMap(TapDownDetails details, Size size) {
    if (!_session.explored || _session.busy) return;
    final cell = MapPainter.cellAtTap(
      tap: details.localPosition,
      size: size,
      params: _session.params,
      robotPose: _session.latestPose,
      followRobot: _followRobot,
      viewRadiusM: _mapViewRadiusM,
      cameraCenterWorld: _cameraCenterWorld,
    );
    if (cell != null) {
      _session.selectGoal(cell);
    }
  }

  Future<void> _checkAndApplyUpdate() async {
    if (_checkingUpdate) return;
    setState(() => _checkingUpdate = true);
    try {
      final res = await _systemChannel.invokeMapMethod<String, dynamic>('checkUpdate');
      if (!mounted) return;
      if (res == null || res['ok'] != true) {
        final err = res?['error'] ?? '通信エラー';
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('更新確認に失敗しました ($err)')),
        );
        return;
      }
      final hasUpdate = res['hasUpdate'] == true;
      final latest = res['latestVersion']?.toString() ?? '';
      final apkUrl = res['apkUrl']?.toString() ?? '';
      if (!hasUpdate || apkUrl.isEmpty) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('最新バージョンです (v$_appVersion)')),
        );
        return;
      }
      final ok = await showDialog<bool>(
        context: context,
        builder: (ctx) => AlertDialog(
          title: Text('アップデート v$latest'),
          content: Text(
            '新しいバージョン (v$latest) が公開されています。\n現在: v$_appVersion\n\nこのままダウンロードしてインストール画面を開きますか？',
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(ctx).pop(false),
              child: const Text('あとで'),
            ),
            FilledButton(
              onPressed: () => Navigator.of(ctx).pop(true),
              child: const Text('更新する'),
            ),
          ],
        ),
      );
      if (ok != true || !mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('APK をダウンロード中… しばらくお待ちください')),
      );
      final dl = await _systemChannel.invokeMapMethod<String, dynamic>(
        'downloadAndInstallApk',
        {'url': apkUrl},
      );
      if (!mounted) return;
      if (dl == null || dl['ok'] != true) {
        final err = dl?['error'] ?? 'ダウンロード失敗';
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('更新に失敗しました: $err')),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('更新エラー: $e')),
        );
      }
    } finally {
      if (mounted) setState(() => _checkingUpdate = false);
    }
  }

  void _startJog(String ch) {
    if (ch != 's' && _activeFlickDir == ch && _jogTimer != null) return;
    _jogTimer?.cancel();
    _activeFlickDir = ch == 's' ? null : ch;
    unawaited(_session.jog(ch));
    if (ch == 's') return;
    _jogTimer = Timer.periodic(
      const Duration(milliseconds: 100),
      (_) => unawaited(_session.jog(ch)),
    );
  }

  void _stopJog({bool sendStop = true}) {
    final hadTimer = _jogTimer != null || _activeFlickDir != null;
    _jogTimer?.cancel();
    _jogTimer = null;
    _activeFlickDir = null;
    if (sendStop && hadTimer) {
      unawaited(_session.jog('s'));
    }
  }

  Future<void> _editWebhookUrl() async {
    final controller = TextEditingController(text: _session.driveWebhookUrl);
    final result = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('自動ドライブ転送 URL', style: TextStyle(fontSize: 16)),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'scripts/google_drive_webhook.gs の URL を設定すると、'
              '記録停止時に自動保存します。空欄時は送信ボタンを押したときだけ共有画面を開きます。',
              style: TextStyle(fontSize: 12, color: Color(0xFF4B5563)),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: controller,
              decoration: const InputDecoration(
                hintText: 'https://script.google.com/macros/s/.../exec',
                border: OutlineInputBorder(),
                isDense: true,
              ),
              style: const TextStyle(fontSize: 12),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(''),
            child: const Text('クリア'),
          ),
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(),
            child: const Text('キャンセル'),
          ),
          FilledButton(
            onPressed: () => Navigator.of(ctx).pop(controller.text),
            child: const Text('保存'),
          ),
        ],
      ),
    );
    controller.dispose();
    if (result != null && mounted) {
      await _session.updateDriveWebhookUrl(result);
    }
  }

  Future<void> _openSettingsSheet() async {
    await showModalBottomSheet<void>(
      context: context,
      isScrollControlled: true,
      builder: (ctx) {
        return StatefulBuilder(
          builder: (ctx, setModalState) {
            void refresh() {
              setModalState(() {});
              if (mounted) setState(() {});
            }

            return SafeArea(
              child: SingleChildScrollView(
                child: Padding(
                  padding: const EdgeInsets.fromLTRB(16, 14, 16, 20),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          const Text(
                            '設定・テスト機能',
                            style: TextStyle(
                              fontSize: 16,
                              fontWeight: FontWeight.w700,
                            ),
                          ),
                          IconButton(
                            onPressed: () => Navigator.of(ctx).pop(),
                            icon: const Icon(Icons.close),
                          ),
                        ],
                      ),
                      const Divider(height: 12),
                      FilledButton(
                        key: const Key('check_update_btn'),
                        onPressed: _checkingUpdate
                            ? null
                            : () {
                                Navigator.of(ctx).pop();
                                unawaited(_checkAndApplyUpdate());
                              },
                        child: Text('アプリ更新 (現在 v$_appVersion)'),
                      ),
                      const SizedBox(height: 6),
                      SwitchListTile(
                        dense: true,
                        title: const Text('アプリ起動時にセンサーを自動開始'),
                        subtitle: const Text(
                          'OFF（推奨）にすると起動直後の電池消費を抑えます',
                        ),
                        value: _autoStartSensors,
                        onChanged: (v) async {
                          setState(() => _autoStartSensors = v);
                          try {
                            await _systemChannel.invokeMethod('setAutoStartSensors', {'on': v});
                          } catch (_) {}
                          refresh();
                        },
                      ),
                      SwitchListTile(
                        dense: true,
                        title: const Text('ARCore センサー'),
                        subtitle: Text(
                          '状態: ${_session.trackingState} (${_session.sensorHealth})',
                        ),
                        value: _session.sensorsOn,
                        onChanged: (v) async {
                          if (v) {
                            await _session.startSensors();
                          } else {
                            await _session.stopSensors();
                          }
                          refresh();
                        },
                      ),
                      SwitchListTile(
                        dense: true,
                        title: const Text('前方安全ブレーキ（45cm で自動停止）'),
                        subtitle: const Text(
                          '85cm 以内で自動減速、45cm 未満で前進を停止します（後退は可）',
                        ),
                        value: _session.frontBrakeEnabled,
                        onChanged: (v) {
                          _session.setFrontBrakeEnabled(v);
                          refresh();
                        },
                      ),
                      SwitchListTile(
                        dense: true,
                        title: const Text('MP4 動画も同時録画（電池消費大）'),
                        subtitle: const Text(
                          '通常は OFF 推奨。必要なときだけ ON にしてください',
                        ),
                        value: _session.recordMp4,
                        onChanged: (v) {
                          _session.setRecordMp4(v);
                          refresh();
                        },
                      ),
                      SwitchListTile(
                        dense: true,
                        title: const Text('操舵反転（左右モーター極性入れ替え）'),
                        value: _session.invertSteer,
                        onChanged: (v) {
                          _session.setInvertSteer(v);
                          refresh();
                        },
                      ),
                      SwitchListTile(
                        dense: true,
                        title: const Text('記録停止時の Webhook 自動転送'),
                        subtitle: const Text(
                          'Webhook URL 未設定時は自動でドライブ画面を開きません',
                        ),
                        value: _session.autoDriveSync,
                        onChanged: (v) {
                          _session.setAutoDriveSync(v);
                          refresh();
                        },
                      ),
                      const SizedBox(height: 6),
                      Row(
                        children: [
                          const Text('測定エリア:', style: TextStyle(fontSize: 13)),
                          const SizedBox(width: 10),
                          ChoiceChip(
                            label: const Text('部屋 6×5m'),
                            selected: !_floorScale,
                            onSelected: _session.busy
                                ? null
                                : (sel) async {
                                    if (sel) {
                                      Navigator.of(ctx).pop();
                                      await _switchScale(false);
                                    }
                                  },
                          ),
                          const SizedBox(width: 8),
                          ChoiceChip(
                            label: const Text('フロア 24×24m'),
                            selected: _floorScale,
                            onSelected: _session.busy
                                ? null
                                : (sel) async {
                                    if (sel) {
                                      Navigator.of(ctx).pop();
                                      await _switchScale(true);
                                    }
                                  },
                          ),
                        ],
                      ),
                      const SizedBox(height: 10),
                      FilledButton.tonal(
                        key: const Key('demo_preview_btn'),
                        onPressed: _session.busy
                            ? null
                            : () {
                                Navigator.of(ctx).pop();
                                _session.loadDemoPreviewMap();
                                setState(() {
                                  _cameraMain = false;
                                  _followRobot = true;
                                });
                              },
                        child: const Text('デモマップ表示'),
                      ),
                      const SizedBox(height: 8),
                      OutlinedButton.icon(
                        onPressed: () async {
                          Navigator.of(ctx).pop();
                          await _editWebhookUrl();
                        },
                        icon: const Icon(Icons.link, size: 18),
                        label: Text(
                          _session.driveWebhookUrl.isEmpty
                              ? 'Google ドライブ Webhook URL を設定'
                              : 'Webhook URL を編集（設定済み）',
                        ),
                      ),
                      const SizedBox(height: 10),
                      const Text(
                        '上級: 自律走行テスト（ラジコン動作確認後に使用）',
                        style: TextStyle(
                          fontSize: 12,
                          color: Color(0xFF6B7280),
                        ),
                      ),
                      const SizedBox(height: 6),
                      Row(
                        children: [
                          Expanded(
                            child: OutlinedButton(
                              onPressed: _session.busy
                                  ? null
                                  : () {
                                      Navigator.of(ctx).pop();
                                      unawaited(_session.runAutoCruise());
                                    },
                              child: const Text('VFH自律'),
                            ),
                          ),
                          const SizedBox(width: 6),
                          Expanded(
                            child: OutlinedButton(
                              onPressed: _session.busy
                                  ? null
                                  : () {
                                      Navigator.of(ctx).pop();
                                      unawaited(_session.runExploration());
                                    },
                              child: const Text('A*探索'),
                            ),
                          ),
                          const SizedBox(width: 6),
                          Expanded(
                            child: OutlinedButton(
                              onPressed: _session.path == null || _session.busy
                                  ? null
                                  : () {
                                      Navigator.of(ctx).pop();
                                      unawaited(_session.goToGoal());
                                    },
                              child: const Text('ゴール'),
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ),
            );
          },
        );
      },
    );
  }

  Widget _buildHeaderBar() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: const BoxDecoration(
        color: Colors.white,
        border: Border(bottom: BorderSide(color: Color(0xFFE5E7EB))),
      ),
      child: Row(
        children: [
          Expanded(
            child: SegmentedButton<_WorkMode>(
              showSelectedIcon: false,
              style: SegmentedButton.styleFrom(
                visualDensity: VisualDensity.compact,
                textStyle: const TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                ),
                selectedBackgroundColor: const Color(0xFF111827),
                selectedForegroundColor: Colors.white,
              ),
              segments: const [
                ButtonSegment(
                  value: _WorkMode.walkScan,
                  label: Text('マップ測定', key: Key('mode_walk')),
                ),
                ButtonSegment(
                  value: _WorkMode.rcControl,
                  label: Text('ラジコン', key: Key('mode_rc')),
                ),
              ],
              selected: {_workMode},
              onSelectionChanged: (sel) {
                if (sel.first != _WorkMode.rcControl) {
                  _stopJog(
                    sendStop: _session.linkState == Esp32LinkState.connected,
                  );
                }
                setState(() => _workMode = sel.first);
              },
            ),
          ),
          const SizedBox(width: 6),
          OutlinedButton(
            key: const Key('sensor_power_btn'),
            onPressed: _session.busy
                ? null
                : () async {
                    if (_session.sensorsOn) {
                      await _session.stopSensors();
                    } else {
                      await _session.startSensors();
                    }
                  },
            style: OutlinedButton.styleFrom(
              visualDensity: VisualDensity.compact,
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 8),
              minimumSize: const Size(0, 34),
              backgroundColor: _session.sensorsOn
                  ? const Color(0xFFECFDF5)
                  : const Color(0xFFF3F4F6),
              side: BorderSide(
                color: _session.sensorsOn
                    ? const Color(0xFF10B981)
                    : const Color(0xFFD1D5DB),
              ),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
            child: Text(
              _session.sensorsOn ? 'センサー ON' : 'センサー OFF',
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w700,
                color: _session.sensorsOn
                    ? const Color(0xFF065F46)
                    : const Color(0xFF4B5563),
              ),
            ),
          ),
          const SizedBox(width: 6),
          OutlinedButton(
            key: const Key('settings_btn'),
            onPressed: _openSettingsSheet,
            style: OutlinedButton.styleFrom(
              visualDensity: VisualDensity.compact,
              padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 8),
              minimumSize: const Size(0, 34),
              side: const BorderSide(color: Color(0xFFD1D5DB)),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
            child: const Text(
              '設定',
              style: TextStyle(
                fontSize: 12,
                fontWeight: FontWeight.w600,
                color: Color(0xFF111827),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildObstacleBanner() {
    final alert = _session.obstacleAlert;
    final bg = switch (alert.severity) {
      ObstacleSeverity.danger => const Color(0xFFDC2626),
      ObstacleSeverity.caution => const Color(0xFFD97706),
      ObstacleSeverity.clear => const Color(0xFF059669),
      ObstacleSeverity.unknown => const Color(0xFF475569),
    };
    String fmtM(double? v) => v == null ? '—' : '${v.toStringAsFixed(1)}m';
    final distBar =
        '左 ${fmtM(alert.leftClearM)}  |  正面 ${fmtM(alert.frontMinM)}  |  右 ${fmtM(alert.rightClearM)}';
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 7),
      color: bg,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        mainAxisSize: MainAxisSize.min,
        children: [
          Row(
            children: [
              Expanded(
                child: Text(
                  alert.message,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 13,
                    fontWeight: FontWeight.w700,
                  ),
                ),
              ),
              Text(
                '記録 ${_session.mappedAreaM2.toStringAsFixed(1)}m²',
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                ),
              ),
            ],
          ),
          const SizedBox(height: 2),
          Text(
            distBar,
            key: const Key('obstacle_dist_bar'),
            style: const TextStyle(
              color: Color(0xFFE2E8F0),
              fontSize: 11,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCameraWidget() {
    final texId = _session.cameraTextureId;
    if (!_session.sensorsOn || texId == null || texId < 0) {
      return Container(
        color: const Color(0xFF111827),
        alignment: Alignment.center,
        padding: const EdgeInsets.all(8),
        child: const Text(
          'カメラ待機中\n（実機センサー開始でライブ表示）',
          textAlign: TextAlign.center,
          style: TextStyle(color: Color(0xFF9CA3AF), fontSize: 12),
        ),
      );
    }
    return Container(
      color: Colors.black,
      alignment: Alignment.center,
      child: FittedBox(
        fit: BoxFit.contain,
        child: SizedBox(
          width: 640,
          height: 480,
          child: Stack(
            fit: StackFit.expand,
            children: [
              Texture(textureId: texId),
              CustomPaint(
                painter: _CarGuidePainter(
                  severity: _session.frontBrakeActive
                      ? ObstacleSeverity.danger
                      : _session.obstacleAlert.severity,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildMapWidget() {
    final params = _session.params;
    final robot =
        _session.robotCell ?? (r: params.gridH ~/ 2, c: params.gridW ~/ 2);
    return LayoutBuilder(
      builder: (context, constraints) {
        final size = Size(constraints.maxWidth, constraints.maxHeight);
        return GestureDetector(
          behavior: HitTestBehavior.opaque,
          onTapDown: (d) => _onTapMap(d, size),
          onScaleStart: (details) {
            _scaleStartRadiusM = _mapViewRadiusM;
            _lastFocalPoint = details.localFocalPoint;
          },
          onScaleUpdate: (details) {
            if (size.width <= 0 || size.height <= 0) return;
            final nextRadius = (_scaleStartRadiusM / details.scale).clamp(0.8, 10.0);
            final prevFocal = _lastFocalPoint ?? details.localFocalPoint;
            final delta = details.localFocalPoint - prevFocal;
            _lastFocalPoint = details.localFocalPoint;

            final pxPerM = math.min(size.width, size.height) / (2.0 * nextRadius);
            if (delta.distance > 1.0 || (details.scale - 1.0).abs() > 0.02) {
              setState(() {
                _mapViewRadiusM = nextRadius;
                if (delta.distance > 1.0) {
                  final rp = _session.latestPose;
                  final base = _cameraCenterWorld ??
                      (rp != null
                          ? Offset(rp.x, rp.y)
                          : Offset(params.roomW / 2.0, params.roomH / 2.0));
                  _followRobot = false;
                  _cameraCenterWorld = Offset(
                    (base.dx - delta.dx / pxPerM).clamp(0.0, params.roomW),
                    (base.dy + delta.dy / pxPerM).clamp(0.0, params.roomH),
                  );
                }
              });
            }
          },
          child: CustomPaint(
            size: size,
            painter: MapPainter(
              logOdds: _session.grid.logOdds,
              gridW: params.gridW,
              gridH: params.gridH,
              robot: robot,
              goal: _session.goal,
              path: _session.path,
              robotPose: _session.latestPose,
              latestScan: _session.latestScan,
              targetHeading: _session.hudTargetHeading,
              targetDistM: _session.hudTargetDistM,
              mappingParams: params,
              followRobot: _followRobot,
              viewRadiusM: _mapViewRadiusM,
              cameraCenterWorld: _cameraCenterWorld,
            ),
          ),
        );
      },
    );
  }

  Widget _buildStageArea() {
    final mainChild = _cameraMain ? _buildCameraWidget() : _buildMapWidget();
    final pipChild = _cameraMain ? _buildMapWidget() : _buildCameraWidget();
    final alert = _session.obstacleAlert;
    final isDanger =
        _session.frontBrakeActive || alert.severity == ObstacleSeverity.danger;
    final Color? signalColor = isDanger
        ? const Color(0xFFEF4444)
        : switch (alert.severity) {
            ObstacleSeverity.clear => const Color(0xFF10B981),
            ObstacleSeverity.caution => const Color(0xFFF59E0B),
            ObstacleSeverity.danger => const Color(0xFFEF4444),
            ObstacleSeverity.unknown => null,
          };
    final String? signalText = isDanger
        ? '停止 (壁)'
        : switch (alert.severity) {
            ObstacleSeverity.clear => '進行可',
            ObstacleSeverity.caution => '注意',
            ObstacleSeverity.danger => '停止 (壁)',
            ObstacleSeverity.unknown => null,
          };

    return Stack(
      fit: StackFit.expand,
      children: [
        mainChild,
        if (signalColor != null)
          IgnorePointer(
            child: Container(
              key: const Key('stage_signal_frame'),
              decoration: BoxDecoration(
                color: isDanger
                    ? signalColor.withAlpha(41)
                    : signalColor.withAlpha(15),
                border: Border.all(
                  color: signalColor,
                  width: isDanger ? 7.0 : 5.0,
                ),
              ),
            ),
          ),
        if (signalColor != null && signalText != null)
          Positioned(
            top: 8,
            right: 8,
            child: IgnorePointer(
              child: Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 10,
                  vertical: 5,
                ),
                decoration: BoxDecoration(
                  color: signalColor,
                  borderRadius: BorderRadius.circular(20),
                  boxShadow: const [
                    BoxShadow(color: Colors.black26, blurRadius: 4),
                  ],
                ),
                child: Text(
                  signalText,
                  style: const TextStyle(
                    color: Colors.white,
                    fontSize: 12,
                    fontWeight: FontWeight.w800,
                  ),
                ),
              ),
            ),
          ),
        Positioned(
          top: 8,
          left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
            decoration: BoxDecoration(
              color: const Color(0xBB111827),
              borderRadius: BorderRadius.circular(6),
            ),
            child: Text(
              _cameraMain
                  ? 'カメラ映像（ARCore ${_session.trackingState}）'
                  : (_followRobot
                        ? 'マップ（現在地追従 / 半径 ${_mapViewRadiusM.toStringAsFixed(1)}m）'
                        : 'マップ（フリー操作 / 半径 ${_mapViewRadiusM.toStringAsFixed(1)}m）'),
              style: const TextStyle(color: Colors.white, fontSize: 11),
            ),
          ),
        ),
        if (!_cameraMain)
          Positioned(
            left: 10,
            bottom: 10,
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: [
                FloatingActionButton.small(
                  heroTag: 'recenter_fab',
                  key: const Key('recenter_map_btn'),
                  backgroundColor: _followRobot ? const Color(0xFF1A73E8) : Colors.white,
                  foregroundColor: _followRobot ? Colors.white : const Color(0xFF1A73E8),
                  tooltip: '現在地へ戻る',
                  onPressed: () {
                    setState(() {
                      _followRobot = true;
                      _cameraCenterWorld = null;
                    });
                  },
                  child: const Icon(Icons.my_location, size: 20),
                ),
                const SizedBox(width: 6),
                FloatingActionButton.small(
                  heroTag: 'zoom_in_fab',
                  backgroundColor: Colors.white,
                  foregroundColor: const Color(0xFF111827),
                  tooltip: '拡大',
                  onPressed: () {
                    setState(() {
                      _mapViewRadiusM = (_mapViewRadiusM * 0.75).clamp(0.8, 10.0);
                    });
                  },
                  child: const Icon(Icons.add, size: 20),
                ),
                const SizedBox(width: 6),
                FloatingActionButton.small(
                  heroTag: 'zoom_out_fab',
                  backgroundColor: Colors.white,
                  foregroundColor: const Color(0xFF111827),
                  tooltip: '縮小',
                  onPressed: () {
                    setState(() {
                      _mapViewRadiusM = (_mapViewRadiusM / 0.75).clamp(0.8, 10.0);
                    });
                  },
                  child: const Icon(Icons.remove, size: 20),
                ),
              ],
            ),
          ),
        Positioned(
          right: 10,
          bottom: 10,
          child: GestureDetector(
            key: const Key('pip_swap'),
            onTap: () => setState(() => _cameraMain = !_cameraMain),
            child: Container(
              width: 128,
              height: 96,
              decoration: BoxDecoration(
                border: Border.all(color: Colors.white, width: 2),
                borderRadius: BorderRadius.circular(8),
                boxShadow: const [
                  BoxShadow(
                    color: Colors.black26,
                    blurRadius: 6,
                    offset: Offset(0, 2),
                  ),
                ],
              ),
              clipBehavior: Clip.antiAlias,
              child: Stack(
                fit: StackFit.expand,
                children: [
                  IgnorePointer(child: pipChild),
                  Positioned(
                    left: 4,
                    bottom: 4,
                    child: Container(
                      padding: const EdgeInsets.symmetric(
                        horizontal: 5,
                        vertical: 2,
                      ),
                      decoration: BoxDecoration(
                        color: const Color(0xBB000000),
                        borderRadius: BorderRadius.circular(4),
                      ),
                      child: Text(
                        _cameraMain ? '地図 (タップで拡大)' : 'カメラ (タップで拡大)',
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 9,
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildRecordActionRow() {
    final rec = _session.isRecording;
    return Row(
      children: [
        Expanded(
          flex: 3,
          child: FilledButton(
            key: const Key('record_toggle_btn'),
            style: FilledButton.styleFrom(
              backgroundColor: rec
                  ? const Color(0xFFDC2626)
                  : const Color(0xFF111827),
              foregroundColor: Colors.white,
              minimumSize: const Size(0, 44),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
            onPressed: _session.busy
                ? null
                : () => unawaited(_session.toggleManualRecording()),
            child: FittedBox(
              fit: BoxFit.scaleDown,
              child: Text(
                rec
                    ? '記録停止 (${_session.recordedRowCount}件)'
                    : '記録開始',
                style: const TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w700,
                ),
              ),
            ),
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          flex: 2,
          child: OutlinedButton(
            key: const Key('drive_send_btn'),
            style: OutlinedButton.styleFrom(
              foregroundColor: const Color(0xFF111827),
              side: const BorderSide(color: Color(0xFFD1D5DB)),
              minimumSize: const Size(0, 44),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(8),
              ),
            ),
            onPressed: _session.busy
                ? null
                : () => unawaited(_session.saveAndShareToDrive()),
            child: const FittedBox(
              fit: BoxFit.scaleDown,
              child: Text(
                'ドライブ送信',
                style: TextStyle(fontSize: 12, fontWeight: FontWeight.w600),
              ),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildWalkPanel({required EdgeInsetsGeometry padding}) {
    final pose = _session.latestPose;
    final posText = pose == null
        ? '位置未確定'
        : '移動 (${pose.x.toStringAsFixed(2)}m, ${pose.y.toStringAsFixed(2)}m) '
              '向 ${(pose.heading * 180 / math.pi).toStringAsFixed(0)}°';
    return Padding(
      padding: padding,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        mainAxisSize: MainAxisSize.min,
        children: [
          _buildRecordActionRow(),
          const SizedBox(height: 8),
          Row(
            children: [
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      '$posText  |  保存: ${_session.lastSaveStatus}',
                      style: const TextStyle(
                        fontSize: 12,
                        color: Color(0xFF374151),
                      ),
                      maxLines: 1,
                      overflow: TextOverflow.ellipsis,
                    ),
                    const SizedBox(height: 2),
                    Text(
                      _session.status,
                      style: const TextStyle(
                        fontSize: 12,
                        fontWeight: FontWeight.w600,
                        color: Color(0xFF111827),
                      ),
                      maxLines: 2,
                      overflow: TextOverflow.ellipsis,
                    ),
                  ],
                ),
              ),
              const SizedBox(width: 8),
              OutlinedButton(
                key: const Key('reset_map_btn'),
                style: OutlinedButton.styleFrom(
                  foregroundColor: const Color(0xFF374151),
                  side: const BorderSide(color: Color(0xFFD1D5DB)),
                  minimumSize: const Size(0, 38),
                  padding: const EdgeInsets.symmetric(horizontal: 10),
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(8),
                  ),
                ),
                onPressed: _session.isRecording || _session.busy
                    ? null
                    : _session.resetMap,
                child: const Text('消去', style: TextStyle(fontSize: 12)),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _jogButton(String label, String ch, {Color? bgColor, Color? fgColor}) {
    final connected = _session.linkState == Esp32LinkState.connected;
    final buttonStyle = OutlinedButton.styleFrom(
      backgroundColor: bgColor ?? Colors.white,
      foregroundColor: fgColor ?? const Color(0xFF111827),
      side: BorderSide(color: bgColor ?? const Color(0xFFD1D5DB)),
      padding: const EdgeInsets.symmetric(vertical: 10),
      minimumSize: const Size(0, 44),
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
      textStyle: const TextStyle(fontSize: 14, fontWeight: FontWeight.w700),
    );
    if (!connected) {
      return OutlinedButton(
        onPressed: null,
        style: buttonStyle,
        child: Text(label),
      );
    }
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onTapDown: (_) => _startJog(ch),
      onTapUp: (_) => _stopJog(sendStop: ch != 's'),
      onTapCancel: () => _stopJog(sendStop: ch != 's'),
      child: IgnorePointer(
        child: OutlinedButton(
          onPressed: () {},
          style: buttonStyle,
          child: Text(label),
        ),
      ),
    );
  }

  Widget _buildButtonPad() {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Row(
          children: [
            const Spacer(),
            Expanded(flex: 2, child: _jogButton('▲ 前', 'f')),
            const Spacer(),
          ],
        ),
        const SizedBox(height: 6),
        Row(
          children: [
            Expanded(child: _jogButton('◀ 左', 'l')),
            const SizedBox(width: 6),
            Expanded(
              child: _jogButton(
                '停止',
                's',
                bgColor: const Color(0xFFDC2626),
                fgColor: Colors.white,
              ),
            ),
            const SizedBox(width: 6),
            Expanded(child: _jogButton('右 ▶', 'r')),
          ],
        ),
        const SizedBox(height: 6),
        Row(
          children: [
            const Spacer(),
            Expanded(flex: 2, child: _jogButton('▼ 後', 'b')),
            const Spacer(),
          ],
        ),
      ],
    );
  }

  void _handleFlickDelta(Offset localPos, Size boxSize) {
    final center = Offset(boxSize.width / 2, boxSize.height / 2);
    final raw = localPos - center;
    final maxR = (math.min(boxSize.width, boxSize.height) / 2 - 18).clamp(
      24.0,
      64.0,
    );
    final dist = raw.distance;
    final clamped = dist > maxR ? raw * (maxR / dist) : raw;

    String? nextDir;
    if (dist >= 20) {
      if (raw.dy.abs() >= raw.dx.abs()) {
        nextDir = raw.dy < 0 ? 'f' : 'b';
      } else {
        nextDir = raw.dx < 0 ? 'l' : 'r';
      }
    }
    setState(() => _flickOffset = clamped);
    if (nextDir == null) {
      if (_activeFlickDir != null) {
        _stopJog(sendStop: true);
      }
    } else if (_activeFlickDir != nextDir) {
      _startJog(nextDir);
    }
  }

  Widget _buildFlickPad() {
    final connected = _session.linkState == Esp32LinkState.connected;
    final dirLabel = switch (_activeFlickDir) {
      'f' => '前進中',
      'b' => '後退中',
      'l' => '左旋回（先当て舵）',
      'r' => '右旋回（先当て舵）',
      _ => connected ? '上下左右にスライドして操縦（離すと停止）' : '先に ESP32 を接続してください',
    };
    return Row(
      children: [
        Expanded(
          child: SizedBox(
            height: 132,
            child: LayoutBuilder(
              builder: (ctx, constraints) {
                final size = Size(constraints.maxWidth, constraints.maxHeight);
                return GestureDetector(
                  key: const Key('flick_pad'),
                  behavior: HitTestBehavior.opaque,
                  onPanStart: !connected
                      ? null
                      : (d) => _handleFlickDelta(d.localPosition, size),
                  onPanUpdate: !connected
                      ? null
                      : (d) => _handleFlickDelta(d.localPosition, size),
                  onPanEnd: (_) {
                    _stopJog(sendStop: connected);
                    setState(() => _flickOffset = Offset.zero);
                  },
                  onPanCancel: () {
                    _stopJog(sendStop: connected);
                    setState(() => _flickOffset = Offset.zero);
                  },
                  child: Container(
                    decoration: BoxDecoration(
                      color: connected
                          ? const Color(0xFFF3F4F6)
                          : const Color(0xFFE5E7EB),
                      border: Border.all(
                        color: _activeFlickDir != null
                            ? const Color(0xFF111827)
                            : const Color(0xFFD1D5DB),
                        width: 1.5,
                      ),
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Stack(
                      alignment: Alignment.center,
                      children: [
                        const Positioned(
                          top: 6,
                          child: Text(
                            '前',
                            style: TextStyle(
                              fontSize: 11,
                              color: Color(0xFF6B7280),
                            ),
                          ),
                        ),
                        const Positioned(
                          bottom: 6,
                          child: Text(
                            '後',
                            style: TextStyle(
                              fontSize: 11,
                              color: Color(0xFF6B7280),
                            ),
                          ),
                        ),
                        const Positioned(
                          left: 10,
                          child: Text(
                            '左',
                            style: TextStyle(
                              fontSize: 11,
                              color: Color(0xFF6B7280),
                            ),
                          ),
                        ),
                        const Positioned(
                          right: 10,
                          child: Text(
                            '右',
                            style: TextStyle(
                              fontSize: 11,
                              color: Color(0xFF6B7280),
                            ),
                          ),
                        ),
                        Transform.translate(
                          offset: _flickOffset,
                          child: Container(
                            width: 44,
                            height: 44,
                            decoration: BoxDecoration(
                              color: connected
                                  ? const Color(0xFF111827)
                                  : const Color(0xFF9CA3AF),
                              shape: BoxShape.circle,
                            ),
                            alignment: Alignment.center,
                            child: Text(
                              _activeFlickDir?.toUpperCase() ?? '●',
                              style: const TextStyle(
                                color: Colors.white,
                                fontWeight: FontWeight.w700,
                                fontSize: 12,
                              ),
                            ),
                          ),
                        ),
                        Positioned(
                          right: 8,
                          bottom: 6,
                          child: Text(
                            dirLabel,
                            style: const TextStyle(
                              fontSize: 10,
                              color: Color(0xFF374151),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                );
              },
            ),
          ),
        ),
        const SizedBox(width: 8),
        SizedBox(
          width: 72,
          height: 132,
          child: FilledButton(
            style: FilledButton.styleFrom(
              backgroundColor: const Color(0xFFDC2626),
              foregroundColor: Colors.white,
              padding: EdgeInsets.zero,
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(10),
              ),
            ),
            onPressed: () {
              _stopJog(sendStop: false);
              unawaited(_session.emergencyStop());
            },
            child: const Text(
              '非常\n停止',
              textAlign: TextAlign.center,
              style: TextStyle(fontSize: 13, fontWeight: FontWeight.w700),
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildRcPanel({required EdgeInsetsGeometry padding}) {
    final connected = _session.linkState == Esp32LinkState.connected;
    final battV = _session.vehicleBatteryV;
    final battSuffix = battV == null
        ? ''
        : (battV < 7.0
              ? ' | 電池低下 ${battV.toStringAsFixed(1)}V'
              : ' | 電池 ${battV.toStringAsFixed(1)}V');
    final linkText = switch (_session.linkState) {
      Esp32LinkState.connected =>
        'ESP32 接続済み (${_session.link.lastCommand})$battSuffix',
      Esp32LinkState.connecting => 'ESP32 接続中…',
      Esp32LinkState.error => 'ESP32 接続エラー',
      Esp32LinkState.disconnected => 'ESP32 未接続',
    };

    return Padding(
      padding: padding,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
            decoration: BoxDecoration(
              color: connected
                  ? const Color(0xFFECFDF5)
                  : const Color(0xFFF3F4F6),
              border: Border.all(
                color: connected
                    ? const Color(0xFF10B981)
                    : const Color(0xFFD1D5DB),
              ),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Row(
              children: [
                Expanded(
                  child: Text(
                    linkText,
                    style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                      color: connected
                          ? const Color(0xFF065F46)
                          : const Color(0xFF374151),
                    ),
                    maxLines: 1,
                    overflow: TextOverflow.ellipsis,
                  ),
                ),
                const SizedBox(width: 8),
                OutlinedButton(
                  key: const Key('esp32_connect_btn'),
                  onPressed: connected
                      ? () => unawaited(_session.disconnectSerial())
                      : () => unawaited(_session.connectSerial()),
                  style: OutlinedButton.styleFrom(
                    visualDensity: VisualDensity.compact,
                    padding: const EdgeInsets.symmetric(horizontal: 10),
                    minimumSize: const Size(0, 30),
                    backgroundColor: connected
                        ? Colors.white
                        : const Color(0xFF111827),
                    foregroundColor: connected
                        ? const Color(0xFF111827)
                        : Colors.white,
                  ),
                  child: Text(
                    connected ? '切断' : '接続する',
                    style: const TextStyle(fontSize: 12),
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 6),
          _buildRecordActionRow(),
          const SizedBox(height: 4),
          Row(
            children: [
              Text(
                '速度 ${(_session.speedNorm * 100).round()}% '
                '(PWM ${_session.currentDrivePwm})',
                style: const TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: Color(0xFF111827),
                ),
              ),
              Expanded(
                child: SliderTheme(
                  data: SliderTheme.of(context).copyWith(
                    trackHeight: 3,
                    thumbShape: const RoundSliderThumbShape(
                      enabledThumbRadius: 7,
                    ),
                    overlayShape: const RoundSliderOverlayShape(
                      overlayRadius: 12,
                    ),
                  ),
                  child: Slider(
                    value: _session.speedNorm.clamp(0.10, 1.0),
                    min: 0.10,
                    max: 1.0,
                    divisions: 18,
                    activeColor: const Color(0xFF111827),
                    inactiveColor: const Color(0xFFD1D5DB),
                    onChanged: (v) => _session.setSpeedNorm(v),
                  ),
                ),
              ),
              SegmentedButton<_RcInputType>(
                showSelectedIcon: false,
                style: SegmentedButton.styleFrom(
                  visualDensity: VisualDensity.compact,
                  padding: const EdgeInsets.symmetric(horizontal: 8),
                  textStyle: const TextStyle(fontSize: 11),
                ),
                segments: const [
                  ButtonSegment(
                    value: _RcInputType.buttons,
                    label: Text('ボタン', key: Key('input_toggle_buttons')),
                  ),
                  ButtonSegment(
                    value: _RcInputType.flick,
                    label: Text('フリック', key: Key('input_toggle_flick')),
                  ),
                ],
                selected: {_rcInputType},
                onSelectionChanged: (s) {
                  _stopJog(sendStop: connected);
                  setState(() {
                    _rcInputType = s.first;
                    _flickOffset = Offset.zero;
                  });
                },
              ),
            ],
          ),
          const SizedBox(height: 4),
          Wrap(
            spacing: 6,
            runSpacing: 4,
            crossAxisAlignment: WrapCrossAlignment.center,
            children: [
              FilterChip(
                key: const Key('front_brake_chip'),
                visualDensity: VisualDensity.compact,
                selected: _session.frontBrakeEnabled,
                onSelected: (v) => _session.setFrontBrakeEnabled(v),
                label: Text(
                  _session.frontBrakeEnabled
                      ? '壁停止 ON (35cm)'
                      : '壁停止 OFF',
                  style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w600),
                ),
              ),
              OutlinedButton(
                key: const Key('auto_drive_btn'),
                style: OutlinedButton.styleFrom(
                  visualDensity: VisualDensity.compact,
                  padding: const EdgeInsets.symmetric(horizontal: 10),
                  minimumSize: const Size(0, 32),
                  backgroundColor: _session.busy
                      ? const Color(0xFFDC2626)
                      : const Color(0xFF111827),
                  foregroundColor: Colors.white,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(8),
                  ),
                ),
                onPressed: () async {
                  if (_session.busy) {
                    _stopJog(sendStop: false);
                    await _session.emergencyStop();
                    return;
                  }
                  if (!_session.sensorsOn) {
                    await _session.startSensors();
                  }
                  if (_session.linkState != Esp32LinkState.connected) {
                    await _session.connectSerial();
                  }
                  unawaited(_session.runAutoCruise());
                },
                child: Text(
                  _session.busy ? '自動停止' : '自動運転',
                  style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w700),
                ),
              ),
              if (_session.frontBrakeActive)
                const Text(
                  '壁の手前で停止中（後退可）',
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w700,
                    color: Color(0xFFDC2626),
                  ),
                ),
            ],
          ),
          const SizedBox(height: 4),
          _rcInputType == _RcInputType.buttons
              ? _buildButtonPad()
              : _buildFlickPad(),
        ],
      ),
    );
  }

  Widget _buildActiveControlPanel({required EdgeInsetsGeometry padding}) {
    return _workMode == _WorkMode.walkScan
        ? _buildWalkPanel(padding: padding)
        : _buildRcPanel(padding: padding);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Column(
          children: [
            _buildHeaderBar(),
            if (_availableUpdateVersion != null && _availableApkUrl != null)
              Container(
                width: double.infinity,
                padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 5),
                color: const Color(0xFF1E293B),
                child: Row(
                  children: [
                    Expanded(
                      child: Text(
                        '新版 v$_availableUpdateVersion あり',
                        style: const TextStyle(
                          color: Colors.white,
                          fontSize: 12,
                          fontWeight: FontWeight.w700,
                        ),
                      ),
                    ),
                    FilledButton(
                      style: FilledButton.styleFrom(
                        visualDensity: VisualDensity.compact,
                        padding: const EdgeInsets.symmetric(horizontal: 12),
                        minimumSize: const Size(0, 28),
                        backgroundColor: Colors.white,
                        foregroundColor: const Color(0xFF111827),
                      ),
                      onPressed: _downloadingUpdate
                          ? null
                          : () => unawaited(_installUpdateDirect(_availableApkUrl!)),
                      child: Text(
                        _downloadingUpdate ? '取得中...' : '更新',
                        style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w700),
                      ),
                    ),
                  ],
                ),
              ),
            _buildObstacleBanner(),
            Expanded(
              child: LayoutBuilder(
                builder: (context, constraints) {
                  final isLandscape =
                      constraints.maxWidth > constraints.maxHeight;
                  if (isLandscape) {
                    return Row(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        Expanded(child: _buildStageArea()),
                        const VerticalDivider(
                          width: 1,
                          color: Color(0xFFE5E7EB),
                        ),
                        SizedBox(
                          width: 348,
                          child: SingleChildScrollView(
                            child: _buildActiveControlPanel(
                              padding: const EdgeInsets.all(10),
                            ),
                          ),
                        ),
                      ],
                    );
                  }
                  return Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      Expanded(child: _buildStageArea()),
                      const Divider(height: 1, color: Color(0xFFE5E7EB)),
                      _buildActiveControlPanel(
                        padding: const EdgeInsets.fromLTRB(12, 8, 12, 10),
                      ),
                    ],
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

/// カメラ映像上にバックモニター風の車幅ガイドライン（幅24cm相当の進路枠）を描く。
class _CarGuidePainter extends CustomPainter {
  final ObstacleSeverity severity;

  const _CarGuidePainter({required this.severity});

  @override
  void paint(Canvas canvas, Size size) {
    final color = switch (severity) {
      ObstacleSeverity.clear => const Color(0xFF10B981),
      ObstacleSeverity.caution => const Color(0xFFF59E0B),
      ObstacleSeverity.danger => const Color(0xFFEF4444),
      ObstacleSeverity.unknown => const Color(0xFF94A3B8),
    };
    final linePaint = Paint()
      ..color = color.withAlpha(220)
      ..strokeWidth = 3.5
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    final w = size.width;
    final h = size.height;
    // 下端（手前）から画面中央やや下（奥）へ向かう車幅台形
    final leftBottom = Offset(w * 0.24, h * 0.96);
    final rightBottom = Offset(w * 0.76, h * 0.96);
    final leftMid = Offset(w * 0.34, h * 0.72);
    final rightMid = Offset(w * 0.66, h * 0.72);
    final leftTop = Offset(w * 0.41, h * 0.54);
    final rightTop = Offset(w * 0.59, h * 0.54);

    canvas.drawLine(leftBottom, leftTop, linePaint);
    canvas.drawLine(rightBottom, rightTop, linePaint);

    // 距離目安の横ライン（手前35cm停止線 / 奥65cm減速線）
    final stopPaint = Paint()
      ..color = const Color(0xFFEF4444).withAlpha(190)
      ..strokeWidth = 2.5;
    canvas.drawLine(leftMid, rightMid, stopPaint);

    final slowPaint = Paint()
      ..color = color.withAlpha(160)
      ..strokeWidth = 2.0;
    canvas.drawLine(leftTop, rightTop, slowPaint);
  }

  @override
  bool shouldRepaint(covariant _CarGuidePainter oldDelegate) =>
      oldDelegate.severity != severity;
}
