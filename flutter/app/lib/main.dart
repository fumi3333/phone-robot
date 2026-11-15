import 'dart:async';

import 'package:flutter/material.dart';

import 'platform/serial_link.dart';
import 'real_page.dart';
import 'sim_page.dart';

void main() {
  runApp(const PhoneRobotApp());
}

enum AppMode { simulation, realDevice }

/// phone-robot マッピングアプリ。
///
/// モード切替:
/// - シミュレーション: 部品なし・端末のセンサーなしでも、中核の探索と
///   ゴール指定の動きがそのまま見える（従来の骨格）。
/// - 実機: ARCore（自己位置＋深度）＋ ESP32（USB シリアル）で実車を
///   動かす。platform/ のチャネル越しにネイティブ実装とつながる。
class PhoneRobotApp extends StatelessWidget {
  const PhoneRobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'phone-robot mapping',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF111827),
          brightness: Brightness.light,
        ),
        scaffoldBackgroundColor: Colors.white,
        useMaterial3: true,
      ),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  AppMode _mode = AppMode.simulation;

  @override
  void initState() {
    super.initState();
    unawaited(_detectUsbAndSwitchMode());
  }

  Future<void> _detectUsbAndSwitchMode() async {
    try {
      final devices = await MethodChannelEsp32Link().listDevices();
      if (devices.isNotEmpty && mounted) {
        setState(() => _mode = AppMode.realDevice);
      }
    } catch (_) {
      // デスクトップ・テスト等チャネル未実装時はシミュレーションのまま
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Column(
        children: [
          SafeArea(
            bottom: false,
            child: Padding(
              padding: const EdgeInsets.fromLTRB(12, 8, 12, 4),
              child: SegmentedButton<AppMode>(
                showSelectedIcon: false,
                segments: const [
                  ButtonSegment(
                    value: AppMode.simulation,
                    label: Text('シミュレーション'),
                  ),
                  ButtonSegment(
                    value: AppMode.realDevice,
                    label: Text('実機'),
                  ),
                ],
                selected: {_mode},
                onSelectionChanged: (s) => setState(() => _mode = s.first),
              ),
            ),
          ),
          Expanded(
            child: _mode == AppMode.simulation
                ? const SimPage()
                : const RealPage(),
          ),
        ],
      ),
    );
  }
}
