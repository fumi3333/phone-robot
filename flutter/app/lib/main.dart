import 'package:flutter/material.dart';

import 'real_page.dart';

void main() {
  runApp(const PhoneRobotApp());
}

/// phone-robot マッピング＆ラジコンテストアプリ。
class PhoneRobotApp extends StatelessWidget {
  const PhoneRobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'phone-robot',
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

class HomePage extends StatelessWidget {
  const HomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return const RealPage();
  }
}
