// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:frontend/main.dart';

void main() {
  testWidgets('App renders main shell', (WidgetTester tester) async {
    await tester.pumpWidget(const RosInspectorApp());
    await tester.pumpAndSettle();

    expect(find.text('ROS2 Inspector'), findsOneWidget);
    expect(find.text('Tópicos'), findsOneWidget);
    expect(find.text('Nós'), findsOneWidget);
    expect(find.text('Serviços'), findsOneWidget);
  });
}
