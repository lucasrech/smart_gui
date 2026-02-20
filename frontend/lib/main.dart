import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:image/image.dart' as img;
import 'package:web_socket_channel/web_socket_channel.dart';

/// Smart GUI Flutter web client.
///
/// Architecture overview:
/// - Left panel(s): entity discovery (topics/services).
/// - Right panel(s): live monitor and interaction tools.
/// - HTTP: graph/service/topic management.
/// - WebSocket: live topic stream.
void main() {
  runApp(const RosInspectorApp());
}

/// Root `MaterialApp` widget and app-wide theme definition.
class RosInspectorApp extends StatelessWidget {
  const RosInspectorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Smart GUI',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF1F6FEB)),
        useMaterial3: true,
      ),
      home: const HomePage(),
    );
  }
}

/// Main page with top-level tabs (Topics, Nodes, Services).
class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

/// Holds shared state for tabs, including backend base URL.
class _HomePageState extends State<HomePage> with TickerProviderStateMixin {
  late final String _backendUrl;
  late final TabController _tabController;

  @override
  void initState() {
    super.initState();
    _backendUrl = RosApi.defaultBaseUrl();
    _tabController = TabController(length: 3, vsync: this);
  }

  @override
  void dispose() {
    _tabController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('ROS2 Smart GUI'),
        bottom: TabBar(
          controller: _tabController,
          tabs: const [
            Tab(text: 'Topics'),
            Tab(text: 'Nodes'),
            Tab(text: 'Services'),
          ],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: [
          TopicsPage(backendUrl: _backendUrl),
          NodesPage(backendUrl: _backendUrl),
          ServicesPage(backendUrl: _backendUrl),
        ],
      ),
    );
  }
}

/// Topics workspace (list + message viewer/publisher split pane).
class TopicsPage extends StatefulWidget {
  const TopicsPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  State<TopicsPage> createState() => _TopicsPageState();
}

/// Tracks selected topic and renders the topic details panel.
class _TopicsPageState extends State<TopicsPage> {
  Map<String, dynamic>? _selectedTopic;

  @override
  Widget build(BuildContext context) {
    return ResponsiveSplitPane(
      left: _TopicsList(
        backendUrl: widget.backendUrl,
        onSelect: (topic) => setState(() => _selectedTopic = topic),
      ),
      right: _selectedTopic == null
          ? const Center(child: Text('Select a topic to view messages.'))
          : TopicMessagesPane(
              backendUrl: widget.backendUrl,
              topicName: _selectedTopic!['name'] as String,
              topicType:
                  ((_selectedTopic!['types'] as List<dynamic>?)?.isNotEmpty ??
                      false)
                  ? (_selectedTopic!['types'] as List<dynamic>).first
                        .toString()
                  : '',
            ),
    );
  }
}

/// Left-side topic list with filtering, refresh, favorites, and topic creation.
class _TopicsList extends StatefulWidget {
  const _TopicsList({required this.backendUrl, required this.onSelect});

  final String backendUrl;
  final ValueChanged<Map<String, dynamic>> onSelect;

  @override
  State<_TopicsList> createState() => _TopicsListState();
}

/// State for topic discovery list and related actions.
class _TopicsListState extends State<_TopicsList> {
  late Future<List<Map<String, dynamic>>> _future;
  final TextEditingController _searchController = TextEditingController();
  String _searchQuery = '';
  final Set<String> _favoriteTopics = <String>{};

  @override
  void initState() {
    super.initState();
    _future = RosApi(widget.backendUrl).getTopics();
  }

  @override
  void dispose() {
    _searchController.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(covariant _TopicsList oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.backendUrl != widget.backendUrl) {
      setState(() {
        _future = RosApi(widget.backendUrl).getTopics();
      });
    }
  }

  Future<void> _showCreateTopicDialog() async {
    final created = await showDialog<Map<String, dynamic>>(
      context: context,
      builder: (context) => _CreateTopicDialog(backendUrl: widget.backendUrl),
    );
    if (!mounted || created == null) {
      return;
    }
    setState(() {
      _future = RosApi(widget.backendUrl).getTopics();
    });
    widget.onSelect(created);
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(12, 12, 12, 6),
          child: Row(
            children: [
              Expanded(
                child: TextField(
                  controller: _searchController,
                  onChanged: (value) {
                    setState(() {
                      _searchQuery = value.trim().toLowerCase();
                    });
                  },
                  decoration: InputDecoration(
                    isDense: true,
                    hintText: 'Search topic...',
                    prefixIcon: const Icon(Icons.search),
                    suffixIcon: _searchQuery.isNotEmpty
                        ? IconButton(
                            tooltip: 'Clear search',
                            onPressed: () {
                              _searchController.clear();
                              setState(() {
                                _searchQuery = '';
                              });
                            },
                            icon: const Icon(Icons.close),
                          )
                        : null,
                    border: const OutlineInputBorder(),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              IconButton(
                tooltip: 'Refresh',
                onPressed: () {
                  setState(() {
                    _future = RosApi(widget.backendUrl).getTopics();
                  });
                },
                icon: const Icon(Icons.refresh),
              ),
              IconButton(
                tooltip: 'Create topic',
                onPressed: _showCreateTopicDialog,
                icon: const Icon(Icons.add_circle_outline),
              ),
            ],
          ),
        ),
        Expanded(
          child: FutureBuilder<List<Map<String, dynamic>>>(
            future: _future,
            builder: (context, snapshot) {
              if (snapshot.connectionState == ConnectionState.waiting) {
                return const Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasError) {
                return Center(child: Text('Error: ${snapshot.error}'));
              }
              final topics = snapshot.data ?? [];
              final filteredTopics = _searchQuery.isEmpty
                  ? topics
                  : topics.where((topic) {
                      final name = (topic['name'] as String? ?? '')
                          .toLowerCase();
                      return name.contains(_searchQuery);
                    }).toList();
              final indexedTopics = <String, int>{};
              for (var i = 0; i < topics.length; i++) {
                indexedTopics[topics[i]['name'] as String? ?? ''] = i;
              }
              filteredTopics.sort((a, b) {
                final aName = a['name'] as String? ?? '';
                final bName = b['name'] as String? ?? '';
                final aFav = _favoriteTopics.contains(aName);
                final bFav = _favoriteTopics.contains(bName);
                if (aFav != bFav) {
                  return aFav ? -1 : 1;
                }
                return (indexedTopics[aName] ?? 0).compareTo(
                  indexedTopics[bName] ?? 0,
                );
              });

              if (filteredTopics.isEmpty) {
                return const Center(
                  child: Text('No topics found for the current filter.'),
                );
              }

              return ListView.separated(
                itemCount: filteredTopics.length,
                separatorBuilder: (_, __) => const Divider(height: 1),
                itemBuilder: (context, index) {
                  final topic = filteredTopics[index];
                  final name = topic['name'] as String? ?? '';
                  final types = (topic['types'] as List<dynamic>? ?? []).join(
                    ', ',
                  );
                  return ListTile(
                    title: Text(name),
                    subtitle: Text(types),
                    onTap: () => widget.onSelect(topic),
                    trailing: IconButton(
                      tooltip: _favoriteTopics.contains(name)
                          ? 'Unfavorite'
                          : 'Favorite',
                      onPressed: () {
                        setState(() {
                          if (_favoriteTopics.contains(name)) {
                            _favoriteTopics.remove(name);
                          } else {
                            _favoriteTopics.add(name);
                          }
                        });
                      },
                      icon: Icon(
                        _favoriteTopics.contains(name)
                            ? Icons.star
                            : Icons.star_border,
                        color: _favoriteTopics.contains(name)
                            ? Theme.of(context).colorScheme.primary
                            : null,
                      ),
                    ),
                  );
                },
              );
            },
          ),
        ),
      ],
    );
  }
}

/// Nodes tab page.
class NodesPage extends StatelessWidget {
  const NodesPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  Widget build(BuildContext context) {
    return _SimpleListPage(
      title: 'Active nodes',
      fetcher: () => RosApi(backendUrl).getNodes(),
      itemBuilder: (item) => '${item['namespace']}${item['name']}',
    );
  }
}

/// Services tab page.
class ServicesPage extends StatelessWidget {
  const ServicesPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  Widget build(BuildContext context) {
    return _ServicesSplitPage(backendUrl: backendUrl);
  }
}

/// Split pane wrapper for service list (left) and service call panel (right).
class _ServicesSplitPage extends StatefulWidget {
  const _ServicesSplitPage({required this.backendUrl});

  final String backendUrl;

  @override
  State<_ServicesSplitPage> createState() => _ServicesSplitPageState();
}

/// Maintains selected service item.
class _ServicesSplitPageState extends State<_ServicesSplitPage> {
  Map<String, dynamic>? _selectedService;

  @override
  Widget build(BuildContext context) {
    return ResponsiveSplitPane(
      left: _ServicesList(
        backendUrl: widget.backendUrl,
        onSelect: (service) => setState(() => _selectedService = service),
      ),
      right: _selectedService == null
          ? const Center(
              child: Text('Select a service to view/call.'),
            )
          : _ServiceCallPane(
              backendUrl: widget.backendUrl,
              serviceName: _selectedService!['name'] as String,
              serviceType:
                  ((_selectedService!['types'] as List<dynamic>?)?.isNotEmpty ??
                      false)
                  ? (_selectedService!['types'] as List<dynamic>).first
                        .toString()
                  : '',
            ),
    );
  }
}

/// Left-side service list with filtering, refresh, and favorites.
class _ServicesList extends StatefulWidget {
  const _ServicesList({required this.backendUrl, required this.onSelect});

  final String backendUrl;
  final ValueChanged<Map<String, dynamic>> onSelect;

  @override
  State<_ServicesList> createState() => _ServicesListState();
}

/// State for service discovery list and related UI controls.
class _ServicesListState extends State<_ServicesList> {
  late Future<List<Map<String, dynamic>>> _future;
  final TextEditingController _searchController = TextEditingController();
  String _searchQuery = '';
  final Set<String> _favoriteServices = <String>{};

  @override
  void initState() {
    super.initState();
    _future = RosApi(widget.backendUrl).getServices();
  }

  @override
  void dispose() {
    _searchController.dispose();
    super.dispose();
  }

  @override
  void didUpdateWidget(covariant _ServicesList oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.backendUrl != widget.backendUrl) {
      setState(() => _future = RosApi(widget.backendUrl).getServices());
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.fromLTRB(12, 12, 12, 6),
          child: Row(
            children: [
              Expanded(
                child: TextField(
                  controller: _searchController,
                  onChanged: (value) {
                    setState(() {
                      _searchQuery = value.trim().toLowerCase();
                    });
                  },
                  decoration: InputDecoration(
                    isDense: true,
                    hintText: 'Search service...',
                    prefixIcon: const Icon(Icons.search),
                    suffixIcon: _searchQuery.isNotEmpty
                        ? IconButton(
                            tooltip: 'Clear search',
                            onPressed: () {
                              _searchController.clear();
                              setState(() {
                                _searchQuery = '';
                              });
                            },
                            icon: const Icon(Icons.close),
                          )
                        : null,
                    border: const OutlineInputBorder(),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              IconButton(
                tooltip: 'Refresh',
                onPressed: () {
                  setState(
                    () => _future = RosApi(widget.backendUrl).getServices(),
                  );
                },
                icon: const Icon(Icons.refresh),
              ),
            ],
          ),
        ),
        Expanded(
          child: FutureBuilder<List<Map<String, dynamic>>>(
            future: _future,
            builder: (context, snapshot) {
              if (snapshot.connectionState == ConnectionState.waiting) {
                return const Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasError) {
                return Center(child: Text('Error: ${snapshot.error}'));
              }
              final services = snapshot.data ?? [];
              final filteredServices = _searchQuery.isEmpty
                  ? services
                  : services.where((service) {
                      final name = (service['name'] as String? ?? '')
                          .toLowerCase();
                      return name.contains(_searchQuery);
                    }).toList();
              final indexedServices = <String, int>{};
              for (var i = 0; i < services.length; i++) {
                indexedServices[services[i]['name'] as String? ?? ''] = i;
              }
              filteredServices.sort((a, b) {
                final aName = a['name'] as String? ?? '';
                final bName = b['name'] as String? ?? '';
                final aFav = _favoriteServices.contains(aName);
                final bFav = _favoriteServices.contains(bName);
                if (aFav != bFav) {
                  return aFav ? -1 : 1;
                }
                return (indexedServices[aName] ?? 0).compareTo(
                  indexedServices[bName] ?? 0,
                );
              });

              if (filteredServices.isEmpty) {
                return const Center(
                  child: Text('No services found for the current filter.'),
                );
              }

              return ListView.separated(
                itemCount: filteredServices.length,
                separatorBuilder: (_, __) => const Divider(height: 1),
                itemBuilder: (context, index) {
                  final service = filteredServices[index];
                  final name = service['name'] as String? ?? '';
                  final type =
                      ((service['types'] as List<dynamic>?)?.isNotEmpty ??
                          false)
                      ? (service['types'] as List<dynamic>).first.toString()
                      : '';
                  return ListTile(
                    title: Text(name),
                    subtitle: Text(type),
                    onTap: () => widget.onSelect(service),
                    trailing: IconButton(
                      tooltip: _favoriteServices.contains(name)
                          ? 'Unfavorite'
                          : 'Favorite',
                      onPressed: () {
                        setState(() {
                          if (_favoriteServices.contains(name)) {
                            _favoriteServices.remove(name);
                          } else {
                            _favoriteServices.add(name);
                          }
                        });
                      },
                      icon: Icon(
                        _favoriteServices.contains(name)
                            ? Icons.star
                            : Icons.star_border,
                        color: _favoriteServices.contains(name)
                            ? Theme.of(context).colorScheme.primary
                            : null,
                      ),
                    ),
                  );
                },
              );
            },
          ),
        ),
      ],
    );
  }
}

/// Right-side service call panel with schema and JSON payload editor.
class _ServiceCallPane extends StatefulWidget {
  const _ServiceCallPane({
    required this.backendUrl,
    required this.serviceName,
    required this.serviceType,
  });

  final String backendUrl;
  final String serviceName;
  final String serviceType;

  @override
  State<_ServiceCallPane> createState() => _ServiceCallPaneState();
}

/// Handles schema loading and service call execution flow.
class _ServiceCallPaneState extends State<_ServiceCallPane> {
  final TextEditingController _payloadController = TextEditingController();
  bool _loadingSchema = false;
  bool _sending = false;
  String? _error;
  Map<String, dynamic>? _schema;
  String? _responseText;

  @override
  void initState() {
    super.initState();
    _loadSchema();
  }

  @override
  void didUpdateWidget(covariant _ServiceCallPane oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.serviceName != widget.serviceName ||
        oldWidget.serviceType != widget.serviceType ||
        oldWidget.backendUrl != widget.backendUrl) {
      _responseText = null;
      _error = null;
      _loadSchema();
    }
  }

  @override
  void dispose() {
    _payloadController.dispose();
    super.dispose();
  }

  Future<void> _loadSchema() async {
    if (widget.serviceType.isEmpty) {
      setState(() {
        _schema = null;
        _payloadController.text = '{}';
      });
      return;
    }
    setState(() => _loadingSchema = true);
    try {
      final schema = await RosApi(
        widget.backendUrl,
      ).getServiceSchema(widget.serviceName, widget.serviceType);
      const encoder = JsonEncoder.withIndent('  ');
      setState(() {
        _schema = schema;
        _payloadController.text = encoder.convert(schema['request_template']);
      });
    } catch (err) {
      setState(() => _error = err.toString());
    } finally {
      setState(() => _loadingSchema = false);
    }
  }

  Future<void> _sendServiceRequest() async {
    setState(() {
      _sending = true;
      _error = null;
    });
    try {
      final payloadObj = jsonDecode(_payloadController.text);
      if (payloadObj is! Map<String, dynamic>) {
        throw Exception('Payload must be a JSON object.');
      }
      final result = await RosApi(widget.backendUrl).callService(
        name: widget.serviceName,
        serviceType: widget.serviceType,
        request: payloadObj,
      );
      setState(() {
        _responseText = const JsonEncoder.withIndent('  ').convert(result);
      });
    } catch (err) {
      setState(() => _error = err.toString());
    } finally {
      setState(() => _sending = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final requestTemplate = _schema?['request_template'];
    final responseTemplate = _schema?['response_template'];
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        ListTile(
          title: Text(widget.serviceName),
          subtitle: Text(
            widget.serviceType.isEmpty ? 'Type unavailable' : widget.serviceType,
          ),
          trailing: IconButton(
            tooltip: 'Reload schema',
            onPressed: _loadingSchema ? null : _loadSchema,
            icon: const Icon(Icons.refresh),
          ),
        ),
        if (_loadingSchema) const LinearProgressIndicator(minHeight: 2),
        Expanded(
          child: ListView(
            padding: const EdgeInsets.all(12),
            children: [
              const Text('Expected payload (request template):'),
              const SizedBox(height: 6),
              Container(
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(
                  border: Border.all(color: Theme.of(context).dividerColor),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(
                  requestTemplate == null
                      ? '{}'
                      : const JsonEncoder.withIndent(
                          '  ',
                        ).convert(requestTemplate),
                  style: const TextStyle(fontFamily: 'monospace'),
                ),
              ),
              const SizedBox(height: 12),
              const Text('Payload to send (JSON):'),
              const SizedBox(height: 6),
              TextField(
                controller: _payloadController,
                maxLines: 10,
                decoration: const InputDecoration(
                  border: OutlineInputBorder(),
                  hintText: '{\n  ...\n}',
                ),
                style: const TextStyle(fontFamily: 'monospace'),
              ),
              const SizedBox(height: 10),
              FilledButton.icon(
                onPressed: (_sending || widget.serviceType.isEmpty)
                    ? null
                    : _sendServiceRequest,
                icon: const Icon(Icons.send),
                label: Text(_sending ? 'Sending...' : 'Call service'),
              ),
              if (_error != null) ...[
                const SizedBox(height: 12),
                Text(
                  'Error: $_error',
                  style: TextStyle(color: Theme.of(context).colorScheme.error),
                ),
              ],
              const SizedBox(height: 16),
              const Text('Expected response format (response template):'),
              const SizedBox(height: 6),
              Container(
                padding: const EdgeInsets.all(10),
                decoration: BoxDecoration(
                  border: Border.all(color: Theme.of(context).dividerColor),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: Text(
                  responseTemplate == null
                      ? '{}'
                      : const JsonEncoder.withIndent(
                          '  ',
                        ).convert(responseTemplate),
                  style: const TextStyle(fontFamily: 'monospace'),
                ),
              ),
              if (_responseText != null) ...[
                const SizedBox(height: 16),
                const Text('Received response:'),
                const SizedBox(height: 6),
                Container(
                  padding: const EdgeInsets.all(10),
                  decoration: BoxDecoration(
                    border: Border.all(color: Theme.of(context).dividerColor),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Text(
                    _responseText!,
                    style: const TextStyle(fontFamily: 'monospace'),
                  ),
                ),
              ],
            ],
          ),
        ),
      ],
    );
  }
}

/// Generic one-column list page used by simple tabs (e.g., Nodes).
class _SimpleListPage extends StatefulWidget {
  const _SimpleListPage({
    required this.title,
    required this.fetcher,
    required this.itemBuilder,
  });

  final String title;
  final Future<List<Map<String, dynamic>>> Function() fetcher;
  final String Function(Map<String, dynamic>) itemBuilder;

  @override
  State<_SimpleListPage> createState() => _SimpleListPageState();
}

/// State holder for generic list page.
class _SimpleListPageState extends State<_SimpleListPage> {
  late Future<List<Map<String, dynamic>>> _future;

  @override
  void initState() {
    super.initState();
    _future = widget.fetcher();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        ListTile(
          title: Text(widget.title),
          trailing: IconButton(
            tooltip: 'Refresh',
            onPressed: () => setState(() => _future = widget.fetcher()),
            icon: const Icon(Icons.refresh),
          ),
        ),
        Expanded(
          child: FutureBuilder<List<Map<String, dynamic>>>(
            future: _future,
            builder: (context, snapshot) {
              if (snapshot.connectionState == ConnectionState.waiting) {
                return const Center(child: CircularProgressIndicator());
              }
              if (snapshot.hasError) {
                return Center(child: Text('Error: ${snapshot.error}'));
              }
              final items = snapshot.data ?? [];
              return ListView.separated(
                itemCount: items.length,
                separatorBuilder: (_, __) => const Divider(height: 1),
                itemBuilder: (context, index) {
                  return ListTile(
                    title: Text(widget.itemBuilder(items[index])),
                  );
                },
              );
            },
          ),
        ),
      ],
    );
  }
}

/// Right-side topic panel: live monitor, single publish, and loop controls.
class TopicMessagesPane extends StatefulWidget {
  const TopicMessagesPane({
    super.key,
    required this.backendUrl,
    required this.topicName,
    required this.topicType,
  });

  final String backendUrl;
  final String topicName;
  final String topicType;

  @override
  State<TopicMessagesPane> createState() => _TopicMessagesPaneState();
}

/// Handles WebSocket stream, publish actions, and anti-self-block traffic logic.
class _TopicMessagesPaneState extends State<TopicMessagesPane> {
  // WebSocket channel used for live topic streaming.
  WebSocketChannel? _channel;
  // Ring-like in-memory list of most recent messages shown in the UI.
  final List<Map<String, dynamic>> _messages = [];
  // Connection and publish status flags.
  String? _error;
  bool _connecting = false;
  bool _imageViewEnabled = false;
  final GlobalKey<_PayloadFieldsFormState> _publishFormKey =
      GlobalKey<_PayloadFieldsFormState>();
  Map<String, dynamic>? _messageTemplate;
  bool _loadingTemplate = false;
  bool _publishing = false;
  String? _publishError;
  String? _publishInfo;
  final TextEditingController _loopFrequencyController = TextEditingController(
    text: '1.0',
  );
  bool _looping = false;
  bool _loopActionInProgress = false;
  Timer? _topicTrafficTimer;
  bool _topicBusyByExternalTraffic = false;
  DateTime? _selfTrafficIgnoreUntil;
  bool _ownLoopActive = false;
  DateTime? _lastRenderedTopicMessageAt;

  @override
  void initState() {
    super.initState();
    _connect();
    _loadTemplate();
  }

  @override
  void didUpdateWidget(covariant TopicMessagesPane oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.topicName != widget.topicName ||
        oldWidget.backendUrl != widget.backendUrl) {
      if (_looping) {
        unawaited(
          _stopLoopPublishingForTopic(
            topicName: oldWidget.topicName,
            topicType: oldWidget.topicType,
            updateState: false,
          ),
        );
      }
      _clearTopicTrafficState();
      _ownLoopActive = false;
      _lastRenderedTopicMessageAt = null;
      _messages.clear();
      _error = null;
      _imageViewEnabled = false;
      _connect();
    }
    if (oldWidget.topicName != widget.topicName ||
        oldWidget.topicType != widget.topicType ||
        oldWidget.backendUrl != widget.backendUrl) {
      _publishError = null;
      _publishInfo = null;
      _loadTemplate();
    }
  }

  @override
  void dispose() {
    if (_looping) {
      unawaited(
        _stopLoopPublishingForTopic(
          topicName: widget.topicName,
          topicType: widget.topicType,
          updateState: false,
        ),
      );
    }
    _topicTrafficTimer?.cancel();
    _loopFrequencyController.dispose();
    _channel?.sink.close();
    super.dispose();
  }

  void _connect() {
    // Recreate stream subscription whenever topic or backend changes.
    _channel?.sink.close();
    _connecting = true;
    final wsBase = RosApi.toWebSocketUrl(widget.backendUrl);
    final safeTopic = widget.topicName.startsWith('/')
        ? widget.topicName.substring(1)
        : widget.topicName;
    final uri = Uri.parse('$wsBase/ws/topics/$safeTopic');
    _channel = WebSocketChannel.connect(uri);

    _channel!.stream.listen(
      (event) {
        try {
          final payload = event is String
              ? event
              : utf8.decode(event as List<int>);
          final data = jsonDecode(payload) as Map<String, dynamic>;
          if (data.containsKey('error')) {
            setState(() => _error = data['error'] as String?);
            return;
          }
          final isTopicData = data.containsKey('timestamp');
          if (isTopicData) {
            _markTopicTraffic();
          }
          if (!_shouldRenderTopicMessage(data)) {
            return;
          }
          if (data['message'] != null && data['message'] is Map<String, dynamic>) {
            data['_render_message'] = const JsonEncoder.withIndent(
              '  ',
            ).convert(data['message']);
          }
          setState(() {
            _error = null;
            _connecting = false;
            _messages.insert(0, data);
            if (data['type']?.toString() == 'sensor_msgs/msg/Image') {
              if (_messages.length > 10) {
                _messages.removeLast();
              }
            } else if (_messages.length > 200) {
              _messages.removeLast();
            }
          });
        } catch (err) {
          setState(() {
            _connecting = false;
            _error = 'Failed to decode message: $err';
          });
        }
      },
      onError: (err) => setState(() => _error = err.toString()),
      onDone: () => setState(() {
        _connecting = false;
        _error ??= 'Connection closed';
      }),
    );
  }

  void _markTopicTraffic() {
    // Ignore traffic-detection while our own backend loop is active to prevent
    // self-blocking behavior in the publisher controls.
    if (_ownLoopActive || _looping) {
      return;
    }
    final now = DateTime.now();
    final ignoreUntil = _selfTrafficIgnoreUntil;
    if (ignoreUntil != null && now.isBefore(ignoreUntil)) {
      return;
    }

    if (!_topicBusyByExternalTraffic && mounted) {
      setState(() {
        _topicBusyByExternalTraffic = true;
      });
    } else {
      _topicBusyByExternalTraffic = true;
    }

    _topicTrafficTimer?.cancel();
    _topicTrafficTimer = Timer(const Duration(seconds: 2), () {
      if (!mounted) {
        return;
      }
      setState(() {
        _topicBusyByExternalTraffic = false;
      });
    });
  }

  bool _shouldRenderTopicMessage(Map<String, dynamic> data) {
    // Throttle high-frequency non-image messages to keep UI responsive.
    if (!data.containsKey('timestamp')) {
      return true;
    }
    final type = data['type']?.toString() ?? '';
    if (type == 'sensor_msgs/msg/Image') {
      return true;
    }
    final now = DateTime.now();
    final last = _lastRenderedTopicMessageAt;
    if (last != null && now.difference(last).inMilliseconds < 80) {
      return false;
    }
    _lastRenderedTopicMessageAt = now;
    return true;
  }

  void _clearTopicTrafficState() {
    _topicTrafficTimer?.cancel();
    _topicTrafficTimer = null;
    _topicBusyByExternalTraffic = false;
    _selfTrafficIgnoreUntil = null;
  }

  Future<void> _loadTemplate() async {
    // Load a dynamic ROS message template to generate the publish form fields.
    if (widget.topicType.isEmpty) {
      setState(() => _messageTemplate = <String, dynamic>{});
      return;
    }
    setState(() => _loadingTemplate = true);
    try {
      final schema = await RosApi(
        widget.backendUrl,
      ).getTopicMessageTemplate(widget.topicType);
      final template = schema['message_template'];
      if (template is! Map<String, dynamic>) {
        throw Exception('Invalid message template.');
      }
      setState(() {
        _messageTemplate = _deepCopyMap(template);
      });
    } catch (err) {
      setState(() => _publishError = err.toString());
    } finally {
      setState(() => _loadingTemplate = false);
    }
  }

  double? _parseLoopFrequencyHz() {
    final text = _loopFrequencyController.text.trim().replaceAll(',', '.');
    final hz = double.tryParse(text);
    if (hz == null || hz <= 0) {
      return null;
    }
    return hz;
  }

  Future<void> _startLoopPublishing() async {
    // Start backend-managed loop publishing with current payload+frequency.
    if (_looping || _loopActionInProgress) {
      return;
    }
    if (_topicBusyByExternalTraffic) {
      setState(() {
        _publishError = 'Publishing blocked: topic is already receiving messages.';
      });
      return;
    }
    final hz = _parseLoopFrequencyHz();
    if (hz == null) {
      setState(() {
        _publishError = 'Invalid frequency. Enter a value > 0.';
      });
      return;
    }
    final payload = _publishFormKey.currentState?.buildPayload();
    if (payload == null) {
      setState(() {
        _publishError = 'Payload unavailable for sending.';
      });
      return;
    }

    setState(() {
      _loopActionInProgress = true;
      _publishError = null;
      _publishInfo = null;
    });
    try {
      await RosApi(widget.backendUrl).startTopicPublishLoop(
        name: widget.topicName,
        messageType: widget.topicType,
        message: payload,
        frequencyHz: hz,
      );
      if (!mounted) {
        return;
      }
      setState(() {
        _looping = true;
        _ownLoopActive = true;
        _topicBusyByExternalTraffic = false;
        _publishInfo = 'Loop active at ${hz.toStringAsFixed(hz < 10 ? 2 : 1)} Hz.';
      });
    } catch (err) {
      if (mounted) {
        setState(() => _publishError = err.toString());
      }
    } finally {
      if (mounted) {
        setState(() => _loopActionInProgress = false);
      }
    }
  }

  void _armSelfTrafficIgnore() {
    // Small grace window to avoid treating immediate loop echoes as external traffic.
    var ignoreMs = 1200;
    if (ignoreMs < 300) {
      ignoreMs = 300;
    }
    if (ignoreMs > 2500) {
      ignoreMs = 2500;
    }
    _selfTrafficIgnoreUntil = DateTime.now().add(Duration(milliseconds: ignoreMs));
  }

  Future<void> _stopLoopPublishing({String? reason, bool updateState = true}) async {
    await _stopLoopPublishingForTopic(
      topicName: widget.topicName,
      topicType: widget.topicType,
      reason: reason,
      updateState: updateState,
    );
  }

  Future<void> _stopLoopPublishingForTopic({
    required String topicName,
    required String topicType,
    String? reason,
    bool updateState = true,
  }) async {
    // Stop loop remotely in backend and synchronize local UI state.
    final wasLooping = _looping;
    _looping = false;
    _ownLoopActive = false;
    _armSelfTrafficIgnore();
    if (!updateState || !mounted) {
      if (!wasLooping || topicType.isEmpty) {
        return;
      }
      try {
        await RosApi(widget.backendUrl).stopTopicPublishLoop(
          name: topicName,
          messageType: topicType,
        );
      } catch (_) {}
      return;
    }

    setState(() {
      _loopActionInProgress = true;
      if (reason != null) {
        _publishInfo = reason;
      }
    });

    if (!wasLooping || topicType.isEmpty) {
      if (mounted) {
        setState(() => _loopActionInProgress = false);
      }
      return;
    }

    try {
      await RosApi(widget.backendUrl).stopTopicPublishLoop(
        name: topicName,
        messageType: topicType,
      );
    } catch (err) {
      if (mounted) {
        setState(() => _publishError = err.toString());
      }
    } finally {
      if (mounted) {
        setState(() => _loopActionInProgress = false);
      }
    }
  }

  Future<void> _publishMessage() async {
    // Single-message publish path (manual button).
    if (_topicBusyByExternalTraffic) {
      setState(() {
        _publishError = 'Publishing blocked: topic is already receiving messages.';
      });
      return;
    }
    if (_publishing || _loopActionInProgress) {
      return;
    }
    setState(() {
      _publishing = true;
      _publishError = null;
      _publishInfo = null;
    });
    try {
      final payload = _publishFormKey.currentState?.buildPayload();
      if (payload == null) {
        throw Exception('Payload unavailable for sending.');
      }
      _armSelfTrafficIgnore();
      await RosApi(widget.backendUrl).publishTopicMessage(
        name: widget.topicName,
        messageType: widget.topicType,
        message: payload,
      );
      if (mounted) {
        setState(() {
          _publishInfo = 'Message published successfully.';
        });
      }
    } catch (err) {
      if (mounted) {
        setState(() => _publishError = err.toString());
      }
    } finally {
      if (mounted) {
        setState(() => _publishing = false);
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    final publishBlocked = _topicBusyByExternalTraffic && !_looping;
    if (_error != null) {
      return Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Error: $_error'),
            const SizedBox(height: 12),
            TextButton(onPressed: _connect, child: const Text('Reconnect')),
          ],
        ),
      );
    }

    return Column(
      children: [
        ListTile(
          title: Text('Topic: ${widget.topicName}'),
          subtitle: Text(
            _connecting ? 'Connecting...' : '${widget.topicType} • ${_messages.length} recent messages',
          ),
          trailing: _isImageTopic()
              ? OutlinedButton.icon(
                  onPressed: () {
                    setState(() {
                      _imageViewEnabled = !_imageViewEnabled;
                    });
                  },
                  icon: Icon(
                    _imageViewEnabled
                        ? Icons.text_snippet_outlined
                        : Icons.image_outlined,
                  ),
                  label: Text(_imageViewEnabled ? 'View text' : 'View image'),
                )
              : null,
        ),
        const Divider(height: 1),
        ExpansionTile(
          title: const Text('Publish message'),
          subtitle: Text(
            widget.topicType.isEmpty ? 'Type unavailable' : widget.topicType,
          ),
          childrenPadding: const EdgeInsets.fromLTRB(12, 0, 12, 12),
          children: [
            if (_loadingTemplate) const LinearProgressIndicator(minHeight: 2),
            if (_messageTemplate != null) ...[
              ConstrainedBox(
                constraints: BoxConstraints(
                  maxHeight: MediaQuery.of(context).size.height * 0.42,
                ),
                child: Scrollbar(
                  thumbVisibility: true,
                  child: SingleChildScrollView(
                    padding: const EdgeInsets.fromLTRB(0, 8, 0, 4),
                    child: _PayloadFieldsForm(
                      key: _publishFormKey,
                      template: _messageTemplate!,
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 12),
              TextField(
                controller: _loopFrequencyController,
                enabled: !_looping && !_loopActionInProgress,
                keyboardType: const TextInputType.numberWithOptions(
                  decimal: true,
                ),
                decoration: const InputDecoration(
                  labelText: 'Publish frequency (Hz)',
                  border: OutlineInputBorder(),
                ),
              ),
              const SizedBox(height: 10),
              Wrap(
                spacing: 8,
                runSpacing: 8,
                children: [
                  FilledButton.icon(
                    onPressed:
                        (_publishing ||
                            _loopActionInProgress ||
                            widget.topicType.isEmpty ||
                            publishBlocked)
                        ? null
                        : _publishMessage,
                    icon: const Icon(Icons.send),
                    label: Text(_publishing ? 'Publishing...' : 'Publish'),
                  ),
                  FilledButton.tonalIcon(
                    onPressed:
                        (widget.topicType.isEmpty || _loopActionInProgress)
                        ? null
                        : (_looping
                              ? () => _stopLoopPublishing()
                              : (publishBlocked
                                    ? null
                                    : _startLoopPublishing)),
                    icon: Icon(_looping ? Icons.stop : Icons.repeat),
                    label: Text(
                      _loopActionInProgress ? 'Processing...' : (_looping ? 'Stop loop' : 'Start loop'),
                    ),
                  ),
                ],
              ),
              if (publishBlocked) ...[
                const SizedBox(height: 8),
                Text(
                  'Publishing blocked: topic has active traffic.',
                  style: TextStyle(color: Theme.of(context).colorScheme.error),
                ),
              ],
            ] else if (!_loadingTemplate) ...[
              const Text('Could not load message fields.'),
            ],
            if (_publishInfo != null) ...[
              const SizedBox(height: 8),
              Text(
                _publishInfo!,
                style: TextStyle(color: Theme.of(context).colorScheme.primary),
              ),
            ],
            if (_publishError != null) ...[
              const SizedBox(height: 8),
              Text(
                'Error: $_publishError',
                style: TextStyle(color: Theme.of(context).colorScheme.error),
              ),
            ],
          ],
        ),
        const Divider(height: 1),
        Expanded(
          child: _imageViewEnabled && _isImageTopic()
              ? _buildImagePreview()
              : ListView.builder(
                  reverse: true,
                  itemCount: _messages.length,
                  itemBuilder: (context, index) {
                    final msg = _messages[index];
                    final ts = DateTime.fromMillisecondsSinceEpoch(
                      (((msg['timestamp'] as num?) ?? 0) * 1000).toInt(),
                    );
                    return ListTile(
                      dense: true,
                      title: Text(msg['type']?.toString() ?? ''),
                      subtitle: Text(
                        msg['_render_message']?.toString() ??
                            const JsonEncoder.withIndent('  ').convert(
                              msg['message'],
                            ),
                      ),
                      trailing: Text(
                        '${ts.hour.toString().padLeft(2, '0')}:${ts.minute.toString().padLeft(2, '0')}:${ts.second.toString().padLeft(2, '0')}',
                      ),
                    );
                  },
                ),
        ),
      ],
    );
  }

  bool _isImageTopic() {
    for (final msg in _messages) {
      if (msg['type']?.toString() == 'sensor_msgs/msg/Image') {
        return true;
      }
    }
    return false;
  }

  Widget _buildImagePreview() {
    // Render latest image frame with zoom/pan support when image mode is enabled.
    final frameMsg = _latestImageMessage();
    if (frameMsg == null) {
      return const Center(child: Text('Waiting for image frame...'));
    }

    final frame = frameMsg['message'] as Map<String, dynamic>;
    final png = _sensorImageToPng(frameMsg);
    if (png == null) {
      final encoding = frame['encoding']?.toString() ?? 'unknown';
      return Center(
        child: Text('Could not render image (encoding: $encoding)'),
      );
    }

    return Column(
      children: [
        Expanded(
          child: Container(
            color: Colors.black,
            alignment: Alignment.center,
            child: InteractiveViewer(
              minScale: 0.2,
              maxScale: 8.0,
              child: Image.memory(
                png,
                gaplessPlayback: true,
                filterQuality: FilterQuality.medium,
              ),
            ),
          ),
        ),
        Padding(
          padding: const EdgeInsets.all(8),
          child: Text(
            'encoding: ${frame['encoding']} | ${frame['width']}x${frame['height']} | ${frame['compressed_format'] ?? 'raw'} ${frame['compressed_size'] ?? ''} bytes',
          ),
        ),
      ],
    );
  }

  Map<String, dynamic>? _latestImageMessage() {
    for (final msg in _messages) {
      if (msg['type']?.toString() != 'sensor_msgs/msg/Image') {
        continue;
      }
      final payload = msg['message'];
      if (payload is Map<String, dynamic>) {
        return msg;
      }
    }
    return null;
  }

  Uint8List? _sensorImageToPng(Map<String, dynamic> wsMsg) {
    // Decode either compressed base64 frame or raw pixel payload to displayable bytes.
    if (wsMsg['image_b64'] is String) {
      return base64Decode(wsMsg['image_b64'] as String);
    }

    final message = wsMsg['message'] as Map<String, dynamic>;
    final width = (message['width'] as num?)?.toInt();
    final height = (message['height'] as num?)?.toInt();
    final step = (message['step'] as num?)?.toInt() ?? 0;
    final encoding = (message['encoding']?.toString() ?? '').toLowerCase();
    final data = message['data'];

    if (width == null || height == null || width <= 0 || height <= 0) {
      return null;
    }

    Uint8List raw;
    if (data is List<dynamic>) {
      raw = Uint8List.fromList(data.map((e) => (e as num).toInt()).toList());
    } else if (wsMsg['image_raw_b64'] is String) {
      raw = base64Decode(wsMsg['image_raw_b64'] as String);
    } else {
      return null;
    }
    final image = img.Image(width: width, height: height);

    int rowStrideFor(int channels) => step > 0 ? step : width * channels;

    if (encoding == 'rgb8') {
      final rowStride = rowStrideFor(3);
      for (var y = 0; y < height; y++) {
        for (var x = 0; x < width; x++) {
          final i = y * rowStride + x * 3;
          if (i + 2 >= raw.length) return null;
          image.setPixelRgb(x, y, raw[i], raw[i + 1], raw[i + 2]);
        }
      }
    } else if (encoding == 'bgr8') {
      final rowStride = rowStrideFor(3);
      for (var y = 0; y < height; y++) {
        for (var x = 0; x < width; x++) {
          final i = y * rowStride + x * 3;
          if (i + 2 >= raw.length) return null;
          image.setPixelRgb(x, y, raw[i + 2], raw[i + 1], raw[i]);
        }
      }
    } else if (encoding == 'rgba8') {
      final rowStride = rowStrideFor(4);
      for (var y = 0; y < height; y++) {
        for (var x = 0; x < width; x++) {
          final i = y * rowStride + x * 4;
          if (i + 3 >= raw.length) return null;
          image.setPixelRgba(x, y, raw[i], raw[i + 1], raw[i + 2], raw[i + 3]);
        }
      }
    } else if (encoding == 'bgra8') {
      final rowStride = rowStrideFor(4);
      for (var y = 0; y < height; y++) {
        for (var x = 0; x < width; x++) {
          final i = y * rowStride + x * 4;
          if (i + 3 >= raw.length) return null;
          image.setPixelRgba(x, y, raw[i + 2], raw[i + 1], raw[i], raw[i + 3]);
        }
      }
    } else if (encoding == 'mono8') {
      final rowStride = rowStrideFor(1);
      for (var y = 0; y < height; y++) {
        for (var x = 0; x < width; x++) {
          final i = y * rowStride + x;
          if (i >= raw.length) return null;
          image.setPixelRgb(x, y, raw[i], raw[i], raw[i]);
        }
      }
    } else {
      return null;
    }

    return Uint8List.fromList(img.encodePng(image));
  }
}

class _CreateTopicDialog extends StatefulWidget {
  /// Dialog used to create a publisher/topic from curated message types.
  const _CreateTopicDialog({required this.backendUrl});

  final String backendUrl;

  @override
  State<_CreateTopicDialog> createState() => _CreateTopicDialogState();
}

/// State for topic creation flow and optional initial publish.
class _CreateTopicDialogState extends State<_CreateTopicDialog> {
  // Topic creation form state.
  final TextEditingController _topicController = TextEditingController();
  final GlobalKey<_PayloadFieldsFormState> _payloadFormKey =
      GlobalKey<_PayloadFieldsFormState>();
  bool _loadingTypes = true;
  bool _loadingTemplate = false;
  bool _saving = false;
  bool _publishInitial = false;
  String? _error;
  Map<String, List<String>> _messageTypes = {};
  String? _selectedPackage;
  String? _selectedType;
  Map<String, dynamic>? _template;

  @override
  void initState() {
    super.initState();
    _loadMessageTypes();
  }

  @override
  void dispose() {
    _topicController.dispose();
    super.dispose();
  }

  Future<void> _loadMessageTypes() async {
    // Fetch backend-supported message types and prioritize common ROS packages.
    setState(() => _loadingTypes = true);
    try {
      final result = await RosApi(widget.backendUrl).getTopicMessageTypes();
      final ordered = <String, List<String>>{};
      for (final pkg in ['std_msgs', 'sensor_msgs', 'geometry_msgs']) {
        if (result.containsKey(pkg) && result[pkg]!.isNotEmpty) {
          ordered[pkg] = result[pkg]!;
        }
      }
      for (final entry in result.entries) {
        ordered.putIfAbsent(entry.key, () => entry.value);
      }
      final firstPackage = ordered.keys.isNotEmpty ? ordered.keys.first : null;
      final firstType = firstPackage == null
          ? null
          : ((ordered[firstPackage] ?? const <String>[]).isNotEmpty
                ? ordered[firstPackage]!.first
                : null);
      setState(() {
        _messageTypes = ordered;
        _selectedPackage = firstPackage;
        _selectedType = firstType;
      });
      await _loadTemplate();
    } catch (err) {
      setState(() => _error = err.toString());
    } finally {
      setState(() => _loadingTypes = false);
    }
  }

  Future<void> _loadTemplate() async {
    // Fetch selected message type template for dynamic field generation.
    final messageType = _selectedType;
    if (messageType == null || messageType.isEmpty) {
      setState(() => _template = null);
      return;
    }
    setState(() => _loadingTemplate = true);
    try {
      final schema = await RosApi(
        widget.backendUrl,
      ).getTopicMessageTemplate(messageType);
      final template = schema['message_template'];
      if (template is! Map<String, dynamic>) {
        throw Exception('Invalid message template.');
      }
      setState(() => _template = _deepCopyMap(template));
    } catch (err) {
      setState(() => _error = err.toString());
    } finally {
      setState(() => _loadingTemplate = false);
    }
  }

  Future<void> _createTopic() async {
    // Create publisher/topic and optionally publish an initial message payload.
    final topicName = _topicController.text.trim();
    final messageType = _selectedType;
    if (topicName.isEmpty) {
      setState(() => _error = 'Enter a topic name.');
      return;
    }
    if (messageType == null || messageType.isEmpty) {
      setState(() => _error = 'Select a message type.');
      return;
    }

    setState(() {
      _saving = true;
      _error = null;
    });
    try {
      final api = RosApi(widget.backendUrl);
      final created = await api.createTopicPublisher(
        name: topicName,
        messageType: messageType,
      );
      if (_publishInitial) {
        final payload = _payloadFormKey.currentState?.buildPayload();
        if (payload == null) {
          throw Exception('Initial payload unavailable.');
        }
        await api.publishTopicMessage(
          name: topicName,
          messageType: messageType,
          message: payload,
        );
      }
      if (!mounted) {
        return;
      }
      final normalizedName =
          created['name']?.toString() ??
          (topicName.startsWith('/') ? topicName : '/$topicName');
      Navigator.of(context).pop({
        'name': normalizedName,
        'types': [messageType],
      });
    } catch (err) {
      setState(() => _error = err.toString());
    } finally {
      setState(() => _saving = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final currentTypes = _selectedPackage == null
        ? const <String>[]
        : (_messageTypes[_selectedPackage] ?? const <String>[]);

    return AlertDialog(
      title: const Text('Create topic'),
      content: SizedBox(
        width: 620,
        child: SingleChildScrollView(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            mainAxisSize: MainAxisSize.min,
            children: [
              TextField(
                controller: _topicController,
                decoration: const InputDecoration(
                  labelText: 'Topic name',
                  hintText: '/my_topic',
                  border: OutlineInputBorder(),
                ),
              ),
              const SizedBox(height: 12),
              if (_loadingTypes) const LinearProgressIndicator(minHeight: 2),
              DropdownButtonFormField<String>(
                value: _selectedPackage,
                decoration: const InputDecoration(
                  labelText: 'Package',
                  border: OutlineInputBorder(),
                ),
                items: _messageTypes.keys
                    .map(
                      (pkg) => DropdownMenuItem(
                        value: pkg,
                        child: Text(pkg),
                      ),
                    )
                    .toList(),
                onChanged: _saving
                    ? null
                    : (value) {
                        if (value == null) {
                          return;
                        }
                        final types = _messageTypes[value] ?? const <String>[];
                        final firstType = types.isNotEmpty ? types.first : null;
                        setState(() {
                          _selectedPackage = value;
                          _selectedType = firstType;
                          _template = null;
                          _error = null;
                        });
                        _loadTemplate();
                      },
              ),
              const SizedBox(height: 12),
              DropdownButtonFormField<String>(
                value: currentTypes.contains(_selectedType)
                    ? _selectedType
                    : null,
                decoration: const InputDecoration(
                  labelText: 'Message type',
                  border: OutlineInputBorder(),
                ),
                items: currentTypes
                    .map(
                      (type) => DropdownMenuItem(
                        value: type,
                        child: Text(type),
                      ),
                    )
                    .toList(),
                onChanged: _saving
                    ? null
                    : (value) {
                        setState(() {
                          _selectedType = value;
                          _template = null;
                          _error = null;
                        });
                        _loadTemplate();
                      },
              ),
              const SizedBox(height: 12),
              CheckboxListTile(
                value: _publishInitial,
                onChanged: _saving
                    ? null
                    : (value) {
                        setState(() => _publishInitial = value ?? false);
                      },
                contentPadding: EdgeInsets.zero,
                title: const Text('Publish initial message after creating'),
              ),
              if (_loadingTemplate) const LinearProgressIndicator(minHeight: 2),
              if (_template != null) ...[
                const SizedBox(height: 8),
                _PayloadFieldsForm(key: _payloadFormKey, template: _template!),
              ],
              if (_error != null) ...[
                const SizedBox(height: 10),
                Text(
                  'Error: $_error',
                  style: TextStyle(color: Theme.of(context).colorScheme.error),
                ),
              ],
            ],
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: _saving ? null : () => Navigator.of(context).pop(),
          child: const Text('Cancel'),
        ),
        FilledButton(
          onPressed: _saving ? null : _createTopic,
          child: Text(_saving ? 'Creating...' : 'Create'),
        ),
      ],
    );
  }
}

enum _PayloadFieldKind {
  stringValue,
  intValue,
  doubleValue,
  boolValue,
  jsonListValue,
}

class _PayloadFieldSpec {
  /// Metadata describing a single editable payload field.
  _PayloadFieldSpec({
    required this.path,
    required this.kind,
    this.initialText,
    this.initialBool,
  });

  final List<String> path;
  final _PayloadFieldKind kind;
  final String? initialText;
  final bool? initialBool;
}

/// Dynamic payload form generated from a ROS message template map.
class _PayloadFieldsForm extends StatefulWidget {
  const _PayloadFieldsForm({super.key, required this.template});

  final Map<String, dynamic> template;

  @override
  State<_PayloadFieldsForm> createState() => _PayloadFieldsFormState();
}

/// Flattens nested template fields into editable widgets and rebuilds payload JSON.
class _PayloadFieldsFormState extends State<_PayloadFieldsForm> {
  // Flattened field specs generated from nested message template map.
  late List<_PayloadFieldSpec> _fields;
  final Map<String, TextEditingController> _controllers = {};
  final Map<String, bool> _boolValues = {};
  bool _hasHeaderStamp = false;

  @override
  void initState() {
    super.initState();
    _rebuildFields();
  }

  @override
  void didUpdateWidget(covariant _PayloadFieldsForm oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.template != widget.template) {
      _disposeControllers();
      _rebuildFields();
    }
  }

  @override
  void dispose() {
    _disposeControllers();
    super.dispose();
  }

  void _disposeControllers() {
    for (final controller in _controllers.values) {
      controller.dispose();
    }
    _controllers.clear();
    _boolValues.clear();
  }

  void _rebuildFields() {
    // Rebuild controllers/toggles whenever message template changes.
    _hasHeaderStamp = false;
    _fields = _flattenFields(widget.template);
    for (final spec in _fields) {
      final key = _pathKey(spec.path);
      if (spec.kind == _PayloadFieldKind.boolValue) {
        _boolValues[key] = spec.initialBool ?? false;
      } else {
        _controllers[key] = TextEditingController(text: spec.initialText ?? '');
      }
    }
  }

  List<_PayloadFieldSpec> _flattenFields(
    Map<String, dynamic> template,
  ) {
    // Convert nested map template into a flat list of editable field specs.
    final specs = <_PayloadFieldSpec>[];

    void walk(dynamic node, List<String> path) {
      if (_isHeaderStampPath(path)) {
        _hasHeaderStamp = true;
        return;
      }
      if (node is Map<String, dynamic>) {
        for (final entry in node.entries) {
          walk(entry.value, [...path, entry.key]);
        }
        return;
      }
      if (node is List<dynamic>) {
        specs.add(
          _PayloadFieldSpec(
            path: path,
            kind: _PayloadFieldKind.jsonListValue,
            initialText: const JsonEncoder.withIndent('  ').convert(node),
          ),
        );
        return;
      }
      if (node is bool) {
        specs.add(
          _PayloadFieldSpec(
            path: path,
            kind: _PayloadFieldKind.boolValue,
            initialBool: node,
          ),
        );
        return;
      }
      if (node is int) {
        specs.add(
          _PayloadFieldSpec(
            path: path,
            kind: _PayloadFieldKind.intValue,
            initialText: node.toString(),
          ),
        );
        return;
      }
      if (node is double) {
        specs.add(
          _PayloadFieldSpec(
            path: path,
            kind: _PayloadFieldKind.doubleValue,
            initialText: node.toString(),
          ),
        );
        return;
      }
      specs.add(
        _PayloadFieldSpec(
          path: path,
          kind: _PayloadFieldKind.stringValue,
          initialText: node?.toString() ?? '',
        ),
      );
    }

    for (final entry in template.entries) {
      walk(entry.value, [entry.key]);
    }
    return specs;
  }

  bool _isHeaderStampPath(List<String> path) {
    if (path.length < 3) {
      return false;
    }
    final n = path.length;
    final tail = path[n - 1];
    return path[n - 3] == 'header' &&
        path[n - 2] == 'stamp' &&
        (tail == 'sec' || tail == 'nanosec');
  }

  Map<String, dynamic> buildPayload() {
    // Recompose nested payload map from current form field values.
    final payload = _deepCopyMap(widget.template);
    for (final spec in _fields) {
      final key = _pathKey(spec.path);
      dynamic parsedValue;
      try {
        switch (spec.kind) {
          case _PayloadFieldKind.stringValue:
            parsedValue = _controllers[key]!.text;
            break;
          case _PayloadFieldKind.intValue:
            parsedValue = int.parse(_controllers[key]!.text.trim());
            break;
          case _PayloadFieldKind.doubleValue:
            parsedValue = double.parse(_controllers[key]!.text.trim());
            break;
          case _PayloadFieldKind.boolValue:
            parsedValue = _boolValues[key] ?? false;
            break;
          case _PayloadFieldKind.jsonListValue:
            final decoded = jsonDecode(_controllers[key]!.text);
            if (decoded is! List<dynamic>) {
              throw const FormatException('must be a JSON list');
            }
            parsedValue = decoded;
            break;
        }
      } catch (err) {
        throw Exception('Invalid field ${spec.path.join('.')}: $err');
      }
      _setValueAtPath(payload, spec.path, parsedValue);
    }
    return payload;
  }

  void _setValueAtPath(
    Map<String, dynamic> root,
    List<String> path,
    dynamic value,
  ) {
    // Generic setter for nested map paths like `header.frame_id`.
    dynamic current = root;
    for (var i = 0; i < path.length - 1; i++) {
      final segment = path[i];
      if (current is Map<String, dynamic>) {
        current = current[segment];
      } else {
        throw Exception('Invalid structure at path ${path.join('.')}');
      }
    }
    if (current is! Map<String, dynamic>) {
      throw Exception('Invalid structure at path ${path.join('.')}');
    }
    current[path.last] = value;
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        const SizedBox(height: 4),
        if (_hasHeaderStamp)
          Padding(
            padding: const EdgeInsets.only(bottom: 8),
            child: Text(
              'Header timestamp will be filled automatically.',
              style: TextStyle(color: Theme.of(context).colorScheme.primary),
            ),
          ),
        for (final spec in _fields) ...[
          if (spec.kind == _PayloadFieldKind.boolValue)
            SwitchListTile(
              contentPadding: EdgeInsets.zero,
              title: Text(spec.path.join('.')),
              value: _boolValues[_pathKey(spec.path)] ?? false,
              onChanged: (value) {
                setState(() {
                  _boolValues[_pathKey(spec.path)] = value;
                });
              },
            )
          else
            TextField(
              controller: _controllers[_pathKey(spec.path)],
              maxLines: spec.kind == _PayloadFieldKind.jsonListValue ? 4 : 1,
              keyboardType: spec.kind == _PayloadFieldKind.intValue
                  ? TextInputType.number
                  : (spec.kind == _PayloadFieldKind.doubleValue
                        ? const TextInputType.numberWithOptions(
                            decimal: true,
                            signed: true,
                          )
                        : TextInputType.text),
              decoration: InputDecoration(
                labelText: spec.path.join('.'),
                hintText: spec.kind == _PayloadFieldKind.jsonListValue
                    ? '[\n  ...\n]'
                    : null,
                border: const OutlineInputBorder(),
              ),
              style: spec.kind == _PayloadFieldKind.jsonListValue
                  ? const TextStyle(fontFamily: 'monospace')
                  : null,
            ),
          const SizedBox(height: 8),
        ],
      ],
    );
  }
}

String _pathKey(List<String> path) => path.join('.');

Map<String, dynamic> _deepCopyMap(Map<String, dynamic> value) {
  return jsonDecode(jsonEncode(value)) as Map<String, dynamic>;
}

class ResponsiveSplitPane extends StatefulWidget {
  /// Reusable resizable split-pane for desktop and stacked layout for mobile.
  const ResponsiveSplitPane({
    super.key,
    required this.left,
    required this.right,
    this.initialLeftFraction = 0.4,
  });

  final Widget left;
  final Widget right;
  final double initialLeftFraction;

  @override
  State<ResponsiveSplitPane> createState() => _ResponsiveSplitPaneState();
}

/// Implements drag-resize behavior and responsive fallback layout.
class _ResponsiveSplitPaneState extends State<ResponsiveSplitPane> {
  static const _dividerWidth = 12.0;
  static const _minPaneWidth = 220.0;
  late double _leftFraction;

  @override
  void initState() {
    super.initState();
    _leftFraction = widget.initialLeftFraction.clamp(0.2, 0.8);
  }

  double _clampFraction(double width, double fraction) {
    final minFraction = (_minPaneWidth / width).clamp(0.1, 0.9);
    final maxFraction = 1.0 - minFraction;
    return fraction.clamp(minFraction, maxFraction);
  }

  @override
  Widget build(BuildContext context) {
    final width = MediaQuery.of(context).size.width;
    if (width < 700) {
      return Column(
        children: [
          Expanded(child: widget.left),
          const Divider(height: 1),
          Expanded(child: widget.right),
        ],
      );
    }

    return LayoutBuilder(
      builder: (context, constraints) {
        final totalWidth = constraints.maxWidth;
        final leftFraction = _clampFraction(totalWidth, _leftFraction);
        final leftWidth = (totalWidth - _dividerWidth) * leftFraction;
        final rightWidth = totalWidth - _dividerWidth - leftWidth;

        return Row(
          children: [
            SizedBox(width: leftWidth, child: widget.left),
            MouseRegion(
              cursor: SystemMouseCursors.resizeColumn,
              child: GestureDetector(
                behavior: HitTestBehavior.opaque,
                onHorizontalDragUpdate: (details) {
                  setState(() {
                    _leftFraction = _clampFraction(
                      totalWidth,
                      _leftFraction + (details.delta.dx / totalWidth),
                    );
                  });
                },
                child: Container(
                  width: _dividerWidth,
                  color: Theme.of(context).dividerColor.withValues(alpha: 0.25),
                  child: Center(
                    child: Container(
                      width: 3,
                      height: 48,
                      decoration: BoxDecoration(
                        color: Theme.of(context).dividerColor,
                        borderRadius: BorderRadius.circular(2),
                      ),
                    ),
                  ),
                ),
              ),
            ),
            SizedBox(width: rightWidth, child: widget.right),
          ],
        );
      },
    );
  }
}

/// Thin HTTP/WebSocket helper used by UI widgets to call backend endpoints.
class RosApi {
  RosApi(this.baseUrl);

  final String baseUrl;

  static String defaultBaseUrl() {
    // Derive backend base URL from current page host, forcing API port 8000.
    final host = Uri.base.host.isEmpty ? 'localhost' : Uri.base.host;
    final scheme = Uri.base.scheme == 'https' ? 'https' : 'http';
    return '$scheme://$host:8000';
  }

  Future<List<Map<String, dynamic>>> getTopics() async {
    // Graph discovery: topics.
    return _getList('/topics');
  }

  Future<Map<String, List<String>>> getTopicMessageTypes() async {
    // Curated types shown in "create topic" dialog.
    final uri = Uri.parse('$baseUrl/topic-message-types');
    final response = await http.get(uri);
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    final data = jsonDecode(response.body) as Map<String, dynamic>;
    return data.map(
      (key, value) => MapEntry(
        key,
        (value as List<dynamic>).map((e) => e.toString()).toList(),
      ),
    );
  }

  Future<Map<String, dynamic>> getTopicMessageTemplate(String messageType) async {
    // Template used to generate dynamic publish form fields.
    final uri = Uri.parse('$baseUrl/topic-message-template').replace(
      queryParameters: {'message_type': messageType},
    );
    final response = await http.get(uri);
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<Map<String, dynamic>> createTopicPublisher({
    required String name,
    required String messageType,
    int qosDepth = 10,
  }) async {
    // Explicit publisher creation/reuse in backend.
    final uri = Uri.parse('$baseUrl/topic-publisher');
    final response = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'name': name,
        'message_type': messageType,
        'qos_depth': qosDepth,
      }),
    );
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<Map<String, dynamic>> publishTopicMessage({
    required String name,
    required String messageType,
    required Map<String, dynamic> message,
    int qosDepth = 10,
  }) async {
    // One-shot topic publish endpoint.
    final uri = Uri.parse('$baseUrl/topic-publish');
    final response = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'name': name,
        'message_type': messageType,
        'message': message,
        'qos_depth': qosDepth,
      }),
    );
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<Map<String, dynamic>> startTopicPublishLoop({
    required String name,
    required String messageType,
    required Map<String, dynamic> message,
    required double frequencyHz,
    int qosDepth = 10,
  }) async {
    // Start backend-managed loop publishing.
    final uri = Uri.parse('$baseUrl/topic-publish-loop/start');
    final response = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'name': name,
        'message_type': messageType,
        'message': message,
        'frequency_hz': frequencyHz,
        'qos_depth': qosDepth,
      }),
    );
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<Map<String, dynamic>> stopTopicPublishLoop({
    required String name,
    required String messageType,
  }) async {
    // Stop backend-managed loop publishing.
    final uri = Uri.parse('$baseUrl/topic-publish-loop/stop');
    final response = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'name': name,
        'message_type': messageType,
      }),
    );
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<List<Map<String, dynamic>>> getNodes() async {
    // Graph discovery: nodes.
    return _getList('/nodes');
  }

  Future<List<Map<String, dynamic>>> getServices() async {
    // Graph discovery: services.
    return _getList('/services');
  }

  Future<Map<String, dynamic>> getServiceSchema(
    String serviceName,
    String serviceType,
  ) async {
    // Retrieve request/response templates for selected service type.
    final uri = Uri.parse('$baseUrl/service-schema').replace(
      queryParameters: {'name': serviceName, 'service_type': serviceType},
    );
    final response = await http.get(uri);
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<Map<String, dynamic>> callService({
    required String name,
    required String serviceType,
    required Map<String, dynamic> request,
    double timeoutSec = 3.0,
  }) async {
    // Invoke ROS service through backend.
    final uri = Uri.parse('$baseUrl/service-call');
    final response = await http.post(
      uri,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'name': name,
        'service_type': serviceType,
        'request': request,
        'timeout_sec': timeoutSec,
      }),
    );
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}: ${response.body}');
    }
    return jsonDecode(response.body) as Map<String, dynamic>;
  }

  Future<List<Map<String, dynamic>>> _getList(String path) async {
    // Shared helper for endpoints returning list payloads.
    final uri = Uri.parse('$baseUrl$path');
    final response = await http.get(uri);
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}');
    }
    final data = jsonDecode(response.body) as List<dynamic>;
    return data.cast<Map<String, dynamic>>();
  }

  static String toWebSocketUrl(String httpUrl) {
    // Convert API base URL into matching WebSocket scheme for live streams.
    if (httpUrl.startsWith('https://')) {
      return httpUrl.replaceFirst('https://', 'wss://');
    }
    if (httpUrl.startsWith('http://')) {
      return httpUrl.replaceFirst('http://', 'ws://');
    }
    return 'ws://$httpUrl';
  }
}
