import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:image/image.dart' as img;
import 'package:web_socket_channel/web_socket_channel.dart';

void main() {
  runApp(const RosInspectorApp());
}

class RosInspectorApp extends StatelessWidget {
  const RosInspectorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ROS2 Inspector',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF1F6FEB)),
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
        title: const Text('ROS2 Inspector'),
        bottom: TabBar(
          controller: _tabController,
          tabs: const [
            Tab(text: 'Tópicos'),
            Tab(text: 'Nós'),
            Tab(text: 'Serviços'),
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

class TopicsPage extends StatefulWidget {
  const TopicsPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  State<TopicsPage> createState() => _TopicsPageState();
}

class _TopicsPageState extends State<TopicsPage> {
  String? _selectedTopic;

  @override
  Widget build(BuildContext context) {
    return ResponsiveSplitPane(
      left: _TopicsList(
        backendUrl: widget.backendUrl,
        onSelect: (topic) => setState(() => _selectedTopic = topic),
      ),
      right: _selectedTopic == null
          ? const Center(child: Text('Selecione um tópico para ver mensagens.'))
          : TopicMessagesPane(
              backendUrl: widget.backendUrl,
              topicName: _selectedTopic!,
            ),
    );
  }
}

class _TopicsList extends StatefulWidget {
  const _TopicsList({required this.backendUrl, required this.onSelect});

  final String backendUrl;
  final ValueChanged<String> onSelect;

  @override
  State<_TopicsList> createState() => _TopicsListState();
}

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
                    hintText: 'Buscar tópico...',
                    prefixIcon: const Icon(Icons.search),
                    suffixIcon: _searchQuery.isNotEmpty
                        ? IconButton(
                            tooltip: 'Limpar busca',
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
                tooltip: 'Atualizar',
                onPressed: () {
                  setState(() {
                    _future = RosApi(widget.backendUrl).getTopics();
                  });
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
                return Center(child: Text('Erro: ${snapshot.error}'));
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
                  child: Text('Nenhum tópico encontrado para o filtro atual.'),
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
                    onTap: () => widget.onSelect(name),
                    trailing: IconButton(
                      tooltip: _favoriteTopics.contains(name)
                          ? 'Desfavoritar'
                          : 'Favoritar',
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

class NodesPage extends StatelessWidget {
  const NodesPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  Widget build(BuildContext context) {
    return _SimpleListPage(
      title: 'Nós ativos',
      fetcher: () => RosApi(backendUrl).getNodes(),
      itemBuilder: (item) => '${item['namespace']}${item['name']}',
    );
  }
}

class ServicesPage extends StatelessWidget {
  const ServicesPage({super.key, required this.backendUrl});

  final String backendUrl;

  @override
  Widget build(BuildContext context) {
    return _ServicesSplitPage(backendUrl: backendUrl);
  }
}

class _ServicesSplitPage extends StatefulWidget {
  const _ServicesSplitPage({required this.backendUrl});

  final String backendUrl;

  @override
  State<_ServicesSplitPage> createState() => _ServicesSplitPageState();
}

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
              child: Text('Selecione um serviço para visualizar/chamar.'),
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

class _ServicesList extends StatefulWidget {
  const _ServicesList({required this.backendUrl, required this.onSelect});

  final String backendUrl;
  final ValueChanged<Map<String, dynamic>> onSelect;

  @override
  State<_ServicesList> createState() => _ServicesListState();
}

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
                    hintText: 'Buscar serviço...',
                    prefixIcon: const Icon(Icons.search),
                    suffixIcon: _searchQuery.isNotEmpty
                        ? IconButton(
                            tooltip: 'Limpar busca',
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
                tooltip: 'Atualizar',
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
                return Center(child: Text('Erro: ${snapshot.error}'));
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
                  child: Text('Nenhum serviço encontrado para o filtro atual.'),
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
                          ? 'Desfavoritar'
                          : 'Favoritar',
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
        throw Exception('Payload deve ser um objeto JSON.');
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
            widget.serviceType.isEmpty
                ? 'Tipo indisponível'
                : widget.serviceType,
          ),
          trailing: IconButton(
            tooltip: 'Recarregar schema',
            onPressed: _loadingSchema ? null : _loadSchema,
            icon: const Icon(Icons.refresh),
          ),
        ),
        if (_loadingSchema) const LinearProgressIndicator(minHeight: 2),
        Expanded(
          child: ListView(
            padding: const EdgeInsets.all(12),
            children: [
              const Text('Payload esperado (request template):'),
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
              const Text('Payload para envio (JSON):'),
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
                label: Text(_sending ? 'Enviando...' : 'Enviar serviço'),
              ),
              if (_error != null) ...[
                const SizedBox(height: 12),
                Text(
                  'Erro: $_error',
                  style: TextStyle(color: Theme.of(context).colorScheme.error),
                ),
              ],
              const SizedBox(height: 16),
              const Text('Formato esperado de resposta (response template):'),
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
                const Text('Resposta recebida:'),
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
            tooltip: 'Atualizar',
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
                return Center(child: Text('Erro: ${snapshot.error}'));
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

class TopicMessagesPane extends StatefulWidget {
  const TopicMessagesPane({
    super.key,
    required this.backendUrl,
    required this.topicName,
  });

  final String backendUrl;
  final String topicName;

  @override
  State<TopicMessagesPane> createState() => _TopicMessagesPaneState();
}

class _TopicMessagesPaneState extends State<TopicMessagesPane> {
  WebSocketChannel? _channel;
  final List<Map<String, dynamic>> _messages = [];
  String? _error;
  bool _connecting = false;
  bool _imageViewEnabled = false;

  @override
  void initState() {
    super.initState();
    _connect();
  }

  @override
  void didUpdateWidget(covariant TopicMessagesPane oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.topicName != widget.topicName ||
        oldWidget.backendUrl != widget.backendUrl) {
      _messages.clear();
      _error = null;
      _imageViewEnabled = false;
      _connect();
    }
  }

  @override
  void dispose() {
    _channel?.sink.close();
    super.dispose();
  }

  void _connect() {
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
            _error = 'Falha ao decodificar mensagem: $err';
          });
        }
      },
      onError: (err) => setState(() => _error = err.toString()),
      onDone: () => setState(() {
        _connecting = false;
        _error ??= 'Conexão encerrada';
      }),
    );
  }

  @override
  Widget build(BuildContext context) {
    if (_error != null) {
      return Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Text('Erro: $_error'),
            const SizedBox(height: 12),
            TextButton(onPressed: _connect, child: const Text('Reconectar')),
          ],
        ),
      );
    }

    return Column(
      children: [
        ListTile(
          title: Text('Tópico: ${widget.topicName}'),
          subtitle: Text(
            _connecting
                ? 'Conectando...'
                : 'Mostrando ${_messages.length} mensagens recentes',
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
                  label: Text(_imageViewEnabled ? 'Ver texto' : 'Ver imagem'),
                )
              : null,
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
                        const JsonEncoder.withIndent(
                          '  ',
                        ).convert(msg['message']),
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
    final frameMsg = _latestImageMessage();
    if (frameMsg == null) {
      return const Center(child: Text('Aguardando frame de imagem...'));
    }

    final frame = frameMsg['message'] as Map<String, dynamic>;
    final png = _sensorImageToPng(frameMsg);
    if (png == null) {
      final encoding = frame['encoding']?.toString() ?? 'desconhecido';
      return Center(
        child: Text('Nao foi possivel renderizar imagem (encoding: $encoding)'),
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

class ResponsiveSplitPane extends StatefulWidget {
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

class RosApi {
  RosApi(this.baseUrl);

  final String baseUrl;

  static String defaultBaseUrl() {
    final host = Uri.base.host.isEmpty ? 'localhost' : Uri.base.host;
    final scheme = Uri.base.scheme == 'https' ? 'https' : 'http';
    return '$scheme://$host:8000';
  }

  Future<List<Map<String, dynamic>>> getTopics() async {
    return _getList('/topics');
  }

  Future<List<Map<String, dynamic>>> getNodes() async {
    return _getList('/nodes');
  }

  Future<List<Map<String, dynamic>>> getServices() async {
    return _getList('/services');
  }

  Future<Map<String, dynamic>> getServiceSchema(
    String serviceName,
    String serviceType,
  ) async {
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
    final uri = Uri.parse('$baseUrl$path');
    final response = await http.get(uri);
    if (response.statusCode != 200) {
      throw Exception('HTTP ${response.statusCode}');
    }
    final data = jsonDecode(response.body) as List<dynamic>;
    return data.cast<Map<String, dynamic>>();
  }

  static String toWebSocketUrl(String httpUrl) {
    if (httpUrl.startsWith('https://')) {
      return httpUrl.replaceFirst('https://', 'wss://');
    }
    if (httpUrl.startsWith('http://')) {
      return httpUrl.replaceFirst('http://', 'ws://');
    }
    return 'ws://$httpUrl';
  }
}
