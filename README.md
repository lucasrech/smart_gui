# smart_gui

## Resumo do pacote e dependências
O `smart_gui` é um pacote ROS 2 (`ament_python`) que fornece:
- Backend HTTP/WebSocket em FastAPI para inspeção do grafo ROS 2.
- Interface web Flutter para listar tópicos, nós e serviços.
- Streaming de mensagens por tópico via WebSocket.
- Visualização de tópicos `sensor_msgs/msg/Image` com compressão JPEG no backend para reduzir payload.

Dependências principais do backend Python:
- `fastapi`
- `uvicorn[standard]`
- `wsproto`
- `numpy`
- `pillow`
- `rosidl-runtime-py`

Dependências ROS 2 (via sistema):
- `rclpy`
- `rosidl_runtime_py`

Dependências de interface (Flutter):
- Projeto em `frontend/` com dependências no `frontend/pubspec.yaml`.

Arquivo de dependências Python deste pacote:
- `requirements.txt`

## Como funciona e para que serve
O objetivo do `smart_gui` é facilitar inspeção e depuração de sistemas ROS 2 em uma interface web.

Fluxo geral:
1. O backend cria um nó ROS 2 (`ros2_inspector_api`) e consulta:
- tópicos (`/topics`)
- nós (`/nodes`)
- serviços (`/services`)

2. Quando a interface seleciona um tópico, ela abre um WebSocket em:
- `/ws/topics/{topic}`

3. O backend cria uma subscription ROS 2 para o tópico e encaminha mensagens para a interface.

4. Para mensagens comuns, envia o JSON completo (`message_to_ordereddict`).

5. Para `sensor_msgs/msg/Image`, o backend:
- converte o frame
- comprime para JPEG
- envia base64 do JPEG + metadados
Isso reduz bastante o volume transferido e evita travamentos no navegador.

A interface possui:
- Aba `Tópicos` com painel dividido (lista + visualização)
- Divisor arrastável para redimensionar os dois painéis
- Alternância entre `Ver texto` e `Ver imagem` para tópicos de imagem

## Tutorial de uso
### 1. Pré-requisitos
- ROS 2 Jazzy instalado
- `colcon` instalado
- Flutter instalado (para rodar a interface via `flutter run`)

### 2. Build do pacote
No workspace ROS 2:
```bash
cd ~/lacbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select smart_gui
source install/setup.bash
```

### 3. Executar backend + interface juntos
```bash
ros2 launch smart_gui smart_gui_api.launch.py \
  frontend_command:='flutter run -d web-server --web-hostname 0.0.0.0 --web-port 3000'
```

Acessar no navegador:
- `http://localhost:3000` (máquina local)
- `http://IP_DA_MAQUINA:3000` (outro dispositivo na rede)

### 4. Executar somente backend
```bash
ros2 launch smart_gui smart_gui_api.launch.py run_frontend:=false
```

### 5. Executar backend sem launch (direto)
```bash
ros2 run smart_gui smart_gui_api --host 0.0.0.0 --port 8000
```

### 6. Endpoints disponíveis
- `GET /health`
- `GET /topics`
- `GET /nodes`
- `GET /services`
- `WS /ws/topics/{topic}`

### 7. Uso da interface
1. Abra a aba `Tópicos`.
2. Clique em um tópico da lista.
3. No painel da direita:
- Mensagens padrão: texto JSON.
- `sensor_msgs/msg/Image`: botão para alternar entre `Ver texto` e `Ver imagem`.
4. Arraste a barra vertical entre os painéis para ajustar o tamanho da lista e da visualização.

### 8. Troubleshooting rápido
- Se a interface não carregar dados, teste backend:
```bash
curl http://localhost:8000/health
```
- Se aparecerem muitos warnings do CycloneDDS, tente:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
```
- Se o Flutter mostrar versão antiga da página, force reload no navegador (`Ctrl+F5`).
