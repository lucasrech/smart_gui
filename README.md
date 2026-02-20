# 🛰️ smart_gui

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-Backend-009688?logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com/)
[![Flutter Web](https://img.shields.io/badge/Flutter-Web-02569B?logo=flutter&logoColor=white)](https://flutter.dev/)
![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)

`smart_gui` is a ROS 2 (`ament_python`) package focused on monitoring and testing ROS systems through a web interface. It combines a FastAPI backend and a Flutter frontend to inspect the ROS graph, stream live topic data, call services, and publish messages (including backend-managed loop publishing) without requiring custom debug scripts.

## ✨ Features

- Graph inspection:
  - `GET /topics`
  - `GET /nodes`
  - `GET /services`
- Service tooling:
  - `GET /service-schema`
  - `POST /service-call`
- Topic tooling:
  - Create a publisher for a topic/type.
  - Publish one message.
  - Start/stop backend publish loops (no per-message HTTP traffic from frontend).
- Live topic monitor via WebSocket:
  - `WS /ws/topics/{topic}`
- Special handling for `sensor_msgs/msg/Image`:
  - Backend JPEG compression to reduce payload size.

## 🗂️ Repository Layout

- `smart_gui/ros2_inspector_api.py`: main ROS2 + FastAPI backend node.
- `frontend/`: Flutter web application.
- `launch/smart_gui_api.launch.py`: launch backend and optional frontend process.
- `smart_gui/random_int8_topics_node.py`: test node that publishes random `std_msgs/msg/Int8` on random topics.

## 📦 Requirements

- ROS 2 Jazzy (or compatible environment with `rclpy`).
- `colcon`.
- Python 3.
- Flutter (only if you want to run the web frontend with `flutter run`).

## 🛠️ Build

From your ROS 2 workspace root:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select smart_gui --symlink-install
source install/setup.bash
```

## ▶️ Run

### 1) Backend + frontend together (recommended for development)

```bash
ros2 launch smart_gui smart_gui_api.launch.py \
  frontend_command:='NO_PROXY=localhost,127.0.0.1 no_proxy=localhost,127.0.0.1 flutter run -d web-server --web-hostname 0.0.0.0 --web-port 3000 --profile'
```

Open:
- Local machine: `http://localhost:3000`
- Another device in LAN: `http://<YOUR_MACHINE_IP>:3000`

### 2) Backend only

```bash
ros2 launch smart_gui smart_gui_api.launch.py run_frontend:=false
```

### 3) Backend only (direct run)

```bash
ros2 run smart_gui smart_gui_api --host 0.0.0.0 --port 8000
```

## 🔌 API Endpoints

### Basic
- `GET /health`
- `GET /topics`
- `GET /nodes`
- `GET /services`

### Topic message types/templates
- `GET /topic-message-types`
- `GET /topic-message-template?message_type=<pkg/msg/Type>`

### Topic publish control
- `POST /topic-publisher`
- `POST /topic-publish`
- `POST /topic-publish-loop/start`
- `POST /topic-publish-loop/stop`

### Services
- `GET /service-schema?name=<service>&service_type=<pkg/srv/Type>`
- `POST /service-call`

### WebSocket
- `WS /ws/topics/{topic}`

## 🔁 Backend-Managed Topic Loop Publishing

Loop publishing is handled by the backend (thread per topic/type loop), so the frontend does not send one HTTP request per message.

Example:

```bash
curl -X POST http://127.0.0.1:8000/topic-publish-loop/start \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "/demo_int",
    "message_type": "std_msgs/msg/Int32",
    "message": {"data": 10},
    "frequency_hz": 5.0
  }'
```

Stop loop:

```bash
curl -X POST http://127.0.0.1:8000/topic-publish-loop/stop \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "/demo_int",
    "message_type": "std_msgs/msg/Int32"
  }'
```

## 🧪 Random Test Publisher Node

Run a helper node that creates random topic names and publishes random `Int8` values:

```bash
ros2 run smart_gui random_int8_topics
```

Optional arguments:

```bash
ros2 run smart_gui random_int8_topics --topic-count 5 --hz 5.0
```

## 📱 Notes for Mobile/LAN Access

- The backend usually runs on port `8000`, frontend on `3000`.
- If running inside WSL2, make sure Windows forwards ports (3000/8000) to WSL and firewall rules allow inbound connections.
- Access from phone should use Windows LAN IP, not WSL internal IP.

## 🧭 Troubleshooting

- Backend health check:

```bash
curl http://127.0.0.1:8000/health
```

- Confirm loop endpoints are available:

```bash
curl -s http://127.0.0.1:8000/openapi.json | rg 'topic-publish-loop/start|topic-publish-loop/stop'
```

- If frontend looks stale on mobile browser, force refresh or open in private/incognito window.
