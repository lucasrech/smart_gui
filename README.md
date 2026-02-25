# 🛰️ smart_gui

[![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![FastAPI](https://img.shields.io/badge/FastAPI-Backend-009688?logo=fastapi&logoColor=white)](https://fastapi.tiangolo.com/)
[![Flutter Web](https://img.shields.io/badge/Flutter-Web-02569B?logo=flutter&logoColor=white)](https://flutter.dev/)
![License](https://img.shields.io/badge/License-Apache--2.0-blue.svg)

> Branch note: this README documents the `ros-noetic` branch (ROS 1 Noetic version).

`smart_gui` is a ROS 1 Noetic (`catkin`) package focused on monitoring and testing ROS systems through a web interface. It combines a FastAPI backend and a Flutter frontend to inspect the ROS graph, stream live topic data, call services, and publish messages (including backend-managed loop publishing) without requiring custom debug scripts.

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
- Special handling for `sensor_msgs/Image`:
  - Backend JPEG compression to reduce payload size.

## 🗂️ Repository Layout

- `smart_gui/ros_inspector_api.py`: main ROS Noetic + FastAPI backend node.
- `frontend/`: Flutter web application.
- `launch/smart_gui_api.launch`: ROS1 roslaunch file for backend and optional frontend process.
- `smart_gui/random_int8_topics_node.py`: test node that publishes random `std_msgs/Int8` on random topics.

## 📦 Requirements

- ROS Noetic (with `rospy`).
- `catkin` workspace (`catkin_make` or `catkin_tools`).
- Python 3.
- Flutter (only if you want to run the web frontend with `flutter run`).

## 🛠️ Build

From your ROS1 catkin workspace root:

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## ▶️ Run

### 1) Backend + frontend together (recommended for development)

```bash
roslaunch smart_gui smart_gui_api.launch \
  frontend_command:='NO_PROXY=localhost,127.0.0.1 no_proxy=localhost,127.0.0.1 flutter run -d web-server --web-hostname 0.0.0.0 --web-port 3000 --profile'
```

Open:
- Local machine: `http://localhost:3000`
- Another device in LAN: `http://<YOUR_MACHINE_IP>:3000`

### 2) Backend only

```bash
roslaunch smart_gui smart_gui_api.launch run_frontend:=false
```

### 3) Backend only (direct run)

```bash
rosrun smart_gui smart_gui_api --host 0.0.0.0 --port 8000
```

## 🔌 API Endpoints

### Basic
- `GET /health`
- `GET /topics`
- `GET /nodes`
- `GET /services`

### Topic message types/templates
- `GET /topic-message-types`
- `GET /topic-message-template?message_type=<pkg/Type>`

### Topic publish control
- `POST /topic-publisher`
- `POST /topic-publish`
- `POST /topic-publish-loop/start`
- `POST /topic-publish-loop/stop`

### Services
- `GET /service-schema?name=<service>&service_type=<pkg/Type>`
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
    "message_type": "std_msgs/Int32",
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
    "message_type": "std_msgs/Int32"
  }'
```

## 🧪 Random Test Publisher Node

Run a helper node that creates random topic names and publishes random `Int8` values:

```bash
rosrun smart_gui random_int8_topics
```

Optional arguments:

```bash
rosrun smart_gui random_int8_topics --topic-count 5 --hz 5.0
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
