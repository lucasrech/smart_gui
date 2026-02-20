# ROS2 Inspector API (FastAPI)

## Requisitos
- ROS2 Jazzy instalado e `source /opt/ros/jazzy/setup.bash`
- Python 3.x
- Pacotes Python do ROS2 disponíveis no ambiente (inclui `rclpy`)

## Instalação
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Executar
```bash
source /opt/ros/jazzy/setup.bash
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Executar como pacote ROS 2
No workspace ROS (`.../src/smart_gui`):

```bash
cd ~/lacbot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select smart_gui
source install/setup.bash
ros2 run smart_gui smart_gui_api --host 0.0.0.0 --port 8000
```

Ou com launch:

```bash
ros2 launch smart_gui smart_gui_api.launch.py host:=0.0.0.0 port:=8000
```

## Endpoints
- `GET /topics`
- `GET /nodes`
- `GET /services`
- `WS /ws/topics/{topic}`

## Observações de rede (ROS2 em outra máquina)
- Configure o mesmo `ROS_DOMAIN_ID` nas duas máquinas.
- Garanta que o DDS consiga comunicar entre as máquinas (firewall e multicast).
- Não use `ROS_LOCALHOST_ONLY=1`.
- Se necessário, ajuste o `FASTRTPS_DEFAULT_PROFILES_FILE` ou o DDS que você usa.
