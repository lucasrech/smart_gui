# ROS Noetic Inspector API (FastAPI)

## Requisitos
- ROS Noetic instalado e `source /opt/ros/noetic/setup.bash`
- Python 3.x
- Pacotes Python do ROS1 disponíveis no ambiente (inclui `rospy`)

## Instalação
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Executar
```bash
source /opt/ros/noetic/setup.bash
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## Executar como pacote ROS 1 (Noetic)
No workspace ROS (`.../src/smart_gui`):

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rosrun smart_gui smart_gui_api --host 0.0.0.0 --port 8000
```

Ou com launch:

```bash
roslaunch smart_gui smart_gui_api.launch host:=0.0.0.0 port:=8000
```

## Endpoints
- `GET /topics`
- `GET /nodes`
- `GET /services`
- `WS /ws/topics/{topic}`

## Observações de rede (ROS1 em outra máquina)
- Configure `ROS_MASTER_URI` e `ROS_IP`/`ROS_HOSTNAME` corretamente.
- Garanta conectividade TCP entre as máquinas (firewall/roteamento).
