"""ROS 2 + FastAPI backend used by Smart GUI.

This module exposes HTTP and WebSocket endpoints to inspect and interact with a
running ROS 2 graph. It also includes topic publishing utilities, including
backend-managed publish loops to avoid per-message frontend HTTP traffic.
"""

import argparse
import asyncio
import base64
from copy import deepcopy
import io
import threading
import time
from typing import Any, Dict, List

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import numpy as np
from PIL import Image
from pydantic import BaseModel

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service
import uvicorn


# Curated list shown in frontend topic-creation UI by package namespace.
SUPPORTED_TOPIC_MESSAGE_TYPES: Dict[str, List[str]] = {
    "std_msgs": [
        "std_msgs/msg/Bool",
        "std_msgs/msg/Byte",
        "std_msgs/msg/ColorRGBA",
        "std_msgs/msg/Float32",
        "std_msgs/msg/Float64",
        "std_msgs/msg/Int32",
        "std_msgs/msg/Int64",
        "std_msgs/msg/String",
        "std_msgs/msg/UInt32",
        "std_msgs/msg/UInt64",
    ],
    "sensor_msgs": [
        "sensor_msgs/msg/BatteryState",
        "sensor_msgs/msg/FluidPressure",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/Joy",
        "sensor_msgs/msg/LaserScan",
        "sensor_msgs/msg/MagneticField",
        "sensor_msgs/msg/NavSatFix",
        "sensor_msgs/msg/Range",
        "sensor_msgs/msg/Temperature",
    ],
    "geometry_msgs": [
        "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/Vector3",
        "geometry_msgs/msg/Vector3Stamped",
        "geometry_msgs/msg/Wrench",
        "geometry_msgs/msg/WrenchStamped",
    ],
}


class TopicLoopPublisher:
    """Container holding lifecycle primitives for a backend publish-loop worker."""

    def __init__(self, stop_event: threading.Event, thread: threading.Thread) -> None:
        self.stop_event = stop_event
        self.thread = thread


class Ros2Manager:
    """Own and coordinate ROS entities used by the API (node, pubs, subs, loops)."""

    def __init__(self) -> None:
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._lock = threading.Lock()
        self._publisher_lock = threading.Lock()
        self._publishers: Dict[tuple[str, str], Any] = {}
        self._loop_lock = threading.Lock()
        self._topic_loops: Dict[tuple[str, str], TopicLoopPublisher] = {}

    def start(self) -> None:
        """Initialize ROS context/node and start executor spinning thread once."""
        with self._lock:
            if self._node is not None:
                return
            rclpy.init()
            self._node = rclpy.create_node("ros2_inspector_api")
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._spin_thread.start()

    def stop(self) -> None:
        """Stop loops, destroy ROS entities, and shutdown ROS context."""
        with self._lock:
            if self._node is None:
                return
            self.stop_all_topic_publish_loops()
            with self._publisher_lock:
                for publisher in self._publishers.values():
                    self._node.destroy_publisher(publisher)
                self._publishers.clear()
            self._executor.shutdown()
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            self._node = None
            rclpy.shutdown()

    @staticmethod
    def normalize_topic_name(topic: str) -> str:
        """Return a canonical topic name with leading slash and basic validation."""
        topic = topic.strip()
        if not topic:
            raise ValueError("topic name cannot be empty")
        return topic if topic.startswith("/") else f"/{topic}"

    def list_topics(self) -> List[Dict[str, Any]]:
        """Return ROS topics and their advertised message types."""
        topics = self._node.get_topic_names_and_types()
        return [{"name": name, "types": types} for name, types in topics]

    def list_nodes(self) -> List[Dict[str, Any]]:
        """Return ROS node names and namespaces currently visible."""
        nodes = self._node.get_node_names_and_namespaces()
        return [{"name": name, "namespace": ns} for name, ns in nodes]

    def list_services(self) -> List[Dict[str, Any]]:
        """Return ROS services and their types currently visible."""
        services = self._node.get_service_names_and_types()
        return [{"name": name, "types": types} for name, types in services]

    def create_subscription(self, topic: str, msg_type: str, callback) -> Any:
        """Create a ROS subscription with a default QoS depth of 10."""
        msg_cls = get_message(msg_type)
        qos = QoSProfile(depth=10)
        return self._node.create_subscription(msg_cls, topic, callback, qos)

    def destroy_subscription(self, sub) -> None:
        """Destroy a previously created ROS subscription."""
        self._node.destroy_subscription(sub)

    def get_message_template(self, msg_type: str) -> Dict[str, Any]:
        """Build a JSON-friendly template for a ROS message type instance."""
        msg_cls = get_message(msg_type)
        template = message_to_ordereddict(msg_cls())
        return {
            "message_type": msg_type,
            "message_template": template,
        }

    def create_topic_publisher(
        self,
        topic_name: str,
        msg_type: str,
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        """Create/reuse a publisher for topic/type and return creation status."""
        topic_name = self.normalize_topic_name(topic_name)
        msg_cls = get_message(msg_type)
        key = (topic_name, msg_type)
        with self._publisher_lock:
            if key in self._publishers:
                created = False
            else:
                qos = QoSProfile(depth=max(1, int(qos_depth)))
                publisher = self._node.create_publisher(msg_cls, topic_name, qos)
                self._publishers[key] = publisher
                created = True
        return {
            "name": topic_name,
            "message_type": msg_type,
            "created": created,
        }

    def publish_topic_message(
        self,
        topic_name: str,
        msg_type: str,
        message_data: Dict[str, Any],
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        """Publish one message on a topic/type, auto-filling header timestamp if present."""
        topic_name = self.normalize_topic_name(topic_name)
        msg_cls = get_message(msg_type)
        key = (topic_name, msg_type)

        with self._publisher_lock:
            publisher = self._publishers.get(key)
            if publisher is None:
                qos = QoSProfile(depth=max(1, int(qos_depth)))
                publisher = self._node.create_publisher(msg_cls, topic_name, qos)
                self._publishers[key] = publisher

        msg = msg_cls()
        set_message_fields(msg, message_data)

        # If message contains a std_msgs/Header, always fill timestamp from ROS clock.
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            now = self._node.get_clock().now().to_msg()
            msg.header.stamp.sec = now.sec
            msg.header.stamp.nanosec = now.nanosec

        publisher.publish(msg)
        return {
            "name": topic_name,
            "message_type": msg_type,
            "published": message_to_ordereddict(msg),
        }

    def start_topic_publish_loop(
        self,
        topic_name: str,
        msg_type: str,
        message_data: Dict[str, Any],
        frequency_hz: float,
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        """Start (or replace) a backend thread that publishes at a fixed frequency."""
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be > 0")

        topic_name = self.normalize_topic_name(topic_name)
        key = (topic_name, msg_type)
        interval_sec = 1.0 / float(frequency_hz)
        publish_payload = deepcopy(message_data)

        # Validate topic type and ensure publisher exists before background loop starts.
        self.create_topic_publisher(topic_name, msg_type, qos_depth=qos_depth)

        previous_loop = None
        with self._loop_lock:
            previous_loop = self._topic_loops.get(key)
            if previous_loop is not None:
                previous_loop.stop_event.set()

        if previous_loop is not None:
            previous_loop.thread.join(timeout=1.0)

        stop_event = threading.Event()

        def _worker() -> None:
            """Periodic publisher worker running until stop_event is set."""
            next_publish = time.monotonic()
            while not stop_event.is_set():
                try:
                    self.publish_topic_message(
                        topic_name=topic_name,
                        msg_type=msg_type,
                        message_data=publish_payload,
                        qos_depth=qos_depth,
                    )
                except Exception as err:
                    if self._node is not None:
                        self._node.get_logger().error(
                            f"topic_loop_publish_failed topic={topic_name} type={msg_type}: {err}"
                        )
                    break

                next_publish += interval_sec
                wait_sec = max(0.0, next_publish - time.monotonic())
                if stop_event.wait(wait_sec):
                    break

            with self._loop_lock:
                current = self._topic_loops.get(key)
                if current is not None and current.stop_event is stop_event:
                    self._topic_loops.pop(key, None)

        worker = threading.Thread(
            target=_worker,
            daemon=True,
            name=f"topic_loop_pub:{topic_name}:{msg_type}",
        )

        with self._loop_lock:
            self._topic_loops[key] = TopicLoopPublisher(stop_event=stop_event, thread=worker)

        worker.start()
        return {
            "name": topic_name,
            "message_type": msg_type,
            "running": True,
            "frequency_hz": float(frequency_hz),
            "replaced_previous_loop": previous_loop is not None,
        }

    def stop_topic_publish_loop(self, topic_name: str, msg_type: str) -> Dict[str, Any]:
        """Stop a running backend publish loop for topic/type if present."""
        topic_name = self.normalize_topic_name(topic_name)
        key = (topic_name, msg_type)

        with self._loop_lock:
            loop_state = self._topic_loops.pop(key, None)

        if loop_state is None:
            return {
                "name": topic_name,
                "message_type": msg_type,
                "running": False,
                "stopped": False,
            }

        loop_state.stop_event.set()
        loop_state.thread.join(timeout=1.0)
        return {
            "name": topic_name,
            "message_type": msg_type,
            "running": False,
            "stopped": True,
        }

    def stop_all_topic_publish_loops(self) -> None:
        """Stop all currently running backend publish loops."""
        with self._loop_lock:
            loops = list(self._topic_loops.values())
            self._topic_loops.clear()

        for loop_state in loops:
            loop_state.stop_event.set()
        for loop_state in loops:
            loop_state.thread.join(timeout=1.0)

    def get_service_schema(self, service_name: str, service_type: str) -> Dict[str, Any]:
        """Return request/response templates for a ROS service type."""
        srv_cls = get_service(service_type)
        request_template = message_to_ordereddict(srv_cls.Request())
        response_template = message_to_ordereddict(srv_cls.Response())
        return {
            "name": service_name,
            "type": service_type,
            "request_template": request_template,
            "response_template": response_template,
        }

    def call_service(
        self,
        service_name: str,
        service_type: str,
        request_data: Dict[str, Any],
        timeout_sec: float = 3.0,
    ) -> Dict[str, Any]:
        """Call a ROS service synchronously with timeout and JSON payload mapping."""
        srv_cls = get_service(service_type)
        client = self._node.create_client(srv_cls, service_name)

        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError("service_unavailable")

            request = srv_cls.Request()
            set_message_fields(request, request_data)

            future = client.call_async(request)
            deadline = time.time() + timeout_sec
            while not future.done():
                if time.time() > deadline:
                    raise TimeoutError("service_timeout")
                time.sleep(0.01)

            exc = future.exception()
            if exc is not None:
                raise RuntimeError(str(exc))

            response = future.result()
            return message_to_ordereddict(response)
        finally:
            self._node.destroy_client(client)


app = FastAPI(title="ROS2 Inspector API", version="0.1.0")
# Allow frontend access from different hosts/ports during development and LAN usage.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros2 = Ros2Manager()


class ServiceCallRequest(BaseModel):
    """HTTP payload model for invoking a ROS service."""

    name: str
    service_type: str
    request: Dict[str, Any]
    timeout_sec: float = 3.0


class TopicPublisherRequest(BaseModel):
    """HTTP payload model for creating/reusing a topic publisher."""

    name: str
    message_type: str
    qos_depth: int = 10


class TopicPublishRequest(BaseModel):
    """HTTP payload model for publishing a single topic message."""

    name: str
    message_type: str
    message: Dict[str, Any]
    qos_depth: int = 10


class TopicPublishLoopStartRequest(BaseModel):
    """HTTP payload model for starting backend-managed loop publishing."""

    name: str
    message_type: str
    message: Dict[str, Any]
    frequency_hz: float = 1.0
    qos_depth: int = 10


class TopicPublishLoopStopRequest(BaseModel):
    """HTTP payload model for stopping backend-managed loop publishing."""

    name: str
    message_type: str


def _compress_ros_image_to_jpeg(
    msg: Any,
    *,
    quality: int = 70,
    max_width: int = 1280,
) -> tuple[bytes, Dict[str, Any]]:
    """Convert `sensor_msgs/msg/Image` payload to JPEG bytes plus metadata."""
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()
    raw = bytes(msg.data)

    def _reshape_rows(expected_row_bytes: int) -> np.ndarray:
        arr = np.frombuffer(raw, dtype=np.uint8)
        if len(raw) < height * step:
            raise ValueError("image payload smaller than expected")
        rows = arr[: height * step].reshape((height, step))
        return rows[:, :expected_row_bytes]

    if encoding == "rgb8":
        rows = _reshape_rows(width * 3)
        rgb = rows.reshape((height, width, 3))
    elif encoding == "bgr8":
        rows = _reshape_rows(width * 3)
        rgb = rows.reshape((height, width, 3))[:, :, ::-1]
    elif encoding == "rgba8":
        rows = _reshape_rows(width * 4)
        rgba = rows.reshape((height, width, 4))
        rgb = rgba[:, :, :3]
    elif encoding == "bgra8":
        rows = _reshape_rows(width * 4)
        bgra = rows.reshape((height, width, 4))
        rgb = bgra[:, :, [2, 1, 0]]
    elif encoding == "mono8":
        rows = _reshape_rows(width)
        mono = rows.reshape((height, width))
        rgb = np.stack([mono, mono, mono], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    pil_img = Image.fromarray(rgb, mode="RGB")

    if max_width > 0 and pil_img.width > max_width:
        new_h = max(1, int(pil_img.height * (max_width / pil_img.width)))
        pil_img = pil_img.resize((max_width, new_h), Image.Resampling.LANCZOS)

    out = io.BytesIO()
    pil_img.save(out, format="JPEG", quality=quality, optimize=True)
    jpeg = out.getvalue()

    meta = {
        "height": msg.height,
        "width": msg.width,
        "encoding": msg.encoding,
        "step": msg.step,
        "source_size": len(raw),
        "compressed_format": "jpeg",
        "compressed_size": len(jpeg),
        "jpeg_quality": quality,
        "max_width": max_width,
        "output_width": pil_img.width,
        "output_height": pil_img.height,
    }
    return jpeg, meta


@app.on_event("startup")
def on_startup() -> None:
    """FastAPI startup hook to initialize ROS resources."""
    ros2.start()


@app.on_event("shutdown")
def on_shutdown() -> None:
    """FastAPI shutdown hook to release ROS resources."""
    ros2.stop()


@app.get("/health")
def health() -> Dict[str, str]:
    """Simple health probe endpoint."""
    return {"status": "ok"}


@app.get("/")
def root() -> Dict[str, str]:
    """Human-readable root endpoint with quick usage hint."""
    return {
        "name": "ROS2 Inspector API",
        "status": "ok",
        "frontend_hint": "Open the Flutter UI on port 3000 (not this API port).",
    }


@app.get("/favicon.ico")
def favicon() -> Response:
    # Avoid noisy 404 logs when browsers probe /favicon.ico on the API host.
    return Response(status_code=204)


@app.get("/topics")
def topics() -> List[Dict[str, Any]]:
    """List ROS topics and their message types."""
    return ros2.list_topics()


@app.get("/topic-message-types")
def topic_message_types() -> Dict[str, List[str]]:
    """Return curated message types grouped by package namespace."""
    return SUPPORTED_TOPIC_MESSAGE_TYPES


@app.get("/topic-message-template")
def topic_message_template(message_type: str) -> Dict[str, Any]:
    """Return JSON template for a message type selected by the frontend."""
    try:
        return ros2.get_message_template(message_type)
    except Exception as err:
        raise HTTPException(status_code=400, detail=f"invalid_message_type: {err}") from err


@app.post("/topic-publisher")
def topic_publisher(payload: TopicPublisherRequest) -> Dict[str, Any]:
    """Create/reuse a publisher for topic/type."""
    try:
        return ros2.create_topic_publisher(
            topic_name=payload.name,
            msg_type=payload.message_type,
            qos_depth=payload.qos_depth,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.post("/topic-publish")
def topic_publish(payload: TopicPublishRequest) -> Dict[str, Any]:
    """Publish one message on a ROS topic."""
    try:
        result = ros2.publish_topic_message(
            topic_name=payload.name,
            msg_type=payload.message_type,
            message_data=payload.message,
            qos_depth=payload.qos_depth,
        )
        return {"ok": True, **result}
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.post("/topic-publish-loop/start")
def topic_publish_loop_start(payload: TopicPublishLoopStartRequest) -> Dict[str, Any]:
    """Start backend-managed periodic publishing for a topic."""
    try:
        return ros2.start_topic_publish_loop(
            topic_name=payload.name,
            msg_type=payload.message_type,
            message_data=payload.message,
            frequency_hz=payload.frequency_hz,
            qos_depth=payload.qos_depth,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.post("/topic-publish-loop/stop")
def topic_publish_loop_stop(payload: TopicPublishLoopStopRequest) -> Dict[str, Any]:
    """Stop backend-managed periodic publishing for a topic."""
    try:
        return ros2.stop_topic_publish_loop(
            topic_name=payload.name,
            msg_type=payload.message_type,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.get("/nodes")
def nodes() -> List[Dict[str, Any]]:
    """List ROS nodes."""
    return ros2.list_nodes()


@app.get("/services")
def services() -> List[Dict[str, Any]]:
    """List ROS services and types."""
    return ros2.list_services()


@app.get("/service-schema")
def service_schema(name: str, service_type: str) -> Dict[str, Any]:
    """Return request/response templates for a service endpoint."""
    try:
        return ros2.get_service_schema(name, service_type)
    except Exception as err:
        raise HTTPException(status_code=400, detail=f"invalid_service_type: {err}") from err


@app.post("/service-call")
def service_call(payload: ServiceCallRequest) -> Dict[str, Any]:
    """Call a ROS service with JSON payload."""
    try:
        response = ros2.call_service(
            service_name=payload.name,
            service_type=payload.service_type,
            request_data=payload.request,
            timeout_sec=payload.timeout_sec,
        )
        return {"ok": True, "response": response}
    except TimeoutError as err:
        raise HTTPException(status_code=504, detail=str(err)) from err
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.websocket("/ws/topics/{topic:path}")
async def topic_ws(websocket: WebSocket, topic: str) -> None:
    """Stream messages from a selected ROS topic over WebSocket."""
    await websocket.accept()

    topics = ros2.list_topics()
    match = next((t for t in topics if t["name"] == f"/{topic}" or t["name"] == topic), None)
    if match is None:
        await websocket.send_json({"error": "topic_not_found", "topic": topic})
        await websocket.close()
        return

    msg_type = match["types"][0] if match["types"] else None
    if msg_type is None:
        await websocket.send_json({"error": "topic_has_no_type", "topic": topic})
        await websocket.close()
        return

    loop = asyncio.get_running_loop()
    queue: asyncio.Queue = asyncio.Queue(maxsize=100)
    is_image_topic = msg_type == "sensor_msgs/msg/Image"
    image_max_fps = 5.0
    min_image_period = 1.0 / image_max_fps
    last_image_emit = 0.0

    def _enqueue(payload: Dict[str, Any]) -> None:
        """Drop-old behavior: silently ignore messages when client queue is full."""
        try:
            queue.put_nowait(payload)
        except asyncio.QueueFull:
            pass

    def _callback(msg) -> None:
        """ROS subscription callback that serializes and forwards message payloads."""
        nonlocal last_image_emit
        now = time.time()

        if is_image_topic:
            if (now - last_image_emit) < min_image_period:
                return
            last_image_emit = now

            try:
                jpeg_bytes, img_meta = _compress_ros_image_to_jpeg(msg)
            except Exception as err:
                payload = {
                    "topic": match["name"],
                    "type": msg_type,
                    "timestamp": now,
                    "error": "image_encode_failed",
                    "details": str(err),
                }
                loop.call_soon_threadsafe(_enqueue, payload)
                return

            payload = {
                "topic": match["name"],
                "type": msg_type,
                "timestamp": now,
                "message": {
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec,
                        },
                        "frame_id": msg.header.frame_id,
                    },
                    **img_meta,
                },
                "image_b64": base64.b64encode(jpeg_bytes).decode("ascii"),
            }
        else:
            payload = {
                "topic": match["name"],
                "type": msg_type,
                "timestamp": now,
                "message": message_to_ordereddict(msg),
            }
        loop.call_soon_threadsafe(_enqueue, payload)

    sub = ros2.create_subscription(match["name"], msg_type, _callback)

    try:
        await websocket.send_json({"topic": match["name"], "type": msg_type})
        while True:
            data = await queue.get()
            await websocket.send_json(data)
    except WebSocketDisconnect:
        pass
    finally:
        ros2.destroy_subscription(sub)


def main() -> None:
    """CLI entrypoint used by `ros2 run smart_gui smart_gui_api`."""
    parser = argparse.ArgumentParser(description="Run Smart GUI ROS2 Inspector API")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    # Ignore ROS launch arguments (e.g. --ros-args) injected by launch_ros Node.
    args, _ = parser.parse_known_args()

    uvicorn.run(app, host=args.host, port=args.port, ws="wsproto")


if __name__ == "__main__":
    main()
