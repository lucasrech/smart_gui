"""ROS Noetic + FastAPI backend used by Smart GUI.

This module preserves the HTTP/WebSocket API used by the Flutter frontend while
using ROS 1 (Noetic) under the hood via `rospy`.
"""

import argparse
import asyncio
import base64
from copy import deepcopy
import io
import re
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import numpy as np
from PIL import Image
from pydantic import BaseModel

import genpy
import rosgraph
import roslib.message
import rosnode
import rospy
import rosservice
import uvicorn


# Curated list shown in frontend topic-creation UI (ROS 1 message type names).
SUPPORTED_TOPIC_MESSAGE_TYPES: Dict[str, List[str]] = {
    "std_msgs": [
        "std_msgs/Bool",
        "std_msgs/Byte",
        "std_msgs/ColorRGBA",
        "std_msgs/Float32",
        "std_msgs/Float64",
        "std_msgs/Int32",
        "std_msgs/Int64",
        "std_msgs/String",
        "std_msgs/UInt32",
        "std_msgs/UInt64",
    ],
    "sensor_msgs": [
        "sensor_msgs/BatteryState",
        "sensor_msgs/FluidPressure",
        "sensor_msgs/Imu",
        "sensor_msgs/Joy",
        "sensor_msgs/LaserScan",
        "sensor_msgs/MagneticField",
        "sensor_msgs/NavSatFix",
        "sensor_msgs/Range",
        "sensor_msgs/Temperature",
    ],
    "geometry_msgs": [
        "geometry_msgs/Pose",
        "geometry_msgs/PoseStamped",
        "geometry_msgs/Quaternion",
        "geometry_msgs/TransformStamped",
        "geometry_msgs/Twist",
        "geometry_msgs/TwistStamped",
        "geometry_msgs/Vector3",
        "geometry_msgs/Vector3Stamped",
        "geometry_msgs/Wrench",
        "geometry_msgs/WrenchStamped",
    ],
}

INTERNAL_TOPICS_HIDDEN_FROM_LIST = {
    "/rosout",
    "/rosout_agg",
}

_ARRAY_TYPE_RE = re.compile(r"^(?P<base>.+)\[(?P<len>\d*)\]$")


def _normalize_ros_type_name(type_name: str) -> str:
    """Normalize message/service type aliases into ROS1 names.

    Accepts canonical ROS1 names (`pkg/Type`) and also normalizes compatible
    alias spellings received from clients.
    """
    return type_name.replace("/msg/", "/").replace("/srv/", "/")


def _ros_time_to_dict(value: Any) -> Dict[str, int]:
    return {
        "sec": int(getattr(value, "secs", getattr(value, "sec", 0))),
        "nanosec": int(getattr(value, "nsecs", getattr(value, "nanosec", 0))),
    }


def _ros_value_to_python(value: Any) -> Any:
    """Convert ROS1 messages/time values into JSON-friendly Python objects."""
    if isinstance(value, genpy.Time):
        return _ros_time_to_dict(value)
    if isinstance(value, genpy.Duration):
        return _ros_time_to_dict(value)
    if isinstance(value, genpy.Message):
        return _ros_message_to_dict(value)
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [_ros_value_to_python(item) for item in value]
    return value


def _ros_message_to_dict(msg: genpy.Message) -> Dict[str, Any]:
    data = {}
    for slot in getattr(msg, "__slots__", []):
        data[slot] = _ros_value_to_python(getattr(msg, slot))

    # Normalize ROS1 /rosout payload to the same shape expected by the frontend
    # log viewer (top-level `stamp` instead of `header.stamp`).
    if getattr(msg, "_type", "") == "rosgraph_msgs/Log":
        header = data.get("header")
        if isinstance(header, dict) and "stamp" in header and "stamp" not in data:
            data["stamp"] = header.get("stamp")
    return data


def _make_time_from_dict(value: Dict[str, Any]) -> genpy.Time:
    sec = value.get("sec", value.get("secs", 0))
    nanosec = value.get("nanosec", value.get("nsecs", 0))
    return rospy.Time(int(sec), int(nanosec))


def _make_duration_from_dict(value: Dict[str, Any]) -> genpy.Duration:
    sec = value.get("sec", value.get("secs", 0))
    nanosec = value.get("nanosec", value.get("nsecs", 0))
    return rospy.Duration(int(sec), int(nanosec))


def _coerce_value_for_slot(slot_type: str, value: Any) -> Any:
    slot_type = _normalize_ros_type_name(slot_type)

    if value is None:
        return value

    array_match = _ARRAY_TYPE_RE.match(slot_type)
    if array_match:
        base_type = array_match.group("base")
        if not isinstance(value, list):
            return value
        return [_coerce_value_for_slot(base_type, item) for item in value]

    if slot_type == "time" and isinstance(value, dict):
        return _make_time_from_dict(value)
    if slot_type == "duration" and isinstance(value, dict):
        return _make_duration_from_dict(value)

    nested_msg_cls = roslib.message.get_message_class(slot_type)
    if nested_msg_cls is not None and isinstance(value, dict):
        nested = nested_msg_cls()
        _set_ros_message_fields(nested, value)
        return nested

    return value


def _set_ros_message_fields(msg: genpy.Message, data: Dict[str, Any]) -> None:
    """Recursively assign JSON payload fields into a ROS1 message instance."""
    slots = list(getattr(msg, "__slots__", []))
    slot_types = list(getattr(msg, "_slot_types", []))
    type_by_slot = dict(zip(slots, slot_types))

    for key, value in data.items():
        if key not in type_by_slot or not hasattr(msg, key):
            continue

        current_value = getattr(msg, key)
        # If current field is a nested ROS message and payload is a dict, recurse.
        if isinstance(current_value, genpy.Message) and isinstance(value, dict):
            _set_ros_message_fields(current_value, value)
            setattr(msg, key, current_value)
            continue

        coerced = _coerce_value_for_slot(type_by_slot[key], value)
        setattr(msg, key, coerced)


class TopicLoopPublisher:
    """Container holding lifecycle primitives for a backend publish-loop worker."""

    def __init__(self, stop_event: threading.Event, thread: threading.Thread) -> None:
        self.stop_event = stop_event
        self.thread = thread


class RosNoeticManager:
    """Own and coordinate ROS1 entities used by the API."""

    def __init__(self) -> None:
        self._initialized = False
        self._master = None  # type: Optional[rosgraph.Master]
        self._lock = threading.Lock()
        self._publisher_lock = threading.Lock()
        self._publishers = {}  # type: Dict[Tuple[str, str], Any]
        self._loop_lock = threading.Lock()
        self._topic_loops = {}  # type: Dict[Tuple[str, str], TopicLoopPublisher]

    def start(self) -> None:
        """Initialize rospy node and master connection once."""
        with self._lock:
            if self._initialized:
                return
            rospy.init_node("smart_gui_api", anonymous=False, disable_signals=True)
            self._master = rosgraph.Master(rospy.get_name())
            self._initialized = True

    def stop(self) -> None:
        """Stop background loops and unregister cached publishers."""
        with self._lock:
            if not self._initialized:
                return
            self.stop_all_topic_publish_loops()
            with self._publisher_lock:
                for pub in self._publishers.values():
                    try:
                        pub.unregister()
                    except Exception:
                        pass
                self._publishers.clear()
            self._initialized = False
            try:
                rospy.signal_shutdown("smart_gui_api shutdown")
            except Exception:
                pass

    @staticmethod
    def normalize_topic_name(topic: str) -> str:
        topic = topic.strip()
        if not topic:
            raise ValueError("topic name cannot be empty")
        return topic if topic.startswith("/") else "/%s" % topic

    @staticmethod
    def _normalize_service_name(name: str) -> str:
        name = name.strip()
        if not name:
            raise ValueError("service name cannot be empty")
        return name if name.startswith("/") else "/%s" % name

    def _topic_types_map(self) -> Dict[str, str]:
        if self._master is None:
            return {}
        try:
            return dict(self._master.getTopicTypes())
        except Exception:
            return {}

    def _topics_with_active_publishers(self) -> Optional[set]:
        """Return topics that currently have at least one publisher registered."""
        if self._master is None:
            return None
        try:
            system_state = self._master.getSystemState()
            publishers = system_state[0] if isinstance(system_state, (list, tuple)) and len(system_state) >= 1 else []
            active = set()
            for entry in publishers:
                if not isinstance(entry, (list, tuple)) or len(entry) < 2:
                    continue
                topic_name, node_names = entry[0], entry[1]
                if topic_name and isinstance(node_names, (list, tuple)) and len(node_names) > 0:
                    active.add(topic_name)
            return active
        except Exception:
            return None

    def list_topics(self, include_hidden_internal: bool = False) -> List[Dict[str, Any]]:
        """Return ROS topics and their advertised message types."""
        topic_types = self._topic_types_map()
        active_publisher_topics = self._topics_with_active_publishers()
        topics = []
        for name in sorted(topic_types.keys()):
            if not include_hidden_internal and name in INTERNAL_TOPICS_HIDDEN_FROM_LIST:
                continue
            if active_publisher_topics is not None and name not in active_publisher_topics:
                continue
            topics.append({"name": name, "types": [topic_types[name]]})
        return topics

    def list_nodes(self) -> List[Dict[str, Any]]:
        """Return ROS1 node names and namespaces currently visible."""
        result = []
        try:
            node_names = rosnode.get_node_names()
        except Exception:
            node_names = []

        for full_name in sorted(node_names):
            normalized = full_name if full_name.startswith("/") else "/%s" % full_name
            parts = [p for p in normalized.split("/") if p]
            if not parts:
                continue
            name = parts[-1]
            namespace = "/" if len(parts) == 1 else "/%s" % "/".join(parts[:-1])
            result.append({"name": name, "namespace": namespace})
        return result

    def list_services(self) -> List[Dict[str, Any]]:
        """Return ROS1 services and their types currently visible."""
        services = []
        try:
            names = rosservice.get_service_list()
        except Exception:
            names = []

        for name in sorted(names):
            srv_type = None
            try:
                srv_type = rosservice.get_service_type(name)
            except Exception:
                srv_type = None
            services.append({"name": name, "types": [srv_type] if srv_type else []})
        return services

    def _get_message_class(self, msg_type: str):
        normalized = _normalize_ros_type_name(msg_type)
        msg_cls = roslib.message.get_message_class(normalized)
        if msg_cls is None:
            raise ValueError("unknown_message_type: %s" % msg_type)
        return normalized, msg_cls

    def _get_service_class(self, service_type: str):
        normalized = _normalize_ros_type_name(service_type)
        srv_cls = roslib.message.get_service_class(normalized)
        if srv_cls is None:
            raise ValueError("unknown_service_type: %s" % service_type)
        return normalized, srv_cls

    def create_subscription(self, topic: str, msg_type: str, callback) -> Any:
        _, msg_cls = self._get_message_class(msg_type)
        return rospy.Subscriber(topic, msg_cls, callback, queue_size=10)

    def destroy_subscription(self, sub: Any) -> None:
        try:
            sub.unregister()
        except Exception:
            pass

    def get_message_template(self, msg_type: str) -> Dict[str, Any]:
        normalized_type, msg_cls = self._get_message_class(msg_type)
        template = _ros_message_to_dict(msg_cls())
        return {"message_type": normalized_type, "message_template": template}

    def create_topic_publisher(
        self,
        topic_name: str,
        msg_type: str,
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        topic_name = self.normalize_topic_name(topic_name)
        normalized_type, msg_cls = self._get_message_class(msg_type)
        key = (topic_name, normalized_type)

        with self._publisher_lock:
            if key in self._publishers:
                created = False
            else:
                pub = rospy.Publisher(topic_name, msg_cls, queue_size=max(1, int(qos_depth)))
                self._publishers[key] = pub
                created = True

        return {
            "name": topic_name,
            "message_type": normalized_type,
            "created": created,
        }

    def publish_topic_message(
        self,
        topic_name: str,
        msg_type: str,
        message_data: Dict[str, Any],
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        topic_name = self.normalize_topic_name(topic_name)
        normalized_type, msg_cls = self._get_message_class(msg_type)
        key = (topic_name, normalized_type)

        with self._publisher_lock:
            publisher = self._publishers.get(key)
            if publisher is None:
                publisher = rospy.Publisher(topic_name, msg_cls, queue_size=max(1, int(qos_depth)))
                self._publishers[key] = publisher

        msg = msg_cls()
        _set_ros_message_fields(msg, message_data)

        # Auto-fill std_msgs/Header timestamp when present.
        if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
            try:
                msg.header.stamp = rospy.Time.now()
            except Exception:
                pass

        publisher.publish(msg)
        return {
            "name": topic_name,
            "message_type": normalized_type,
            "published": _ros_message_to_dict(msg),
        }

    def start_topic_publish_loop(
        self,
        topic_name: str,
        msg_type: str,
        message_data: Dict[str, Any],
        frequency_hz: float,
        qos_depth: int = 10,
    ) -> Dict[str, Any]:
        if frequency_hz <= 0:
            raise ValueError("frequency_hz must be > 0")

        topic_name = self.normalize_topic_name(topic_name)
        normalized_type = _normalize_ros_type_name(msg_type)
        key = (topic_name, normalized_type)
        interval_sec = 1.0 / float(frequency_hz)
        publish_payload = deepcopy(message_data)

        self.create_topic_publisher(topic_name, normalized_type, qos_depth=qos_depth)

        previous_loop = None
        with self._loop_lock:
            previous_loop = self._topic_loops.get(key)
            if previous_loop is not None:
                previous_loop.stop_event.set()

        if previous_loop is not None:
            previous_loop.thread.join(timeout=1.0)

        stop_event = threading.Event()

        def _worker() -> None:
            next_publish = time.monotonic()
            while not stop_event.is_set() and not rospy.is_shutdown():
                try:
                    self.publish_topic_message(
                        topic_name=topic_name,
                        msg_type=normalized_type,
                        message_data=publish_payload,
                        qos_depth=qos_depth,
                    )
                except Exception as err:
                    try:
                        rospy.logerr(
                            "topic_loop_publish_failed topic=%s type=%s: %s",
                            topic_name,
                            normalized_type,
                            err,
                        )
                    except Exception:
                        pass
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
            name="topic_loop_pub:%s:%s" % (topic_name, normalized_type),
        )

        with self._loop_lock:
            self._topic_loops[key] = TopicLoopPublisher(stop_event=stop_event, thread=worker)

        worker.start()
        return {
            "name": topic_name,
            "message_type": normalized_type,
            "running": True,
            "frequency_hz": float(frequency_hz),
            "replaced_previous_loop": previous_loop is not None,
        }

    def stop_topic_publish_loop(self, topic_name: str, msg_type: str) -> Dict[str, Any]:
        topic_name = self.normalize_topic_name(topic_name)
        normalized_type = _normalize_ros_type_name(msg_type)
        key = (topic_name, normalized_type)

        with self._loop_lock:
            loop_state = self._topic_loops.pop(key, None)

        if loop_state is None:
            return {
                "name": topic_name,
                "message_type": normalized_type,
                "running": False,
                "stopped": False,
            }

        loop_state.stop_event.set()
        loop_state.thread.join(timeout=1.0)
        return {
            "name": topic_name,
            "message_type": normalized_type,
            "running": False,
            "stopped": True,
        }

    def stop_all_topic_publish_loops(self) -> None:
        with self._loop_lock:
            loops = list(self._topic_loops.values())
            self._topic_loops.clear()

        for loop_state in loops:
            loop_state.stop_event.set()
        for loop_state in loops:
            loop_state.thread.join(timeout=1.0)

    def get_service_schema(self, service_name: str, service_type: str) -> Dict[str, Any]:
        normalized_type, srv_cls = self._get_service_class(service_type)
        request_template = _ros_message_to_dict(srv_cls._request_class())
        response_template = _ros_message_to_dict(srv_cls._response_class())
        return {
            "name": self._normalize_service_name(service_name),
            "type": normalized_type,
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
        service_name = self._normalize_service_name(service_name)
        normalized_type, srv_cls = self._get_service_class(service_type)

        rospy.wait_for_service(service_name, timeout=timeout_sec)
        proxy = rospy.ServiceProxy(service_name, srv_cls, persistent=False)

        request = srv_cls._request_class()
        _set_ros_message_fields(request, request_data)

        result = {}  # type: Dict[str, Any]
        error = {}  # type: Dict[str, Exception]

        def _invoke() -> None:
            try:
                response = proxy.call(request)
                result["response"] = _ros_message_to_dict(response)
            except Exception as exc:
                error["error"] = exc

        thread = threading.Thread(target=_invoke, daemon=True)
        thread.start()
        thread.join(timeout=timeout_sec)

        try:
            close_fn = getattr(proxy, "close", None)
            if callable(close_fn):
                close_fn()
        except Exception:
            pass

        if thread.is_alive():
            raise TimeoutError("service_timeout")
        if "error" in error:
            raise RuntimeError(str(error["error"]))
        if "response" not in result:
            raise RuntimeError("service_call_failed")
        return result["response"]


app = FastAPI(title="ROS Noetic Inspector API", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros2 = RosNoeticManager()  # Kept variable name for minimal diff in endpoint code.


class ServiceCallRequest(BaseModel):
    name: str
    service_type: str
    request: Dict[str, Any]
    timeout_sec: float = 3.0


class TopicPublisherRequest(BaseModel):
    name: str
    message_type: str
    qos_depth: int = 10


class TopicPublishRequest(BaseModel):
    name: str
    message_type: str
    message: Dict[str, Any]
    qos_depth: int = 10


class TopicPublishLoopStartRequest(BaseModel):
    name: str
    message_type: str
    message: Dict[str, Any]
    frequency_hz: float = 1.0
    qos_depth: int = 10


class TopicPublishLoopStopRequest(BaseModel):
    name: str
    message_type: str


def _compress_ros_image_to_jpeg(
    msg: Any,
    quality: int = 70,
    max_width: int = 1280,
) -> Tuple[bytes, Dict[str, Any]]:
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
        raise ValueError("unsupported image encoding: %s" % msg.encoding)

    pil_img = Image.fromarray(rgb, mode="RGB")
    if max_width > 0 and pil_img.width > max_width:
        new_h = max(1, int(pil_img.height * (float(max_width) / float(pil_img.width))))
        pil_img = pil_img.resize((max_width, new_h), Image.Resampling.LANCZOS)

    out = io.BytesIO()
    pil_img.save(out, format="JPEG", quality=quality, optimize=True)
    jpeg = out.getvalue()

    meta = {
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": str(msg.encoding),
        "step": int(msg.step),
        "source_size": len(raw),
        "compressed_format": "jpeg",
        "compressed_size": len(jpeg),
        "jpeg_quality": int(quality),
        "max_width": int(max_width),
        "output_width": int(pil_img.width),
        "output_height": int(pil_img.height),
    }
    return jpeg, meta


@app.on_event("startup")
def on_startup() -> None:
    ros2.start()


@app.on_event("shutdown")
def on_shutdown() -> None:
    ros2.stop()


@app.get("/health")
def health() -> Dict[str, str]:
    return {"status": "ok"}


@app.get("/")
def root() -> Dict[str, str]:
    return {
        "name": "ROS Noetic Inspector API",
        "status": "ok",
        "frontend_hint": "Open the Flutter UI on port 3000 (not this API port).",
    }


@app.get("/favicon.ico")
def favicon() -> Response:
    return Response(status_code=204)


@app.get("/topics")
def topics() -> List[Dict[str, Any]]:
    return ros2.list_topics()


@app.get("/topic-message-types")
def topic_message_types() -> Dict[str, List[str]]:
    return SUPPORTED_TOPIC_MESSAGE_TYPES


@app.get("/topic-message-template")
def topic_message_template(message_type: str) -> Dict[str, Any]:
    try:
        return ros2.get_message_template(message_type)
    except Exception as err:
        raise HTTPException(status_code=400, detail="invalid_message_type: %s" % err)


@app.post("/topic-publisher")
def topic_publisher(payload: TopicPublisherRequest) -> Dict[str, Any]:
    try:
        return ros2.create_topic_publisher(
            topic_name=payload.name,
            msg_type=payload.message_type,
            qos_depth=payload.qos_depth,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err))


@app.post("/topic-publish")
def topic_publish(payload: TopicPublishRequest) -> Dict[str, Any]:
    try:
        result = ros2.publish_topic_message(
            topic_name=payload.name,
            msg_type=payload.message_type,
            message_data=payload.message,
            qos_depth=payload.qos_depth,
        )
        return dict(ok=True, **result)
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err))


@app.post("/topic-publish-loop/start")
def topic_publish_loop_start(payload: TopicPublishLoopStartRequest) -> Dict[str, Any]:
    try:
        return ros2.start_topic_publish_loop(
            topic_name=payload.name,
            msg_type=payload.message_type,
            message_data=payload.message,
            frequency_hz=payload.frequency_hz,
            qos_depth=payload.qos_depth,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err))


@app.post("/topic-publish-loop/stop")
def topic_publish_loop_stop(payload: TopicPublishLoopStopRequest) -> Dict[str, Any]:
    try:
        return ros2.stop_topic_publish_loop(
            topic_name=payload.name,
            msg_type=payload.message_type,
        )
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err))


@app.get("/nodes")
def nodes() -> List[Dict[str, Any]]:
    return ros2.list_nodes()


@app.get("/services")
def services() -> List[Dict[str, Any]]:
    return ros2.list_services()


@app.get("/service-schema")
def service_schema(name: str, service_type: str) -> Dict[str, Any]:
    try:
        return ros2.get_service_schema(name, service_type)
    except Exception as err:
        raise HTTPException(status_code=400, detail="invalid_service_type: %s" % err)


@app.post("/service-call")
def service_call(payload: ServiceCallRequest) -> Dict[str, Any]:
    try:
        response = ros2.call_service(
            service_name=payload.name,
            service_type=payload.service_type,
            request_data=payload.request,
            timeout_sec=payload.timeout_sec,
        )
        return {"ok": True, "response": response}
    except TimeoutError as err:
        raise HTTPException(status_code=504, detail=str(err))
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err))


@app.websocket("/ws/topics/{topic:path}")
async def topic_ws(websocket: WebSocket, topic: str) -> None:
    await websocket.accept()

    topics_list = ros2.list_topics(include_hidden_internal=True)
    match = next((t for t in topics_list if t["name"] == "/%s" % topic or t["name"] == topic), None)
    if match is None:
        await websocket.send_json({"error": "topic_not_found", "topic": topic})
        await websocket.close()
        return

    msg_type = match["types"][0] if match.get("types") else None
    if msg_type is None:
        await websocket.send_json({"error": "topic_has_no_type", "topic": topic})
        await websocket.close()
        return

    loop = asyncio.get_running_loop()
    queue = asyncio.Queue(maxsize=100)  # type: asyncio.Queue
    normalized_type = _normalize_ros_type_name(msg_type)
    is_image_topic = normalized_type == "sensor_msgs/Image"
    image_max_fps = 5.0
    min_image_period = 1.0 / image_max_fps
    last_image_emit = 0.0

    def _enqueue(payload: Dict[str, Any]) -> None:
        try:
            queue.put_nowait(payload)
        except asyncio.QueueFull:
            pass

    def _callback(msg: Any) -> None:
        nonlocal last_image_emit
        now = time.time()

        if is_image_topic:
            if (now - last_image_emit) < min_image_period:
                return
            last_image_emit = now

            try:
                jpeg_bytes, img_meta = _compress_ros_image_to_jpeg(msg)
            except Exception as err:
                loop.call_soon_threadsafe(
                    _enqueue,
                    {
                        "topic": match["name"],
                        "type": msg_type,
                        "timestamp": now,
                        "error": "image_encode_failed",
                        "details": str(err),
                    },
                )
                return

            header_stamp = getattr(getattr(msg, "header", None), "stamp", None)
            stamp_dict = _ros_time_to_dict(header_stamp) if header_stamp is not None else {"sec": 0, "nanosec": 0}
            frame_id = getattr(getattr(msg, "header", None), "frame_id", "")
            payload = {
                "topic": match["name"],
                "type": msg_type,
                "timestamp": now,
                "message": {
                    "header": {
                        "stamp": stamp_dict,
                        "frame_id": frame_id,
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
                "message": _ros_message_to_dict(msg),
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
    parser = argparse.ArgumentParser(description="Run Smart GUI ROS Noetic Inspector API")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args, _ = parser.parse_known_args()
    uvicorn.run(app, host=args.host, port=args.port, ws="wsproto")


if __name__ == "__main__":
    main()
