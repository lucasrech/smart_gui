import argparse
import asyncio
import base64
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


class Ros2Manager:
    def __init__(self) -> None:
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._lock = threading.Lock()

    def start(self) -> None:
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
        with self._lock:
            if self._node is None:
                return
            self._executor.shutdown()
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            self._node = None
            rclpy.shutdown()

    def list_topics(self) -> List[Dict[str, Any]]:
        topics = self._node.get_topic_names_and_types()
        return [{"name": name, "types": types} for name, types in topics]

    def list_nodes(self) -> List[Dict[str, Any]]:
        nodes = self._node.get_node_names_and_namespaces()
        return [{"name": name, "namespace": ns} for name, ns in nodes]

    def list_services(self) -> List[Dict[str, Any]]:
        services = self._node.get_service_names_and_types()
        return [{"name": name, "types": types} for name, types in services]

    def create_subscription(self, topic: str, msg_type: str, callback) -> Any:
        msg_cls = get_message(msg_type)
        qos = QoSProfile(depth=10)
        return self._node.create_subscription(msg_cls, topic, callback, qos)

    def destroy_subscription(self, sub) -> None:
        self._node.destroy_subscription(sub)

    def get_service_schema(self, service_name: str, service_type: str) -> Dict[str, Any]:
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
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ros2 = Ros2Manager()


class ServiceCallRequest(BaseModel):
    name: str
    service_type: str
    request: Dict[str, Any]
    timeout_sec: float = 3.0


def _compress_ros_image_to_jpeg(
    msg: Any,
    *,
    quality: int = 70,
    max_width: int = 1280,
) -> tuple[bytes, Dict[str, Any]]:
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
    return ros2.list_topics()


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
        raise HTTPException(status_code=400, detail=f"invalid_service_type: {err}") from err


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
        raise HTTPException(status_code=504, detail=str(err)) from err
    except Exception as err:
        raise HTTPException(status_code=400, detail=str(err)) from err


@app.websocket("/ws/topics/{topic:path}")
async def topic_ws(websocket: WebSocket, topic: str) -> None:
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
        try:
            queue.put_nowait(payload)
        except asyncio.QueueFull:
            pass

    def _callback(msg) -> None:
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
    parser = argparse.ArgumentParser(description="Run Smart GUI ROS2 Inspector API")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    # Ignore ROS launch arguments (e.g. --ros-args) injected by launch_ros Node.
    args, _ = parser.parse_known_args()

    uvicorn.run(app, host=args.host, port=args.port, ws="wsproto")


if __name__ == "__main__":
    main()
