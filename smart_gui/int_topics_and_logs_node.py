"""Simple ROS 1 (Noetic) test node that publishes integer topics and emits logs.

This helper node is intended for UI testing:
- Publishes on four `std_msgs/Int32` topics.
- Publishes video frames on one `sensor_msgs/Image` topic.
- Emits log messages at debug/info/warning/error levels on a timer.
"""

import argparse
from pathlib import Path
import random
import tempfile
import threading
from typing import Any, List
from urllib.parse import urlparse

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class IntTopicsAndLogsNode:
    """Publish integers on multiple topics and generate ROS logs periodically."""

    VIDEO_SAMPLE_URLS = [
        "https://samplelib.com/lib/preview/mp4/sample-5s.mp4",
        "https://samplelib.com/lib/preview/mp4/sample-10s.mp4",
        "https://samplelib.com/lib/preview/mp4/sample-15s.mp4",
    ]

    def __init__(
        self,
        publish_hz: float = 5.0,
        log_hz: float = 1.0,
        image_hz: float = 5.0,
    ) -> None:
        """Create publishers and timers

        Args:
            publish_hz: Integer topic publish frequency in Hz.
            log_hz: Log emission frequency in Hz.
            image_hz: Image topic publish frequency in Hz.
        """
        self._publishers: List[Any] = []
        self._topic_names: List[str] = []
        self._counter = 0
        self._image_topic_name = "/test/camera/image"
        self._image_publisher = rospy.Publisher(self._image_topic_name, Image, queue_size=2)
        self._video_frame_lock = threading.Lock()
        self._video_frames: List[np.ndarray] = []
        self._video_frame_index = 0

        topic_names = [
            "/test/int_topic_1",
            "/test/int_topic_2",
            "/test/int_topic_3",
            "/test/int_topic_4",
        ]
        for topic_name in topic_names:
            publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
            self._publishers.append(publisher)
            self._topic_names.append(topic_name)

        publish_period = 1.0 / publish_hz if publish_hz > 0 else 0.2
        log_period = 1.0 / log_hz if log_hz > 0 else 1.0
        image_period = 1.0 / image_hz if image_hz > 0 else 0.2
        self._publish_timer = rospy.Timer(rospy.Duration(publish_period), self._publish_ints)
        self._log_timer = rospy.Timer(rospy.Duration(log_period), self._emit_logs)
        self._image_timer = rospy.Timer(rospy.Duration(image_period), self._publish_image_frame)

        rospy.loginfo("Int topics + logs test node started.")
        for topic_name in self._topic_names:
            rospy.loginfo("Publishing Int32 on: %s", topic_name)
        rospy.loginfo("Publishing Image frames on: %s", self._image_topic_name)
        rospy.loginfo("Tip: set ROS console level to DEBUG to see debug messages.")
        self._prepare_video_source()

    def _publish_ints(self, _event: Any) -> None:
        """Publish one integer message on each test topic."""
        self._counter += 1
        for index, (topic_name, publisher) in enumerate(
            zip(self._topic_names, self._publishers),
            start=1,
        ):
            msg = Int32()
            # Publish fully random values so the UI shows changing traffic clearly.
            msg.data = random.randint(-1000, 1000)
            publisher.publish(msg)

    def _emit_logs(self, _event: Any) -> None:
        """Emit one message for each requested ROS log level."""
        c = self._counter
        rospy.logdebug("[DEBUG] Test debug message #%s", c)
        rospy.loginfo("[INFO] Test info message #%s", c)
        rospy.logwarn("[WARN] Test warning message #%s", c)
        rospy.logerr("[ERROR] Test error message #%s", c)

    def _prepare_video_source(self) -> None:
        """Download and decode a random sample video into memory for frame publishing."""
        try:
            import imageio.v3 as iio
            import requests
        except Exception as err:
            rospy.logwarn(
                "Image topic disabled: missing dependency (%s). "
                "Install with: pip3 install requests imageio imageio-ffmpeg",
                err,
            )
            return

        url = random.choice(self.VIDEO_SAMPLE_URLS)
        cache_dir = Path(tempfile.gettempdir()) / "smart_gui_video_samples"
        cache_dir.mkdir(parents=True, exist_ok=True)
        filename = Path(urlparse(url).path).name or "sample.mp4"
        local_path = cache_dir / filename

        if not local_path.exists() or local_path.stat().st_size == 0:
            rospy.loginfo("Downloading sample video for Image topic: %s", url)
            try:
                response = requests.get(url, timeout=20)
                response.raise_for_status()
                local_path.write_bytes(response.content)
            except Exception as err:
                rospy.logwarn("Failed to download sample video: %s", err)
                return
        else:
            rospy.loginfo("Using cached sample video for Image topic: %s", local_path)

        try:
            frames: List[np.ndarray] = []
            for idx, frame in enumerate(iio.imiter(local_path)):
                arr = np.asarray(frame)
                if arr.ndim != 3:
                    continue
                if arr.shape[2] == 4:
                    arr = arr[:, :, :3]
                if arr.shape[2] != 3:
                    continue
                # Downscale large frames so WS/UI stays responsive.
                arr = self._resize_to_max_width(arr, max_width=640)
                frames.append(arr.astype(np.uint8, copy=False))
                if idx >= 180:
                    break

            if not frames:
                rospy.logwarn("Image topic disabled: no decodable frames in video %s", local_path)
                return

            with self._video_frame_lock:
                self._video_frames = frames
                self._video_frame_index = 0
            rospy.loginfo(
                "Loaded %d video frames for topic %s from %s",
                len(frames),
                self._image_topic_name,
                local_path.name,
            )
        except Exception as err:
            rospy.logwarn(
                "Image topic disabled: failed to decode video (%s). "
                "Try: pip3 install imageio imageio-ffmpeg",
                err,
            )

    def _resize_to_max_width(self, image: np.ndarray, max_width: int) -> np.ndarray:
        """Resize RGB image using nearest-neighbor to avoid extra dependencies."""
        height, width = image.shape[:2]
        if width <= max_width or width <= 0:
            return image
        scale = max_width / float(width)
        new_width = max(1, int(width * scale))
        new_height = max(1, int(height * scale))
        x_idx = np.linspace(0, width - 1, new_width).astype(np.int32)
        y_idx = np.linspace(0, height - 1, new_height).astype(np.int32)
        return image[y_idx][:, x_idx]

    def _publish_image_frame(self, _event: Any) -> None:
        """Publish one RGB frame from the preloaded sample video on sensor_msgs/Image."""
        with self._video_frame_lock:
            if not self._video_frames:
                return
            frame = self._video_frames[self._video_frame_index]
            self._video_frame_index = (self._video_frame_index + 1) % len(self._video_frames)

        height, width = frame.shape[:2]
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "smart_gui_test_camera"
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = int(width * 3)
        msg.data = frame.tobytes()
        self._image_publisher.publish(msg)


def main() -> None:
    """CLI entrypoint used by `rosrun smart_gui int_topics_and_logs`."""
    parser = argparse.ArgumentParser(
        description="Publish 4 Int32 topics, 1 Image topic, and emit test logs."
    )
    parser.add_argument("--hz", type=float, default=5.0, help="Topic publish rate (Hz)")
    parser.add_argument("--log-hz", type=float, default=1.0, help="Log rate (Hz)")
    parser.add_argument(
        "--image-hz",
        type=float,
        default=5.0,
        help="Image topic publish rate (Hz)",
    )
    args, _ = parser.parse_known_args()

    rospy.init_node("int_topics_and_logs_node", anonymous=False)
    IntTopicsAndLogsNode(publish_hz=args.hz, log_hz=args.log_hz, image_hz=args.image_hz)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
