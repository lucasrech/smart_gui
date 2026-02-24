"""Simple ROS 2 test node that publishes integer topics and emits logs.

This helper node is intended for UI testing:
- Publishes on four `std_msgs/msg/Int32` topics.
- Emits log messages at debug/info/warning/error levels on a timer.
"""

import argparse
import random
from typing import Any, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class IntTopicsAndLogsNode(Node):
    """Publish integers on multiple topics and generate ROS logs periodically."""

    def __init__(self, publish_hz: float = 5.0, log_hz: float = 1.0) -> None:
        """Create publishers and timers.

        Args:
            publish_hz: Integer topic publish frequency in Hz.
            log_hz: Log emission frequency in Hz.
        """
        super().__init__("int_topics_and_logs_node")

        self._publishers: List[Any] = []
        self._topic_names: List[str] = []
        self._counter = 0

        topic_names = [
            "/test/int_topic_1",
            "/test/int_topic_2",
            "/test/int_topic_3",
            "/test/int_topic_4",
        ]
        for topic_name in topic_names:
            publisher = self.create_publisher(Int32, topic_name, 10)
            self._publishers.append(publisher)
            self._topic_names.append(topic_name)

        publish_period = 1.0 / publish_hz if publish_hz > 0 else 0.2
        log_period = 1.0 / log_hz if log_hz > 0 else 1.0
        self.create_timer(publish_period, self._publish_ints)
        self.create_timer(log_period, self._emit_logs)

        self.get_logger().info("Int topics + logs test node started.")
        for topic_name in self._topic_names:
            self.get_logger().info(f"Publishing Int32 on: {topic_name}")
        self.get_logger().info(
            "Tip: run with '--ros-args --log-level debug' to see debug messages."
        )

    def _publish_ints(self) -> None:
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

    def _emit_logs(self) -> None:
        """Emit one message for each requested ROS log level."""
        c = self._counter
        self.get_logger().debug(f"[DEBUG] Test debug message #{c}")
        self.get_logger().info(f"[INFO] Test info message #{c}")
        self.get_logger().warning(f"[WARN] Test warning message #{c}")
        self.get_logger().error(f"[ERROR] Test error message #{c}")


def main() -> None:
    """CLI entrypoint used by `ros2 run smart_gui int_topics_and_logs`."""
    parser = argparse.ArgumentParser(
        description="Publish Int32 messages on 4 topics and emit test logs."
    )
    parser.add_argument("--hz", type=float, default=5.0, help="Topic publish rate (Hz)")
    parser.add_argument("--log-hz", type=float, default=1.0, help="Log rate (Hz)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = IntTopicsAndLogsNode(publish_hz=args.hz, log_hz=args.log_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
