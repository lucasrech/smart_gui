import argparse
import random
import string
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


def _random_suffix(size: int = 6) -> str:
    chars = string.ascii_lowercase + string.digits
    return "".join(random.choice(chars) for _ in range(size))


class RandomInt8TopicsNode(Node):
    def __init__(self, topic_count: int = 5, publish_hz: float = 5.0) -> None:
        super().__init__("random_int8_topics_node")
        self._publishers = []
        self._topics: List[str] = []

        for _ in range(max(1, topic_count)):
            topic_name = f"/rand_int8_{_random_suffix()}"
            publisher = self.create_publisher(Int8, topic_name, 10)
            self._publishers.append(publisher)
            self._topics.append(topic_name)

        period_sec = 1.0 / publish_hz if publish_hz > 0 else 0.2
        self.create_timer(period_sec, self._on_timer)

        self.get_logger().info("Random Int8 publisher node started.")
        for topic_name in self._topics:
            self.get_logger().info(f"Publishing on: {topic_name}")

    def _on_timer(self) -> None:
        for publisher, topic_name in zip(self._publishers, self._topics):
            msg = Int8()
            msg.data = random.randint(-128, 127)
            publisher.publish(msg)
            self.get_logger().debug(f"{topic_name}: {msg.data}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish random Int8 values on random topic names."
    )
    parser.add_argument("--topic-count", type=int, default=5)
    parser.add_argument("--hz", type=float, default=5.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RandomInt8TopicsNode(topic_count=args.topic_count, publish_hz=args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
