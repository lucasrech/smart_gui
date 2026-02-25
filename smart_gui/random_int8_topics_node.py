"""ROS 1 (Noetic) helper node that publishes random Int8 values on random topics.

This node is intended for UI/integration testing, allowing the frontend to
discover and monitor multiple active topics without requiring a real robot stack.
"""

import argparse
import random
import string
from typing import Any, List

import rospy
from std_msgs.msg import Int8


def _random_suffix(size: int = 6) -> str:
    """Generate a short random suffix for topic name uniqueness."""
    chars = string.ascii_lowercase + string.digits
    return "".join(random.choice(chars) for _ in range(size))


class RandomInt8TopicsNode:
    """Create multiple random `std_msgs/Int8` publishers and publish periodically."""

    def __init__(self, topic_count: int = 5, publish_hz: float = 5.0) -> None:
        """Initialize publishers/timer.

        Args:
            topic_count: Number of random topics to create.
            publish_hz: Publish frequency in Hz for each topic.
        """
        self._publishers = []
        self._topics: List[str] = []

        for _ in range(max(1, topic_count)):
            topic_name = f"/rand_int8_{_random_suffix()}"
            publisher = rospy.Publisher(topic_name, Int8, queue_size=10)
            self._publishers.append(publisher)
            self._topics.append(topic_name)

        period_sec = 1.0 / publish_hz if publish_hz > 0 else 0.2
        self._timer = rospy.Timer(rospy.Duration(period_sec), self._on_timer)

        rospy.loginfo("Random Int8 publisher node started.")
        for topic_name in self._topics:
            rospy.loginfo("Publishing on: %s", topic_name)

    def _on_timer(self, _event: Any) -> None:
        """Publish one random Int8 message on each configured topic."""
        for publisher, topic_name in zip(self._publishers, self._topics):
            msg = Int8()
            msg.data = random.randint(-128, 127)
            publisher.publish(msg)
            rospy.logdebug("%s: %s", topic_name, msg.data)


def main() -> None:
    """CLI entrypoint used by `rosrun smart_gui random_int8_topics`."""
    parser = argparse.ArgumentParser(
        description="Publish random Int8 values on random topic names."
    )
    parser.add_argument("--topic-count", type=int, default=5)
    parser.add_argument("--hz", type=float, default=5.0)
    args, _ = parser.parse_known_args()

    rospy.init_node("random_int8_topics_node", anonymous=False)
    RandomInt8TopicsNode(topic_count=args.topic_count, publish_hz=args.hz)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
