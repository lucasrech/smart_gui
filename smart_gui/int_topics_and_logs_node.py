"""Simple ROS 1 (Noetic) test node that publishes integer topics and emits logs.

This helper node is intended for UI testing:
- Publishes on four `std_msgs/Int32` topics.
- Emits log messages at debug/info/warning/error levels on a timer.
"""

import argparse
import random
from typing import Any, List

import rospy
from std_msgs.msg import Int32


class IntTopicsAndLogsNode:
    """Publish integers on multiple topics and generate ROS logs periodically."""

    def __init__(self, publish_hz: float = 5.0, log_hz: float = 1.0) -> None:
        """Create publishers and timers

        Args:
            publish_hz: Integer topic publish frequency in Hz.
            log_hz: Log emission frequency in Hz.
        """
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
            publisher = rospy.Publisher(topic_name, Int32, queue_size=10)
            self._publishers.append(publisher)
            self._topic_names.append(topic_name)

        publish_period = 1.0 / publish_hz if publish_hz > 0 else 0.2
        log_period = 1.0 / log_hz if log_hz > 0 else 1.0
        self._publish_timer = rospy.Timer(rospy.Duration(publish_period), self._publish_ints)
        self._log_timer = rospy.Timer(rospy.Duration(log_period), self._emit_logs)

        rospy.loginfo("Int topics + logs test node started.")
        for topic_name in self._topic_names:
            rospy.loginfo("Publishing Int32 on: %s", topic_name)
        rospy.loginfo("Tip: set ROS console level to DEBUG to see debug messages.")

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


def main() -> None:
    """CLI entrypoint used by `rosrun smart_gui int_topics_and_logs`."""
    parser = argparse.ArgumentParser(
        description="Publish Int32 messages on 4 topics and emit test logs."
    )
    parser.add_argument("--hz", type=float, default=5.0, help="Topic publish rate (Hz)")
    parser.add_argument("--log-hz", type=float, default=1.0, help="Log rate (Hz)")
    args, _ = parser.parse_known_args()

    rospy.init_node("int_topics_and_logs_node", anonymous=False)
    IntTopicsAndLogsNode(publish_hz=args.hz, log_hz=args.log_hz)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
