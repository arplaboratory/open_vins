#!/usr/bin/env python3
"""Prints the VIO position uncertainty (std-dev, in meters) from nav_msgs/Odometry.pose.covariance.

Usage:
    source /opt/ros/humble/setup.bash && source <colcon_ws>/install/setup.bash
    python3 watch_covariance.py [odom_topic]

Defaults to /quadrotor/odom; pass a different topic name as the first argument
(e.g. /quadrotor/ov_msckf/odomimu) to watch it before the odom_transform remap.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

DEFAULT_TOPIC = "/quadrotor/odom"


class CovarianceWatcher(Node):
    def __init__(self, topic: str):
        super().__init__("covariance_watcher")
        self.create_subscription(Odometry, topic, self.cb, 10)
        self.get_logger().info(f"Watching position covariance on '{topic}'")

    def cb(self, msg: Odometry):
        var_x, var_y, var_z = msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]
        self.get_logger().info(
            f"pos std-dev [m]  x={math.sqrt(var_x):.4f}  y={math.sqrt(var_y):.4f}  z={math.sqrt(var_z):.4f}"
        )


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    rclpy.init()
    try:
        rclpy.spin(CovarianceWatcher(topic))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
