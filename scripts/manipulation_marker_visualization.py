#!/usr/bin/env python3
"""Publish only the latest complete manipulation visualization to RViz."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


CLEAR_NAMESPACE = ""
CLEAR_ID = -2147483648


def replacement_snapshot(plan):
    """Clear the previous display and add the new snapshot in a single message."""
    keys = set()
    for marker in plan.markers:
        key = (marker.ns, marker.id)
        if key in keys:
            raise ValueError(f"Duplicate marker key: {key}")
        if key == (CLEAR_NAMESPACE, CLEAR_ID):
            raise ValueError(f"Reserved marker key: {key}")
        if marker.action != Marker.ADD:
            raise ValueError("A plan snapshot must contain only ADD markers")
        keys.add(key)

    clear = Marker()
    if plan.markers:
        clear.header = deepcopy(plan.markers[0].header)
    clear.ns = CLEAR_NAMESPACE
    clear.id = CLEAR_ID
    clear.action = Marker.DELETEALL
    clear.pose.orientation.w = 1.0
    # RViz scopes DELETEALL to its namespace when nonempty. Keep it empty to
    # clear all plans; the reserved ID prevents a duplicate key in this array.
    return MarkerArray(markers=[clear, *plan.markers])


class ManipulationMarkerVisualization(Node):
    def __init__(self):
        super().__init__("manipulation_marker_visualization")
        input_topic = self.declare_parameter(
            "input_topic", "manipulation/marker_plans").value
        output_topic = self.declare_parameter(
            "output_topic", "manipulation/markers").value
        if self.resolve_topic_name(input_topic) == self.resolve_topic_name(output_topic):
            raise ValueError("Marker input and output topics must differ")
        self._publisher = self.create_publisher(MarkerArray, output_topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Inputs are new events, not per-action retained histories. A reconnect
        # cannot replay an old action's plan over the latest visualization.
        self._subscription = self.create_subscription(
            MarkerArray, input_topic, self._on_plan, QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE))

    def _on_plan(self, plan):
        try:
            snapshot = replacement_snapshot(plan)
        except ValueError as error:
            self.get_logger().warning(f"Rejected marker snapshot: {error}")
            return
        self._publisher.publish(snapshot)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManipulationMarkerVisualization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
