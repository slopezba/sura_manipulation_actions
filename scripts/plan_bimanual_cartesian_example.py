#!/usr/bin/env python3
"""Send a relative goal derived from the current TF, avoiding a guessed pose."""

import argparse
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sura_manipulation_actions.action import PlanBimanualCartesianTrajectory
from tf2_ros import Buffer, TransformException, TransformListener


def relative_pose_components(arm_a, arm_b):
    """Displacement and spatial rotation difference in shared reference axes."""
    position = [getattr(arm_b.translation, axis) - getattr(arm_a.translation, axis)
                for axis in ("x", "y", "z")]
    quaternions = []
    for transform in (arm_a, arm_b):
        values = [getattr(transform.rotation, axis) for axis in ("x", "y", "z", "w")]
        norm = math.hypot(*values)
        if not math.isfinite(norm) or norm < 1e-12:
            raise RuntimeError("TF orientation must have a finite, nonzero norm")
        quaternions.append([value / norm for value in values])
    lx, ly, lz, lw = quaternions[0]
    fx, fy, fz, fw = quaternions[1]
    quaternion = [fx * lw - fw * lx - fy * lz + fz * ly,
                  fy * lw - fw * ly - fz * lx + fx * lz,
                  fz * lw - fw * lz - fx * ly + fy * lx,
                  fw * lw + fx * lx + fy * ly + fz * lz]
    if quaternion[3] < 0.0:
        quaternion = [-value for value in quaternion]
    if not all(math.isfinite(value) for value in position):
        raise RuntimeError("TF position must be finite")
    return position, quaternion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-namespace", default="cirtesub")
    parser.add_argument("--arm-a", default="alpha_left")
    parser.add_argument("--arm-b", default="alpha_right")
    parser.add_argument("--arm-a-tip", default="")
    parser.add_argument("--arm-b-tip", default="")
    parser.add_argument("--reference-frame", default="",
                        help="Must match relative_reference_frame in the planner; default is robot base_link")
    parser.add_argument("--action", default="")
    parser.add_argument("--relative-dx", type=float, default=0.0,
                        help="Offset in metres along reference/base X; default preserves current relative pose")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--velocity-scaling", type=float, default=0.0,
                        help="Joint velocity limit factor in (0,1]; zero uses server default")
    parser.add_argument("--acceleration-scaling", type=float, default=0.0,
                        help="Joint acceleration limit factor in (0,1]; zero uses server default")
    args, ros_args = parser.parse_known_args()
    for name in ("velocity_scaling", "acceleration_scaling"):
        value = getattr(args, name)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be finite in [0,1]")
    rclpy.init(args=ros_args)
    node = Node("bimanual_cartesian_example")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    namespace = args.robot_namespace.strip("/")
    frame_prefix = f"{namespace}/" if namespace else ""
    topic_prefix = f"/{namespace}" if namespace else ""
    arm_a_tip = args.arm_a_tip or f"{frame_prefix}{args.arm_a}/standard_jaws_tool"
    arm_b_tip = args.arm_b_tip or f"{frame_prefix}{args.arm_b}/standard_jaws_tool"
    reference = args.reference_frame or f"{frame_prefix}base_link"
    action_name = args.action or f"{topic_prefix}/manipulation/plan_bimanual_cartesian_trajectory"
    client = ActionClient(node, PlanBimanualCartesianTrajectory, action_name)
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action unavailable: {action_name}; configure the lifecycle node")
        future = buffer.wait_for_transform_async(reference, arm_a_tip, Time())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError("Timed out waiting for the reference-to-arm-A TF")
        arm_a_tf = buffer.lookup_transform(reference, arm_a_tip, Time())
        stamp = Time.from_msg(arm_a_tf.header.stamp)
        future = buffer.wait_for_transform_async(reference, arm_b_tip, stamp)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError("Timed out waiting for the reference-to-arm-B TF at the same timestamp")
        arm_b_tf = buffer.lookup_transform(reference, arm_b_tip, stamp)
        position, quaternion = relative_pose_components(arm_a_tf.transform, arm_b_tf.transform)
        goal = PlanBimanualCartesianTrajectory.Goal()
        goal.arm_a = args.arm_a
        goal.arm_b = args.arm_b
        goal.relative_target.header.frame_id = reference
        goal.relative_target.header.stamp = arm_a_tf.header.stamp
        goal.relative_target.pose.position.x = position[0] + args.relative_dx
        goal.relative_target.pose.position.y = position[1]
        goal.relative_target.pose.position.z = position[2]
        for axis, value in zip(("x", "y", "z", "w"), quaternion):
            setattr(goal.relative_target.pose.orientation, axis, value)
        goal.relative_position_tolerance.x = 0.002
        goal.relative_position_tolerance.y = 0.002
        goal.relative_position_tolerance.z = 0.002
        goal.relative_orientation_tolerance.x = 0.03
        goal.relative_orientation_tolerance.y = 0.03
        goal.relative_orientation_tolerance.z = 0.03
        goal.cartesian_step = 0.005
        goal.planning_time = 10.0
        goal.velocity_scaling = args.velocity_scaling
        goal.acceleration_scaling = args.acceleration_scaling
        goal.execute = args.execute
        future = client.send_goal_async(
            goal, feedback_callback=lambda msg: node.get_logger().info(
                f"{msg.feedback.current_stage}: {msg.feedback.message}"))
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("Cartesian goal rejected; inspect the server diagnostic")
        node.get_logger().info(
            "Goal accepted. With execute=false it stays pending until the execute service is called.")
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result)
        except KeyboardInterrupt:
            canceled = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, canceled, timeout_sec=2.0)
            return
        node.get_logger().info(result.result().result.message)
    except (RuntimeError, TransformException) as error:
        node.get_logger().error(str(error))
    finally:
        del listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
