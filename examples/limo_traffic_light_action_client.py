#!/usr/bin/env python3
"""Reference LIMO action client for traffic_light_msgs/action/LookupTrafficLight.

Not built by colcon. Copy the ActionClient setup and the goal/result handling
below into your own LIMO control node, alongside whatever other clients it
already runs (nav goals, docking, etc.).

Run directly against a live traffic_light node for a quick check:

    source ~/ros2_ws/install/setup.bash
    python3 limo_traffic_light_action_client.py
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from geometry_msgs.msg import Twist
from traffic_light_msgs.action import LookupTrafficLight
from traffic_light_msgs.msg import TrafficLight

STOP = Twist()

GO = Twist()
GO.linear.x = 0.2

STATE_NAME = {
    TrafficLight.UNKNOWN: "UNKNOWN",
    TrafficLight.RED: "RED",
    TrafficLight.YELLOW: "YELLOW",
    TrafficLight.GREEN: "GREEN",
}


class LimoTrafficLightClient(Node):

    def __init__(self):
        super().__init__("limo_traffic_light_client")

        self.declare_parameter("action_name", "traffic_light/lookup")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("lookup_timeout", 1.0)
        self.declare_parameter("poll_period", 1.0)

        action_name = self.get_parameter("action_name").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._action_client = ActionClient(self, LookupTrafficLight, action_name)

        self._goal_in_flight = False
        self._timer = self.create_timer(
            self.get_parameter("poll_period").value,
            self._request_lookup,
        )

    def _request_lookup(self) -> None:
        if self._goal_in_flight:
            return

        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                "traffic_light/lookup action server not available yet",
                throttle_duration_sec=5.0,
            )
            return

        goal = LookupTrafficLight.Goal()
        goal.lookup_timeout = float(self.get_parameter("lookup_timeout").value)

        self._goal_in_flight = True
        send_goal_future = self._action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("traffic_light/lookup goal was rejected")
            self._goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future: Future) -> None:
        self._goal_in_flight = False

        wrapped_result = future.result()
        state = wrapped_result.result.traffic_light.state
        state_name = STATE_NAME.get(state, "UNKNOWN")

        self.get_logger().info(f"traffic light: {state_name}")
        self._drive_for(state)

    def _drive_for(self, state: int) -> None:
        # RED, YELLOW, and UNKNOWN all stop the robot; only a confirmed
        # GREEN lets it proceed.
        self._cmd_vel_publisher.publish(GO if state == TrafficLight.GREEN else STOP)


def main() -> None:
    rclpy.init()

    node = LimoTrafficLightClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
