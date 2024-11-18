"""Demonstration of using the nav2 action interface in Python.

This node navigates to a goal pose provided on the command line.  This
code also include a demonstration of interacting with OccupancyGrid
messages through the map_util.Map class.

DO NOT MODIFY OR IMPORT THIS FILE.  It is only provided as an
illustration.

Author: Nathan Sprague and Kevin Molloy
Version: 10/24/2023

"""

import argparse
import time
import numpy as np

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
# this import relies on the .vscode/settings.json file to locate jmu_ros2_util
from jmu_ros2_util import map_utils
import tf_transformations


import random


def create_nav_goal(x, y, theta):
    goal = NavigateToPose.Goal()

    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y

    # We need to convert theta to a quaternion....
    quaternion = tf_transformations.quaternion_from_euler(0, 0, theta, "rxyz")
    goal.pose.pose.orientation.x = quaternion[0]
    goal.pose.pose.orientation.y = quaternion[1]
    goal.pose.pose.orientation.z = quaternion[2]
    goal.pose.pose.orientation.w = quaternion[3]
    return goal


class rescue_node(rclpy.node.Node):

    def __init__(self, x=0.0, y=0.0, theta=0.0, timeout=float("inf")):
        super().__init__("rescue_node")

        # This QOS Setting is used for topics where the messages
        # should continue to be available indefinitely once they are
        # published. Maps fall into this category.  They typically
        # don't change, so it makes sense to publish them once.
        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid, "map", self.map_callback, qos_profile=latching_qos
        )

        # Create a timer to continously send goals
        self.create_timer(1.0, self.continous_callback)

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.map = None
        self.goal = create_nav_goal(x, y, theta)
        self.goal_set = False
        self.goal_future = None
        self.start_time = None
        self.timeout = 60.0

    def map_callback(self, map_msg):
        """Process the map message.

        This doesn't really do anything useful, it is purely intended
        as an illustration of the Map class.

        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))

            # Here is how to access map cells to see if they are free:
            x = self.goal.pose.pose.position.x
            y = self.goal.pose.pose.position.y
            val = self.map.get_cell(x, y)
            if val == 100:
                free = "occupied"
            elif val == 0:
                free = "free"
            else:
                free = "unknown"
            self.get_logger().info(f"HEY! Map position ({x:.2f}, {y:.2f}) is {free}")

    def continous_callback(self):
        # If no map dont make a random goal and just return
        if self.map is None:
            self.get_logger().info(
                "attempted to set a rand goal, map does not exist yet"
            )
            return

        # If we have a goal set then check if it is done
        if self.goal_set:
            self.get_logger().info("Goal set, navigation in progress")
            # Check if the goal has timed out
            if time.time() - self.start_time > self.timeout:
                self.get_logger().info("Goal has timed out, cancelling")
                self.goal_future.cancel()
                self.goal_future = None
                self.goal_set = False
                return
            return
        else:
            # If we have a map and dont have a goal then create a random goal
            self.create_rand_goal()

    def create_rand_goal(self):
        free = False
        # Keep trying until we find a free location
        while free is False:
            self.get_logger().info("Finding a free location")
            random_x = (
                self.map.origin_x
                + (self.map.width + self.map.origin_x) * random.random()
            )
            random_y = (
                self.map.origin_y
                + (self.map.height + self.map.origin_y) * random.random()
            )
            loc_value = self.map.get_cell(random_x, random_y)
            # Check if the location is free
            if loc_value == 0:
                free = True
        self.get_logger().info(
            f"Setting goal: ({random_x:.2f}, {random_y:.2f}) is {free}"
        )
        # Create the goal
        self.goal = create_nav_goal(random_x, random_y, 0)
        # Goal set
        self.goal_set = True
        # Set time
        self.start_time = time.time()
        # Send the goal
        self.goal_future = self.ac.send_goal_async(self.goal)
        # Add the callback to check if the goal is done (didnt know this was a thing, but glad I found it)
        self.goal_future.add_done_callback(self.on_goal_result)

    # Used to check
    def on_goal_result(self, future):
        if future.done():
            if future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    "NAVIGATION SERVER REPORTS SUCCESS. ONWARDS TO NEXT GOAL!!"
                )
            if future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. RETRYING!!")
        else:
            self.get_logger().info("Goal result unavailable")

        # Reset the goal set flag and future
        self.goal_future = None
        self.goal_set = False


def main():
    # parser = argparse.ArgumentParser(
    #     formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    #     description="Navigate to a designated location.",
    # )
    # parser.add_argument("x", default=0.0, type=float, help="X-coordinate of goal")
    # parser.add_argument("y", default=0.0, type=float, help="Y-coordinate of goal")
    # parser.add_argument("theta", default=0.0, type=float, help="Orientation of goal")
    # parser.add_argument(
    #     "--timeout",
    #     default=float("inf"),
    #     type=float,
    #     help="How long to wait before cancelling navigation.",
    # )
    # args = parser.parse_args()

    rclpy.init()

    # node = rescue_node(args.x, args.y, args.theta, args.timeout)
    node = rescue_node()

    # future = node.send_goal()

    rclpy.spin(node)

    # rclpy.spin_until_future_complete(node, future)

    # node.get_logger().info("Node's future: " + str(future.result()))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
