#!/usr/bin/env python3

import math
import random
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class RandomGoalWander(object):
    """
    Simple node that continuously sends random 2D goals to move_base.
    Rely on move_base (and Turtlebot3 navigation stack) to avoid obstacles.
    """

    def __init__(self):
        rospy.init_node("random_goal_wander")

        # Parameters (can be overridden via ROS params)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.min_x = rospy.get_param("~min_x", -2.0)
        self.max_x = rospy.get_param("~max_x", 2.0)
        self.min_y = rospy.get_param("~min_y", -2.0)
        self.max_y = rospy.get_param("~max_y", 2.0)
        self.goal_timeout = rospy.get_param("~goal_timeout", 120.0)  # seconds
        self.retry_delay = rospy.get_param("~retry_delay", 3.0)

        rospy.loginfo("RandomGoalWander: connecting to move_base...")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("RandomGoalWander: connected to move_base.")

        self.current_goal = None
        self.current_goal_time = None

    def make_random_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.frame_id

        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)
        yaw = random.uniform(-math.pi, math.pi)

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Convert yaw to quaternion (z, w for 2D)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        rospy.loginfo("RandomGoalWander: new random goal x=%.2f y=%.2f yaw=%.2f", x, y, yaw)
        return goal

    def send_new_goal(self):
        self.current_goal = self.make_random_goal()
        self.current_goal_time = rospy.Time.now()
        self.client.send_goal(self.current_goal)

    def spin(self):
        rate = rospy.Rate(1.0)
        self.send_new_goal()

        while not rospy.is_shutdown():
            state = self.client.get_state()

            if state in [GoalStatus.SUCCEEDED]:
                rospy.loginfo("RandomGoalWander: goal reached, sending a new one.")
                self.send_new_goal()

            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("RandomGoalWander: goal aborted/rejected, picking a new goal after delay.")
                rospy.sleep(self.retry_delay)
                self.send_new_goal()

            else:
                # Check for timeout on the current goal
                if (self.current_goal_time is not None and
                        (rospy.Time.now() - self.current_goal_time).to_sec() > self.goal_timeout):
                    rospy.logwarn("RandomGoalWander: goal timeout, canceling and sending a new goal.")
                    self.client.cancel_goal()
                    rospy.sleep(self.retry_delay)
                    self.send_new_goal()

            rate.sleep()

    def shutdown(self):
        rospy.loginfo("RandomGoalWander: shutting down, canceling goal.")
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass


if __name__ == "__main__":
    node = RandomGoalWander()
    rospy.on_shutdown(node.shutdown)
    node.spin()


