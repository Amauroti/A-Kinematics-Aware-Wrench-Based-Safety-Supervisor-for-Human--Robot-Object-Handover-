#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from franka_gripper.msg import MoveAction, MoveGoal

class SafetySupervisor:
    def __init__(self):
        rospy.init_node("safety_supervisor")

        self.release_done = False
        self.last_state = None

        self.open_width = rospy.get_param("~open_width", 0.08)
        self.open_speed = rospy.get_param("~open_speed", 0.03)

        rospy.loginfo("Waiting for /franka_gripper/move action server...")
        self.client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)

        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Cannot connect to /franka_gripper/move action server.")
            rospy.logerr("Please check whether franka_control and franka_gripper are running.")
            return

        rospy.loginfo("Connected to /franka_gripper/move action server.")

        self.sub = rospy.Subscriber("/safety_state", String, self.state_callback)

        rospy.loginfo("Safety supervisor started.")
        rospy.loginfo("Only SAFE_RELEASE can open the gripper.")

        rospy.spin()

    def state_callback(self, msg):
        state = msg.data.strip()

        if state != self.last_state:
            rospy.loginfo("Safety state changed: %s", state)
            self.last_state = state

        if state == "WAITING":
            self.release_done = False
            rospy.loginfo("WAITING: keep holding.")

        elif state == "HAND_APPROACH":
            rospy.loginfo("HAND_APPROACH: hand detected, but do not release.")

        elif state == "SHARED_HOLDING":
            rospy.loginfo("SHARED_HOLDING: possible shared holding, wait for stable SAFE_RELEASE.")

        elif state == "SAFE_RELEASE":
            if not self.release_done:
                rospy.logwarn("SAFE_RELEASE received: opening gripper.")
                self.open_gripper()
                self.release_done = True
            else:
                rospy.loginfo("SAFE_RELEASE already handled, ignoring repeated command.")

        elif state == "ABORT":
            rospy.logwarn("ABORT: do not release. Keep holding object.")

        else:
            rospy.logwarn("Unknown safety state: %s. Fail-safe: do not release.", state)

    def open_gripper(self):
        goal = MoveGoal()
        goal.width = self.open_width
        goal.speed = self.open_speed

        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(5.0))

        if finished:
            result = self.client.get_result()
            rospy.loginfo("Gripper move finished. Result: %s", result)
        else:
            rospy.logerr("Gripper move timeout.")
            self.client.cancel_goal()

if __name__ == "__main__":
    SafetySupervisor()
