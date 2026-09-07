#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import actionlib
import rospy

from actionlib_msgs.msg import GoalStatus
from franka_msgs.msg import (
    ErrorRecoveryAction,
    ErrorRecoveryGoal,
    FrankaState,
)


STATE_TOPIC = "/franka_state_controller/franka_states"
RECOVERY_ACTION = "/franka_control/error_recovery"

ROBOT_MODES = {
    0: "OTHER",
    1: "IDLE",
    2: "MOVE",
    3: "GUIDING",
    4: "REFLEX",
    5: "USER_STOPPED",
    6: "AUTOMATIC_ERROR_RECOVERY",
}

NORMAL_MODES = {1, 2}
REFLEX_MODE = 4
RECOVERY_MODE = 6


def mode_name(mode):
    return ROBOT_MODES.get(
        int(mode),
        "UNKNOWN_%s" % str(mode)
    )


def wait_for_robot_state(timeout):
    try:
        return rospy.wait_for_message(
            STATE_TOPIC,
            FrankaState,
            timeout=timeout
        )
    except rospy.ROSException:
        return None


def wait_until_normal(timeout):
    deadline = time.monotonic() + timeout
    last_mode = None

    while (
        not rospy.is_shutdown()
        and time.monotonic() < deadline
    ):
        state = wait_for_robot_state(1.0)

        if state is None:
            continue

        current_mode = int(state.robot_mode)

        if current_mode != last_mode:
            rospy.loginfo(
                "Robot mode while recovering: %s",
                mode_name(current_mode)
            )
            last_mode = current_mode

        if current_mode in NORMAL_MODES:
            return True

        if current_mode in {
            0, 3, 5
        }:
            rospy.logerr(
                "Recovery stopped because robot entered "
                "non-automatic mode: %s",
                mode_name(current_mode)
            )
            return False

    return False


def main():
    rospy.init_node(
        "recover_franka_reflex",
        anonymous=True,
        disable_signals=False
    )

    state_timeout = float(
        rospy.get_param("~state_timeout", 10.0)
    )

    server_timeout = float(
        rospy.get_param("~server_timeout", 15.0)
    )

    recovery_timeout = float(
        rospy.get_param("~recovery_timeout", 30.0)
    )

    rospy.loginfo(
        "Checking Franka robot mode..."
    )

    state = wait_for_robot_state(
        state_timeout
    )

    if state is None:
        rospy.logerr(
            "No FrankaState received from %s.",
            STATE_TOPIC
        )
        return 2

    current_mode = int(
        state.robot_mode
    )

    rospy.loginfo(
        "Current Franka mode: %s",
        mode_name(current_mode)
    )

    # Robot is already ready.
    if current_mode in NORMAL_MODES:
        rospy.loginfo(
            "No error recovery required."
        )
        return 0

    # Recovery was already started elsewhere.
    if current_mode == RECOVERY_MODE:
        rospy.logwarn(
            "Robot is already in automatic error recovery."
        )

        if wait_until_normal(
            recovery_timeout
        ):
            rospy.loginfo(
                "Franka recovered successfully."
            )
            return 0

        rospy.logerr(
            "Existing recovery did not reach IDLE/MOVE."
        )
        return 3

    # Only Reflex is automatically cleared.
    if current_mode != REFLEX_MODE:
        rospy.logerr(
            "Automatic recovery refused for mode %s.",
            mode_name(current_mode)
        )

        rospy.logerr(
            "Check Franka Desk, emergency stop, brakes "
            "and FCI activation manually."
        )

        return 4

    rospy.logwarn(
        "Franka is in REFLEX mode."
    )

    rospy.logwarn(
        "Confirm that no person or object is contacting "
        "the robot before recovery."
    )

    client = actionlib.SimpleActionClient(
        RECOVERY_ACTION,
        ErrorRecoveryAction
    )

    rospy.loginfo(
        "Waiting for recovery action server: %s",
        RECOVERY_ACTION
    )

    if not client.wait_for_server(
        rospy.Duration(server_timeout)
    ):
        rospy.logerr(
            "Error recovery action server is unavailable."
        )
        return 5

    goal = ErrorRecoveryGoal()

    rospy.logwarn(
        "Sending automatic Franka error recovery..."
    )

    client.send_goal(goal)

    finished = client.wait_for_result(
        rospy.Duration(recovery_timeout)
    )

    if not finished:
        client.cancel_goal()

        rospy.logerr(
            "Automatic error recovery timed out."
        )
        return 6

    action_state = client.get_state()

    if action_state != GoalStatus.SUCCEEDED:
        rospy.logerr(
            "Recovery action failed with action state %d.",
            action_state
        )
        return 7

    if not wait_until_normal(
        recovery_timeout
    ):
        rospy.logerr(
            "Recovery action completed, but robot did not "
            "return to IDLE/MOVE."
        )
        return 8

    rospy.loginfo(
        "Franka REFLEX recovery completed successfully."
    )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())

    except rospy.ROSInterruptException:
        sys.exit(130)

    except Exception as error:
        rospy.logfatal(
            "Unexpected recovery error: %s",
            str(error)
        )
        sys.exit(10)
