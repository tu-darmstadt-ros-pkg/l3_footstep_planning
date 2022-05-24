#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import l3_footstep_planning_msgs.msg

from l3_footstep_planning_msgs.msg import ErrorStatus, ExecuteStepPlanFeedback


# creates error status with given error and/or warning code and message
def create_error_status(error=0, error_msg=str(), warning=0, warning_msg=str()):
    error_status = ErrorStatus()

    if error != 0:
        error_status.error = error
        error_status.error_msg = "[Error] " + error_msg

    if warning != 0:
        warning_status.warning = warning
        warning_status.warning_msg = "[Warning] " + warning_msg

    return error_status


def walk_controller_state_to_string(state):
    if state == ExecuteStepPlanFeedback.NOT_READY:
        return "NOT_READY"
    elif state == ExecuteStepPlanFeedback.READY:
        return "READY"
    elif state == ExecuteStepPlanFeedback.ACTIVE:
        return "ACTIVE"
    elif state == ExecuteStepPlanFeedback.PAUSED:
        return "PAUSED"
    elif state == ExecuteStepPlanFeedback.FINISHED:
        return "FINISHED"
    elif state == ExecuteStepPlanFeedback.FAILED:
        return "FAILED"

