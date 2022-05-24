#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import l3_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt, QSize, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox

from l3_footstep_planning_msgs.msg import StepPlan, StepPlanRequestAction, StepPlanRequestGoal
from l3_footstep_planning_vis_tools.logging import *
from l3_footstep_planning_vis_tools.topic_widget import *


# widget for parameter set selection
class QPlannerServerWidget(QWidgetWithLogger):

    topic_changed_signal = Signal(str)

    def __init__(self, parent=None, logger=Logger()):
        QWidgetWithLogger.__init__(self, parent, logger)

        # start widget
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)

        # parameter action server topic selection
        topic_widget = QTopicWidget(self, 'l3_footstep_planning_msgs/StepPlanRequestAction', True)
        vbox.addWidget(topic_widget)

        # parameter set selection
        topic_widget.topic_changed_signal.connect(self.topic_changed)

        # end widget
        self.setLayout(vbox)

        self.step_plan_request_client = None

        # init widget
        topic_widget.emit_topic_name()

    def _init_action_client(self, topic_name):
        # namespace = str('/').join(topic_name.rsplit('/')[:-1])
        self.step_plan_pub = rospy.Publisher("step_plan", StepPlan, queue_size=1)
        self.step_plan_request_client = actionlib.SimpleActionClient(topic_name, StepPlanRequestAction)
        print("Planner server changed: " + topic_name)

    @Slot(str)
    def topic_changed(self, name):
        if len(name) > 0:
            self._init_action_client(name)
        self.topic_changed_signal.emit(name)

    def is_connected(self):
        if self.step_plan_request_client:
            return self.step_plan_request_client.wait_for_server(rospy.Duration(0.5))
        return False

    def send_plan_request(self, request):
        # send request
        if self.step_plan_request_client.wait_for_server(rospy.Duration(0.5)):
            self.logger.log_info("Sending footstep plan request...")

            goal = StepPlanRequestGoal()
            goal.plan_request = request
            self.step_plan_request_client.send_goal(goal)

            if self.step_plan_request_client.wait_for_result(rospy.Duration(5.0)):
                self.logger.log_info("Received footstep plan!")
                self.logger.log(self.step_plan_request_client.get_result().status)
                self.step_plan_pub.publish(self.step_plan_request_client.get_result().step_plan)
            else:
                self.logger.log_error("Didn't received any results. Check communcation!")
        else:
            self.logger.log_error("Can't connect to footstep planner action server!")
