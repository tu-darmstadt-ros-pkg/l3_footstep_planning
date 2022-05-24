#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox

from l3_footstep_planning_msgs.footstep_planning_msgs import *
from l3_footstep_planning_msgs.msg import StepPlan, ExecuteStepPlanAction, ExecuteStepPlanGoal
from l3_footstep_planning_vis_tools.error_status_widget import *
from l3_footstep_planning_vis_tools.logging import *
from l3_footstep_planning_vis_tools.topic_widget import *


# widget for sending step plans to controller
class QExecuteStepPlanWidget(QWidgetWithLogger):

    step_plan_sub = None
    execute_step_plan_client = None
    step_plan = None

    step_queue_size_signal = Signal(int)
    executed_steps_signal = Signal(int)

    def __init__(self, parent=None, logger=Logger(), step_plan_topic=str()):
        super(QExecuteStepPlanWidget, self).__init__(parent, logger)

        # start widget
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)

        # step plan input topic selection
        if len(step_plan_topic) == 0:
            step_plan_topic_widget = QTopicWidget(topic_type='l3_footstep_planning_msgs/StepPlan')
            step_plan_topic_widget.topic_changed_signal.connect(self._init_step_plan_subscriber)
            vbox.addWidget(step_plan_topic_widget)
        else:
            self._init_step_plan_subscriber(step_plan_topic)

        # execute action server topic selection
        execute_topic_widget = QTopicWidget(topic_type='l3_footstep_planning_msgs/ExecuteStepPlanAction', is_action_topic=True)
        execute_topic_widget.topic_changed_signal.connect(self._init_execute_action_client)
        vbox.addWidget(execute_topic_widget)

        # load execute widget from ui
        execute_step_plan_widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('l3_footstep_planning_vis_tools'), 'resource', 'execute_step_plan.ui')
        loadUi(ui_file, execute_step_plan_widget, {'QWidget': QWidget})
        vbox.addWidget(execute_step_plan_widget)

        # execute
        self.execute_button = execute_step_plan_widget.executePushButton
        self.execute_button.clicked.connect(self.execute_button_callback)

        # repeat
        self.repeat_button = execute_step_plan_widget.repeatPushButton
        self.repeat_button.clicked.connect(self.execute_button_callback)

        # stop
        self.stop_button = execute_step_plan_widget.stopPushButton
        self.stop_button.clicked.connect(self.stop_button_callback)

        # progress bar
        self.execution_progress_bar = execute_step_plan_widget.executionProgressBar
        self.execution_progress_bar.setFormat("0/0")
        self.step_queue_size_signal.connect(self.execution_progress_bar.setMaximum)
        self.executed_steps_signal.connect(self.execution_progress_bar.setValue)

        # status info
        self.status_line_edit = execute_step_plan_widget.statusLineEdit

        # step queue info
        self.step_queue_line_edit = execute_step_plan_widget.stepQueueLineEdit

        # first changeable step info
        self.first_changeable_step_line_edit = execute_step_plan_widget.firstChangeableLineEdit

        # end widget
        self.setLayout(vbox)

        self.reset_ui()

        # init widget
        if len(step_plan_topic) == 0:
            step_plan_topic_widget.emit_topic_name()
        execute_topic_widget.emit_topic_name()

    def reset_ui(self):
        self.execute_button.setEnabled(False)
        self.execute_button.setText("Execute [empty]")
        self.repeat_button.setEnabled(False)
        self.repeat_button.setText("Repeat [empty]")
        self.stop_button.setEnabled(False)
        self.execution_progress_bar.reset()
        self.execution_progress_bar.setFormat("0/0")
        self.status_line_edit.setText("N/A")
        self.step_queue_line_edit.setText("N/A [empty]")
        self.first_changeable_step_line_edit.setText("N/A")

    def shutdown_plugin(self):
        print("Shutting down ...")
        if self.step_plan_sub is None:
            self.step_plan_sub.unregister()
        print("Done!")

    def _init_step_plan_subscriber(self, topic_name):
        if len(topic_name) > 0:
            if self.step_plan_sub is not None:
                self.step_plan_sub.unregister()
            self.step_plan_sub = rospy.Subscriber(topic_name, StepPlan, self.step_plan_callback)
            print("Step Plan topic changed: " + topic_name)

    def _init_execute_action_client(self, topic_name):
        if len(topic_name) > 0:
            self.execute_step_plan_client = actionlib.SimpleActionClient(topic_name, ExecuteStepPlanAction)
            self.execute_button.setEnabled(self.step_plan is not None)
            self.repeat_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            print("Execution topic changed: " + topic_name)

    def step_plan_callback(self, step_plan):
        if not len(step_plan.plan.steps):
            return

        self.step_plan = step_plan
        self.execute_button.setText("Execute [" + str(self.step_plan.plan.steps[0].idx) + "; " +
                                    str(self.step_plan.plan.steps[-1].idx) + "]")

        self.execute_button.setEnabled(self.execute_step_plan_client is not None)

    def execute_button_callback(self):
        if self.execute_step_plan_client.wait_for_server(rospy.Duration(0.5)):
            # bring ui into execution mode
            self.reset_ui()
            self.repeat_button.setEnabled(True)
            self.repeat_button.setText("Repeat [" + str(self.step_plan.plan.steps[0].idx) + "; " +
                                        str(self.step_plan.plan.steps[-1].idx) + "]")
            self.stop_button.setEnabled(True)
            self.step_queue_size_signal.emit(0)

            self.logger.log_info("Requesting footstep plan execution...")
            goal = ExecuteStepPlanGoal()
            goal.step_plan = self.step_plan
            self.execute_step_plan_client.send_goal(goal, self.execute_step_plan_done_cb, self.execute_step_plan_active_cb, self.execute_step_plan_feedback_cb)
        else:
            self.logger.log_error("Can't connect to footstep controller action server!")

    def stop_button_callback(self):
        self.stop_button.setEnabled(False)
        self.execute_step_plan_client.cancel_goal()
        self.logger.log_info("Preempting step plan.")

    # called by action client
    def execute_step_plan_active_cb(self):
        self.logger.log_info("Execution of step plan started.")

    # called by action client
    def execute_step_plan_done_cb(self, goal_status, result):
        self.stop_button.setEnabled(False)
        if goal_status == 3:
            self.logger.log_info("Execution of step plan finished.")
        else:
            self.stop_button.setEnabled(False)
            self.step_queue_size_signal.emit(len(self.step_plan.plan.steps))
            self.status_line_edit.setText("FAILED")
            self.logger.log_error("Execution of step plan failed!")

        # update info field
        self.status_line_edit.setText(walk_controller_state_to_string(result.controller_state))

        # update logger
        self.logger.log(result.error_status)

    # called by action client
    def execute_step_plan_feedback_cb(self, feedback):
        # update progress bar
        if feedback.last_performed_step_idx >= 0:
            if feedback.queue_size > 0:
                self.step_queue_size_signal.emit(feedback.last_queued_step_idx)
            self.executed_steps_signal.emit(feedback.last_performed_step_idx)
            self.execution_progress_bar.setFormat(str(feedback.last_performed_step_idx) + "/" +
                                                  str(self.execution_progress_bar.maximum()))

        # update info fields
        self.status_line_edit.setText(walk_controller_state_to_string(feedback.controller_state))

        if feedback.queue_size > 0:
            self.step_queue_line_edit.setText(str(feedback.currently_executing_step_idx) + " [" +
                                              str(feedback.first_queued_step_idx) + "; " +
                                              str(feedback.last_queued_step_idx) + "]")

            self.first_changeable_step_line_edit.setText(str(feedback.first_changeable_step_idx))
        else:
            self.step_queue_line_edit.setText("N/A [empty]")
            self.first_changeable_step_line_edit.setText("N/A")

        # update logger
        self.logger.log(feedback.error_status)
