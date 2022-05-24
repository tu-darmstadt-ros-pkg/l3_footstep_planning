#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import l3_footstep_planning_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QSplitter, QVBoxLayout, QPushButton

from vigir_generic_params.msg import SetParameterSetAction, SetParameterSetGoal, GetParameterSetAction, GetParameterSetGoal, GetParameterSetResult
from l3_footstep_planning_msgs.msg import ErrorStatus
from l3_footstep_planning_msgs.parameter import *
from l3_footstep_planning_vis_tools.error_status_widget import *
from l3_footstep_planning_vis_tools.logging import *
from l3_footstep_planning_vis_tools.parameter_set_widget import *
from l3_footstep_planning_vis_tools.parameter_tree_widget import *
from l3_footstep_planning_vis_tools.topic_widget import *
from l3_footstep_planning_vis_tools.qt_helper import *


class ParameterEditorDialog(Plugin):

    def __init__(self, context):
        super(ParameterEditorDialog, self).__init__(context)
        self.setObjectName('ParameterEditorDialog')

        self._parent = QWidget()
        self._widget = ParameterEditorWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class ParameterEditorWidget(QObject):

    def __init__(self, context):
        super(ParameterEditorWidget, self).__init__()

        # start widget
        widget = context
        vbox = QVBoxLayout()
        error_status_widget = QErrorStatusWidget()
        self.logger = Logger(error_status_widget)

        # parameter set selection
        self.parameter_set_widget = QParameterSetWidget(logger=self.logger)
        add_widget_with_frame(vbox, self.parameter_set_widget, "Parameter Set:")

        # start horizontal splitter
        vsplitter = QSplitter()
        vsplitter.setOrientation(Qt.Vertical)
        vsplitter.setChildrenCollapsible(False)

        # parameter set editor
        parameter_editor_widget = QParameterEditorWidget(logger=self.logger)
        add_widget_with_frame(vsplitter, parameter_editor_widget, "Editor:")

        # add error status widget
        add_widget_with_frame(vsplitter, error_status_widget, "Status:")

        # end horizontal splitter
        vbox.addWidget(vsplitter)

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # init parameter editor
        self.parameter_set_widget.topic_changed_signal.connect(parameter_editor_widget.set_get_parameter_set_topic_name)
        self.parameter_set_widget.param_changed_signal.connect(parameter_editor_widget.load_parameters)
        self.parameter_set_widget.param_cleared_signal.connect(parameter_editor_widget.clear)

    def shutdown_plugin(self):
        print("Shutting down ...")
        print("Done!")


class QParameterEditorWidget(QWidgetWithLogger):

    parameter_set_name = str()

    def __init__(self, parent=None, logger=Logger()):
        QWidgetWithLogger.__init__(self, parent, logger)

        # start widget
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)

        # add layout which is dynamically filled
        self.parameter_tree_widget = QParameterTreeWidget(logger=self.logger)
        vbox.addWidget(self.parameter_tree_widget)

        # button panel
        vbox_commands = QVBoxLayout()
        hbox = QHBoxLayout()

        # upload topic
        send_parameter_topic_widget = QTopicWidget(self, 'l3_footstep_planning_msgs/SetParameterSetAction', True)
        send_parameter_topic_widget.topic_changed_signal.connect(self._init_upload_paramater_set_client)
        hbox.addWidget(send_parameter_topic_widget)

        # upload command
        self.upload_command = QPushButton("Upload")
        self.upload_command.clicked.connect(self.upload_parameters)
        hbox.addWidget(self.upload_command)
        vbox_commands.addLayout(hbox)

        # reload command
        self.reload_command = QPushButton("Reload")
        self.reload_command.clicked.connect(self.reload_parameters)
        vbox_commands.addWidget(self.reload_command)

        # add button panel
        vbox.addLayout(vbox_commands)

        # end widget
        self.setLayout(vbox)

        # init
        self.clear()
        send_parameter_topic_widget.emit_topic_name()

    @Slot(str)
    def _init_get_parameter_set_client(self, topic_name):
        self.get_parameter_set_client = actionlib.SimpleActionClient(topic_name, GetParameterSetAction)
        print("Parameter set topic changed: " + topic_name)

    @Slot(str)
    def _init_upload_paramater_set_client(self, topic_name):
        if len(topic_name) > 0:
            print("Upload parameter set topic changed: " + topic_name)
            self.upload_parameter_set_client = actionlib.SimpleActionClient(topic_name, SetParameterSetAction)

    @Slot(str)
    def set_get_parameter_set_topic_name(self, topic_name):
        # assume that there are no topic remaps
        if len(topic_name) > 0:
            self._init_get_parameter_set_client(topic_name[:-6])

    # clear current vis
    @Slot()
    def clear(self):
        self.upload_command.setEnabled(False)
        self.reload_command.setEnabled(False)
        self.parameter_tree_widget.clear()

    @Slot()
    def upload_parameters(self):
        param_set_msg = self.parameter_tree_widget.get_parameter_set()

        if self.upload_parameter_set_client.wait_for_server(rospy.Duration(0.5)):
            self.logger.log_info("Sending parameter set '" + param_set_msg.name.data + "'...")

            goal = SetParameterSetGoal()
            goal.params = param_set_msg;
            self.upload_parameter_set_client.send_goal(goal)

            if self.upload_parameter_set_client.wait_for_result(rospy.Duration(0.5)):
                self.logger.log_info("Sent parameter set!")
                result = self.upload_parameter_set_client.get_result()
                self.logger.log(result.status)
            else:
                self.logger.log_error("Can't send parameter set. Check communcation!")
        else:
            self.logger.log_error("Can't connect to footstep parameter action server!")

    @Slot(str)
    def load_parameters(self, name):
        self.clear()

        if self.get_parameter_set_client.wait_for_server(rospy.Duration(0.5)):
            self.logger.log_info("Requesting parameter set '" + name + "'...")

            goal = GetParameterSetGoal()
            goal.name.data = name;
            self.get_parameter_set_client.send_goal(goal)

            if self.get_parameter_set_client.wait_for_result(rospy.Duration(0.5)):
                self.logger.log_info("Received parameter set!")
                result = self.get_parameter_set_client.get_result()
                self.current_parameter_set_name = name
                self._visualize_parameters(result.params)
            else:
                self.logger.log_error("Didn't received parameter set. Check communcation!")
        else:
            self.logger.log_error("Can't connect to footstep parameter action server!")

    @Slot()
    def reload_parameters(self):
        self.load_parameters(self.current_parameter_set_name)

    # get parameter set as msg
    def _visualize_parameters(self, param_set_msg):
        self.logger.log_info("Visualize parameter set with " + str(len(param_set_msg.params)) + " parameters.")
        self.parameter_tree_widget.set_parameter_set(param_set_msg)
        self.upload_command.setEnabled(True)
        self.reload_command.setEnabled(True)
