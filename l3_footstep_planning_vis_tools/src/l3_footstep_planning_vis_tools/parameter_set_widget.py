#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import l3_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt, QSize, Signal, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox

from vigir_generic_params.msg import GetParameterSetNamesAction, GetParameterSetNamesGoal, GetParameterSetNamesResult
from l3_footstep_planning_vis_tools.logging import *
from l3_footstep_planning_vis_tools.topic_widget import *


# widget for parameter set selection
class QParameterSetWidget(QWidgetWithLogger):

    topic_changed_signal = Signal(str)
    param_cleared_signal = Signal()
    param_changed_signal = Signal(str)

    def __init__(self, parent=None, logger=Logger()):
        QWidgetWithLogger.__init__(self, parent, logger)

        # start widget
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)

        # parameter action server topic selection
        topic_widget = QTopicWidget(self, 'vigir_generic_params/GetParameterSetNamesAction', True)
        vbox.addWidget(topic_widget)

        # parameter set selection
        self.parameter_set_selection_widget = QParameterSetSelectionWidget(self, logger)
        self.parameter_set_selection_widget.param_cleared_signal.connect(self.param_cleared)
        self.parameter_set_selection_widget.param_changed_signal.connect(self.param_changed)
        topic_widget.topic_changed_signal.connect(self.topic_changed)
        topic_widget.topic_changed_signal.connect(self.parameter_set_selection_widget.set_topic_name)
        vbox.addWidget(self.parameter_set_selection_widget)

        # end widget
        self.setLayout(vbox)

        # init widget
        topic_widget.emit_topic_name()

    def current_parameter_set_name(self):
        return self.parameter_set_selection_widget.current_parameter_set_name()

    @Slot(str)
    def topic_changed(self, name):
        self.topic_changed_signal.emit(name)

    @Slot()
    def param_cleared(self):
        self.param_cleared_signal.emit()

    @Slot(str)
    def param_changed(self, name):
        self.param_changed_signal.emit(name)


# widget for parameter set selection
class QParameterSetSelectionWidget(QWidgetWithLogger):

    NO_PARAM_SET_TEXT = "No parameters available!"
    SELECT_TEXT = "<Select>"

    param_cleared_signal = Signal()
    param_changed_signal = Signal(str)

    def __init__(self, parent=None, logger=Logger()):
        QWidgetWithLogger.__init__(self, parent, logger)

        # start widget
        hbox = QHBoxLayout()
        hbox.setContentsMargins(0, 0, 0, 0)

        # get system icon
        icon = QIcon.fromTheme("view-refresh")
        size = icon.actualSize(QSize(32, 32))

        # add combo box
        self.parameter_set_names_combo_box = QComboBox()
        self.parameter_set_names_combo_box.currentIndexChanged[str].connect(self.param_changed)
        hbox.addWidget(self.parameter_set_names_combo_box)

        # add refresh button
        self.get_all_parameter_set_names_button = QPushButton()
        self.get_all_parameter_set_names_button.clicked.connect(self._get_all_parameter_set_names)

        self.get_all_parameter_set_names_button.setIcon(icon)
        self.get_all_parameter_set_names_button.setFixedSize(size.width()+2, size.height()+2)

        hbox.addWidget(self.get_all_parameter_set_names_button)

        # end widget
        self.setLayout(hbox)

        # init widget
        self.reset_parameter_set_selection()

    def _init_action_client(self, topic_name):
        self.get_parameter_set_names_client = actionlib.SimpleActionClient(topic_name, GetParameterSetNamesAction)
        print("Parameter set topic changed: " + topic_name)

    @Slot(str)
    def set_topic_name(self, topic_name):
        if len(topic_name) > 0:
            self._init_action_client(topic_name)
            self._get_all_parameter_set_names()
        else:
            self.reset_parameter_set_selection()

    def reset_parameter_set_selection(self):
        self.parameter_set_names_combo_box.setEnabled(False)
        self.parameter_set_names_combo_box.blockSignals(True)
        self.parameter_set_names_combo_box.clear()
        self.parameter_set_names_combo_box.addItem(self.NO_PARAM_SET_TEXT)
        self.get_all_parameter_set_names_button.setEnabled(False)
        self.param_cleared_signal.emit()

    def current_parameter_set_name(self):
        if self.parameter_set_names_combo_box.currentText() == self.NO_PARAM_SET_TEXT:
            return str()
        else:
            return self.parameter_set_names_combo_box.currentText()

    @Slot(str)
    def param_changed(self, name):
        self.param_changed_signal.emit(name)

    # parameter set names handler
    def _get_all_parameter_set_names(self):
        if self.get_parameter_set_names_client.wait_for_server(rospy.Duration(0.5)):
            self.logger.log_info("Requesting current list of parameter set names.")
            goal = GetParameterSetNamesGoal()
            self.get_parameter_set_names_client.send_goal(goal)

            # waiting for getting list of parameter set names
            if self.get_parameter_set_names_client.wait_for_result(rospy.Duration(1.0)):
                result = self.get_parameter_set_names_client.get_result()

                self.logger.log_info("Received " + str(len(result.names)) + " parameter set names.")

                self.parameter_set_names_combo_box.blockSignals(True)
                self.parameter_set_names_combo_box.clear()
                self.parameter_set_names_combo_box.addItem(self.SELECT_TEXT)
                self.parameter_set_names_combo_box.setItemData(0, 0, Qt.UserRole-1)
                self.param_cleared_signal.emit()

                for name in result.names:
                    self.parameter_set_names_combo_box.addItem(name.data)

                self.parameter_set_names_combo_box.setEnabled(True)
                self.parameter_set_names_combo_box.blockSignals(False)
                self.get_all_parameter_set_names_button.setEnabled(True)

                if self.parameter_set_names_combo_box.count > 0:
                    self.parameter_set_names_combo_box.setCurrentIndex(1)
            else:
                self.logger.log_error("Didn't received any results. Check communcation!")
                self.reset_parameter_set_selection()
        else:
            self.logger.log_error("Can't connect to footstep planner parameter action server!")
            self.reset_parameter_set_selection()
