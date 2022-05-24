#!/usr/bin/env python

import math

import rospy
import actionlib
import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, QObject, Slot, QSignalMapper
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLineEdit, QPushButton, QDoubleSpinBox

from l3_msgs.msg import Foothold
from l3_footstep_planning_msgs.msg import StepPlanRequest, StepPlanRequestAction, StepPlanRequestGoal, StepPlanRequestResult, PatternParameters
from l3_footstep_planning_vis_tools.execute_step_plan_widget import *
from l3_footstep_planning_vis_tools.error_status_widget import *
from l3_footstep_planning_vis_tools.parameter_set_widget import *
from l3_footstep_planning_vis_tools.planner_server_widget import *
from l3_footstep_planning_vis_tools.qt_helper import *


class StepInterfaceDialog(Plugin):

    def __init__(self, context):
        super(StepInterfaceDialog, self).__init__(context)
        self.setObjectName('StepInterfaceDialog')

        self._parent = QWidget()
        self._widget = StepInterfaceWidget(context)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class StepInterfaceWidget(QObject):

    command_buttons = []

    def __init__(self, context, add_execute_widget=True):
        super(StepInterfaceWidget, self).__init__(context)
        self.setObjectName('StepInterface')

        # init signal mapper
        self.command_mapper = QSignalMapper(self)
        self.command_mapper.mapped.connect(self._publish_step_plan_request)

        # start widget
        self._widget = QWidget()
        error_status_widget = QErrorStatusWidget()
        self.logger = Logger(error_status_widget)
        vbox = QVBoxLayout()

        # start control box
        controls_hbox = QHBoxLayout()

        # left coloumn
        left_controls_vbox = QVBoxLayout()
        left_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(left_controls_vbox, "Rotate Left", PatternParameters.ROTATE_LEFT)
        self.add_command_button(left_controls_vbox, "Strafe Left", PatternParameters.STRAFE_LEFT)
        self.add_command_button(left_controls_vbox, "Step Up", PatternParameters.STEP_UP)

        left_controls_vbox.addStretch()
        controls_hbox.addLayout(left_controls_vbox, 1)

        # center coloumn
        center_controls_vbox = QVBoxLayout()
        center_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(center_controls_vbox, "Forward", PatternParameters.FORWARD)
        self.add_command_button(center_controls_vbox, "Backward", PatternParameters.BACKWARD)
        self.add_command_button(center_controls_vbox, "Step Over", PatternParameters.STEP_OVER)
        self.add_command_button(center_controls_vbox, "Center Feet", PatternParameters.FEET_REALIGN)
        self.add_command_button(center_controls_vbox, "Wide Stance", PatternParameters.WIDE_STANCE)

        center_controls_vbox.addStretch()
        controls_hbox.addLayout(center_controls_vbox, 1)

        # right coloumn
        right_controls_vbox = QVBoxLayout()
        right_controls_vbox.setContentsMargins(0,0,0,0)

        self.add_command_button(right_controls_vbox, "Rotate Right", PatternParameters.ROTATE_RIGHT)
        self.add_command_button(right_controls_vbox, "Strafe Right", PatternParameters.STRAFE_RIGHT)
        self.add_command_button(right_controls_vbox, "Step Down", PatternParameters.STEP_DOWN)

        right_controls_vbox.addStretch()
        controls_hbox.addLayout(right_controls_vbox, 1)

        # end control box
        add_layout_with_frame(vbox, controls_hbox, "Commands:")

        # start settings
        settings_hbox = QHBoxLayout()
        settings_hbox.setContentsMargins(0,0,0,0)

        # start left column
        left_settings_vbox = QVBoxLayout()
        left_settings_vbox.setContentsMargins(0,0,0,0)

        # frame id
        self.frame_id_line_edit = QLineEdit("odom")
        add_widget_with_frame(left_settings_vbox, self.frame_id_line_edit, "Frame ID:")

        # do closing step
        self.close_step_checkbox = QCheckBox()
        self.close_step_checkbox.setText("Do closing step")
        self.close_step_checkbox.setChecked(True)
        left_settings_vbox.addWidget(self.close_step_checkbox)

        # extra seperation
        self.ignore_robot_model_checkbox = QCheckBox()
        self.ignore_robot_model_checkbox.setText("Ignore Robot Model")
        self.ignore_robot_model_checkbox.setChecked(False)
        left_settings_vbox.addWidget(self.ignore_robot_model_checkbox)

        left_settings_vbox.addStretch()

        # number of steps
        self.step_number = generate_q_double_spin_box(1, 1, 100, 0, 1.0)
        add_widget_with_frame(left_settings_vbox, self.step_number, "Number Steps:")

        # start step index
        self.start_step_index = generate_q_double_spin_box(0, 0, 1000, 0, 1.0)
        add_widget_with_frame(left_settings_vbox, self.start_step_index, "Start Step Index:")

        # end left column
        settings_hbox.addLayout(left_settings_vbox, 1)

        # start center column
        center_settings_vbox = QVBoxLayout()
        center_settings_vbox.setContentsMargins(0,0,0,0)

        # start foot selection
        self.start_foot_selection_combo_box = QComboBox()
        self.start_foot_selection_combo_box.addItem("AUTO")
        add_widget_with_frame(center_settings_vbox, self.start_foot_selection_combo_box, "Start foot selection:")

        center_settings_vbox.addStretch()

        # step Distance
        self.step_distance = generate_q_double_spin_box(0.0, 0.0, 1.0, 2, 0.01)
        add_widget_with_frame(center_settings_vbox, self.step_distance, "Step Distance (m):")

        # side step distance
        self.side_step = generate_q_double_spin_box(0.0, 0.0, 1.0, 2, 0.01)
        add_widget_with_frame(center_settings_vbox, self.side_step, "Side Step (m):")

        # rotation per step
        self.step_rotation = generate_q_double_spin_box(0.0, -90.0, 90.0, 0, 1.0)
        add_widget_with_frame(center_settings_vbox, self.step_rotation, "Step Rotation (deg):")

        # end center column
        settings_hbox.addLayout(center_settings_vbox, 1)

        # start right column
        right_settings_vbox = QVBoxLayout()
        right_settings_vbox.setContentsMargins(0,0,0,0)

        # roll
        self.roll = generate_q_double_spin_box(0.0, -90.0, 90.0, 0, 1.0)
        add_widget_with_frame(right_settings_vbox, self.roll, "Roll (deg):")

        # pitch
        self.pitch = generate_q_double_spin_box(0.0, -90.0, 90.0, 0, 1.0)
        add_widget_with_frame(right_settings_vbox, self.pitch, "Pitch (deg):")

        # use terrain model
        self.use_terrain_model_checkbox = QCheckBox()
        self.use_terrain_model_checkbox.setText("Use Terrain Model")
        self.use_terrain_model_checkbox.setChecked(False)
        self.use_terrain_model_checkbox.stateChanged.connect(self.use_terrain_model_changed)
        right_settings_vbox.addWidget(self.use_terrain_model_checkbox)

        # override mode
        self.override_checkbox = QCheckBox()
        self.override_checkbox.setText("Override 3D")
        self.override_checkbox.setChecked(False)
        right_settings_vbox.addWidget(self.override_checkbox)

        right_settings_vbox.addStretch()

        # delta z
        self.dz = generate_q_double_spin_box(0.0, -0.5, 0.5, 2, 0.01)
        add_widget_with_frame(right_settings_vbox, self.dz, "delta z per step (m):")

        # end right column
        settings_hbox.addLayout(right_settings_vbox, 1)

        # end settings
        add_layout_with_frame(vbox, settings_hbox, "Settings:")

        # planner server topic selection
        self.planner_server_widget = QPlannerServerWidget(logger = self.logger)
        add_widget_with_frame(vbox, self.planner_server_widget, "Planner Topic:")

        # parameter set selection
        self.parameter_set_widget = QParameterSetWidget(logger = self.logger)
        add_widget_with_frame(vbox, self.parameter_set_widget, "Parameter Set:")

        # execute option
        if add_execute_widget:
            add_widget_with_frame(vbox, QExecuteStepPlanWidget(logger = self.logger, step_plan_topic = "step_plan"), "Execute:")

        # add error status widget
        add_widget_with_frame(vbox, error_status_widget, "Status:")

        # end widget
        self._widget.setLayout(vbox)
        self._widget.setObjectName('StepInterfaceUi')

        # set window title
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # add widget to the user interface
        context.add_widget(self._widget)

        # init widget
        self.planner_server_widget.topic_changed_signal.connect(self.planner_server_selected)
        self.parameter_set_widget.param_cleared_signal.connect(self.param_cleared)
        self.parameter_set_widget.param_changed_signal.connect(self.param_selected)

        self._check_command_button_condition()

    def shutdown_plugin(self):
        print("Shutting down ...")
        print("Done!")

    def add_command_button(self, parent, text, command):
        button = QPushButton(text)
        self.command_mapper.setMapping(button, command)
        button.clicked.connect(self.command_mapper.map)
        parent.addWidget(button)
        self.command_buttons.append(button)
        return button

    # message publisher
    def _publish_step_plan_request(self, walk_command):
        params = PatternParameters()
        params.steps                = self.step_number.value()
        params.mode                 = walk_command
        params.close_step           = self.close_step_checkbox.isChecked()
        params.ignore_robot_model   = self.ignore_robot_model_checkbox.isChecked()
        params.use_terrain_model    = self.use_terrain_model_checkbox.isChecked()
        params.override             = self.override_checkbox.isChecked() and not self.use_terrain_model_checkbox.isChecked()
        params.roll                 = math.radians(self.roll.value())
        params.pitch                = math.radians(self.pitch.value())
        params.dz                   = self.dz.value()

        params.step_distance_forward   = self.step_distance.value()
        params.step_distance_sideward  = self.side_step.value()
        params.turn_angle              = math.radians(self.step_rotation.value())

        request = StepPlanRequest()
        request.header = std_msgs.msg.Header()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = self.frame_id_line_edit.text().encode('ascii','ignore')
        request.start = list()
        request.start_step_idx = self.start_step_index.value()

        if self.start_foot_selection_combo_box.currentText() == "AUTO":
            request.start_foot_idx = StepPlanRequest.AUTO_START_FOOT_IDX
        elif self.start_foot_selection_combo_box.currentText() == "LEFT":
            request.start_foot_idx = 0
        elif self.start_foot_selection_combo_box.currentText() == "RIGHT":
            request.start_foot_idx = 1
        else:
            self.logger.log_error("Unknown start foot selection mode ('" + self.start_foot_selection_combo_box.currentText() + "')!")
            return

        if walk_command == PatternParameters.FORWARD:
            params.mode = PatternParameters.SAMPLING
        elif walk_command == PatternParameters.BACKWARD:
            params.mode                      = PatternParameters.SAMPLING
            params.step_distance_forward    *= -1
            params.step_distance_sideward   *= -1
            params.turn_angle               *= -1

        request.pattern_parameters = params
        request.planning_mode = StepPlanRequest.PLANNING_MODE_PATTERN
        request.parameter_set_name.data = self.parameter_set_widget.current_parameter_set_name()

        print("Send request = ", request)

        self.planner_server_widget.send_plan_request(request)

    def commands_set_enabled(self, enable):
        for button in self.command_buttons:
            button.setEnabled(enable)

    def _check_command_button_condition(self):
        enable = True

        if len(self.parameter_set_widget.current_parameter_set_name()) == 0:
            self.logger.log_error("No parameter set selected!")
            enable = False

        if not self.planner_server_widget.is_connected():
            self.logger.log_error("No connection to footstep planner request action server!")
            enable = False

        self.commands_set_enabled(enable)

    @Slot(str)
    def planner_server_selected(self, name):
        self._check_command_button_condition()

    @Slot()
    def param_cleared(self):
        self.commands_set_enabled(False)

    @Slot(str)
    def param_selected(self, name):
        self._check_command_button_condition()

    @Slot(int)
    def use_terrain_model_changed(self, state):
        enable_override = True
        if state == Qt.Checked:
            enable_override = False
        self.roll.setEnabled(enable_override)
        self.pitch.setEnabled(enable_override)
        self.override_checkbox.setEnabled(enable_override)
        self.dz.setEnabled(enable_override)
