#!/usr/bin/env python

import math

import rospy
import tf
import std_msgs.msg

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, Slot, QAbstractListModel
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QDoubleSpinBox, QFrame

from l3_footstep_planning_msgs.msg import PatternGeneratorParameters
from l3_footstep_planning_vis_tools.parameter_set_widget import *
from l3_footstep_planning_vis_tools.qt_helper import *
from l3_footstep_planning_vis_tools.logging import *


class PatternGeneratorDialog(Plugin):

    def __init__(self, context):
        super(PatternGeneratorDialog, self).__init__(context)
        self.setObjectName('PatternGeneratorDialog')

        self._parent = QWidget()
        self._widget = PatternGeneratorWidget(self._parent)

        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class PatternGeneratorWidget(QObject):

    enable_pattern_generator = False

    def __init__(self, context):
        super(PatternGeneratorWidget, self).__init__()

        # publisher
        self.pattern_generator_params_pub = rospy.Publisher('pattern_generator/set_params', PatternGeneratorParameters, queue_size = 1)

        # start widget
        widget = context

        # start upper part
        hbox = QHBoxLayout()

        # start left column
        left_vbox = QVBoxLayout()

        # start button
        start_command = QPushButton("Start")
        left_vbox.addWidget(start_command)

        # simulation checkbox
        self.simulation_mode_checkbox = QCheckBox()
        self.simulation_mode_checkbox.setText("Simulation Mode")
        self.simulation_mode_checkbox.setChecked(False)
        left_vbox.addWidget(self.simulation_mode_checkbox)

        # realtime checkbox
        self.realtime_mode_checkbox = QCheckBox()
        self.realtime_mode_checkbox.setText("Realtime Mode")
        self.realtime_mode_checkbox.setChecked(False)
        left_vbox.addWidget(self.realtime_mode_checkbox)

        # joystick checkbox
        self.joystick_mode_checkbox = QCheckBox()
        self.joystick_mode_checkbox.setText("Joystick Mode")
        self.joystick_mode_checkbox.setChecked(False)
        left_vbox.addWidget(self.joystick_mode_checkbox)

        # ignore invalid steps checkbox
        self.ignore_invalid_steps_checkbox = QCheckBox()
        self.ignore_invalid_steps_checkbox.setText("Ignore Invalid Steps")
        self.ignore_invalid_steps_checkbox.setChecked(True)
        left_vbox.addWidget(self.ignore_invalid_steps_checkbox)

        # foot seperation
        self.foot_seperation = generate_q_double_spin_box(0.2, 0.15, 0.3, 2, 0.01)
        self.foot_seperation.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.foot_seperation, "Foot Seperation (m):")

        # delta x
        self.delta_x = generate_q_double_spin_box(0.0, -0.4, 0.4, 2, 0.01)
        self.delta_x.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.delta_x, "dX (m):")

        # delta y
        self.delta_y = generate_q_double_spin_box(0.0, -2.2, 2.2, 2, 0.01)
        self.delta_y.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.delta_y, "dY (m):")

        # delta yaw
        self.delta_yaw = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        self.delta_yaw.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.delta_yaw, "dYaw (deg):")

        # roll
        self.roll = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        self.roll.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.roll, "Roll (deg):")

        # pitch
        self.pitch = generate_q_double_spin_box(0.0, -30.0, 30.0, 0, 1.0)
        self.pitch.valueChanged.connect(self.callback_spin_box)
        add_widget_with_frame(left_vbox, self.pitch, "Pitch (deg):")

        # end left column
        left_vbox.addStretch()
        hbox.addLayout(left_vbox, 1)

        # start right column
        right_vbox = QVBoxLayout()

        # stop button
        stop_command = QPushButton("Stop")
        right_vbox.addWidget(stop_command)

        # ignore collision
        self.collision_checkbox = QCheckBox()
        self.collision_checkbox.setText("Ignore Collision")
        self.collision_checkbox.setChecked(True)
        right_vbox.addWidget(self.collision_checkbox)

        # override 3D
        self.override_checkbox = QCheckBox()
        self.override_checkbox.setText("Override 3D")
        self.override_checkbox.setChecked(False)
        right_vbox.addWidget(self.override_checkbox)

        # end right coloumn
        right_vbox.addStretch()
        hbox.addLayout(right_vbox, 1)

        # add upper part
        hbox.setContentsMargins(0,0,0,0)
        vbox = QVBoxLayout()
        vbox.addLayout(hbox)

        # parameter set selection
        self.parameter_set_widget = QParameterSetWidget()
        add_widget_with_frame(vbox, self.parameter_set_widget, "Parameter Set:")

        # end widget
        widget.setLayout(vbox)
        #context.add_widget(widget)

        # signal connections
        start_command.clicked.connect(self.start_command_callback)
        stop_command.clicked.connect(self.stop_command_callback)
        self.joystick_mode_checkbox.clicked.connect(self.joystick_mode_check_callback)
        self.ignore_invalid_steps_checkbox.clicked.connect(self._publish_parameters)

    def shutdown_plugin(self):
        print("Shutting down ...")
        self.pattern_generator_params_pub.unregister()
        print("Done!")

    # message publisher
    def _publish_parameters(self):
        params = PatternGeneratorParameters()

        params.enable = self.enable_pattern_generator
        params.simulation_mode = self.simulation_mode_checkbox.isChecked()
        params.joystick_mode = self.joystick_mode_checkbox.isChecked()
        params.ignore_invalid_steps = self.ignore_invalid_steps_checkbox.isChecked()
        params.cmd.linear.x = self.delta_x.value()
        params.cmd.linear.y = self.delta_y.value()
        params.cmd.linear.z = 0
        params.cmd.angular.x = math.radians(self.roll.value())
        params.cmd.angular.y = math.radians(self.pitch.value())
        params.cmd.angular.z = math.radians(self.delta_yaw.value())
        params.foot_seperation = self.foot_seperation.value()
        params.parameter_set_name.data = self.parameter_set_widget.current_parameter_set_name()

        print("Send stepping command = \n",params)
        self.pattern_generator_params_pub.publish(params)

    # Define system command strings
    def start_command_callback(self):
        self.enable_pattern_generator = True
        self._publish_parameters()

    def stop_command_callback(self):
        self.enable_pattern_generator = False
        self._publish_parameters()

    def callback_spin_box(self, value_as_int):
        if self.realtime_mode_checkbox.isChecked():
            self._publish_parameters()

    def joystick_mode_check_callback(self):
        self.enable_pattern_generator = False
        self._publish_parameters()

