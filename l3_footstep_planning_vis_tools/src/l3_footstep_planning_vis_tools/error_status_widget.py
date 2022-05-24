#!/usr/bin/env python

import rospy
import l3_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt, Signal, Slot
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton, QCheckBox, QSizePolicy

from l3_footstep_planning_msgs.msg import ErrorStatus
from l3_footstep_planning_vis_tools.logging import *


# widget for printing error status msgs to a text box
class QErrorStatusTextBox(QTextEdit):

    def __init__(self):
        QTextEdit.__init__(self)
        self.setReadOnly(True)

    @Slot(str, QColor)
    def out_log(self, msg, color=Qt.black):
        self.setTextColor(color)
        self.append(msg)

    def set_error_status(self, error_status):
        self.clear()
        self.append_error_status(error_status)

    def append_error_status(self, error_status):
        if error_status.error != 0:
            self.out_log(error_status.error_msg, Qt.red)

        if error_status.warning != 0:
            self.out_log(error_status.warning_msg, QColor(255, 165, 0))


# complex widget for printing error status msgs to a text box
class QErrorStatusWidget(QWidget):

    error_status_signal = Signal(ErrorStatus)

    def __init__(self, parent=None, subscribe=False):
        QWidget.__init__(self, parent)

        # start widget
        vbox = QVBoxLayout()
        vbox.setContentsMargins(0, 0, 0, 0)

        # add error status text edit
        self.error_status_text_box = QErrorStatusTextBox()
        self.error_status_text_box_layout = QHBoxLayout()
        self.error_status_text_box_layout.addWidget(self.error_status_text_box)
        vbox.addLayout(self.error_status_text_box_layout)

        # add panel
        hbox = QHBoxLayout()

        # clear push button
        self.execute_command = QPushButton("Clear")
        self.execute_command.clicked.connect(self.error_status_text_box.clear)
        hbox.addWidget(self.execute_command)

        hbox.addStretch()

        # hide window checkbox
        hide_window_check_box = QCheckBox("Hide")
        hide_window_check_box.stateChanged.connect(self.state_changed)
        hbox.addWidget(hide_window_check_box)

        # end panel
        vbox.addLayout(hbox)

        # end widget
        self.setLayout(vbox)
        #self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)

        # subscriber
        if subscribe:
            self.error_status_sub = rospy.Subscriber("error_status", ErrorStatus, self.error_status_callback)
        self.subscribed = subscribe

        # connect signal slot internally to prevent crash by subscriber
        self.error_status_signal.connect(self.append_error_status)

    def __del__(self):
        if self.subscribed:
            self.error_status_sub.unregister()

    def error_status_callback(self, error_status):
        self.error_status_signal.emit(error_status)

    def get_text_box(self):
        return self.error_status_text_box

    @Slot(str, QColor)
    def set_text(self, msg, color):
        self.error_status_text_box.clear()
        self.append(msg, color)

    @Slot(str, QColor)
    def append(self, msg, color):
        self.error_status_text_box.out_log(msg, color)

    @Slot(ErrorStatus)
    def set_error_status(self, error_status):
        self.error_status_text_box.set_error_status(error_status)

    @Slot(ErrorStatus)
    def append_error_status(self, error_status):
        self.error_status_text_box.append_error_status(error_status)

    @Slot(int)
    def state_changed(self, state):
        if state == Qt.Unchecked:
            self.error_status_text_box_layout.addWidget(self.error_status_text_box)
            self.error_status_text_box.setVisible(True)
        elif state == Qt.Checked:
            self.error_status_text_box_layout.removeWidget(self.error_status_text_box)
            self.error_status_text_box.setVisible(False)
