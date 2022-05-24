#!/usr/bin/env python

import rospy

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QHBoxLayout, QGroupBox, QTextEdit, QDoubleSpinBox


# generic helper to generate quickly QDoubleSpinBox
def generate_q_double_spin_box(default_val, range_min, range_max, decimals, single_step):
    spin_box = QDoubleSpinBox()
    spin_box.setValue(default_val)
    spin_box.setRange(range_min, range_max)
    spin_box.setDecimals(decimals)
    spin_box.setSingleStep(single_step)
    #spin_box.valueChanged[unicode].connect(self.callback_spin_box)
    return spin_box


# adds a layout with frame and text to parent widget
def add_layout_with_frame(parent, layout, text = ""):
    box_layout = QHBoxLayout()
    box_layout.addLayout(layout)

    group_box = QGroupBox()

    group_box.setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 4px; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")
    group_box.setTitle(text)
    group_box.setLayout(box_layout)

    parent.addWidget(group_box)


# adds a widget with frame and text to parent widget
def add_widget_with_frame(parent, widget, text = ""):
    box_layout = QHBoxLayout()
    box_layout.addWidget(widget)

    group_box = QGroupBox()

    group_box.setStyleSheet("QGroupBox { border: 1px solid gray; border-radius: 4px; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")
    group_box.setTitle(text)
    group_box.setLayout(box_layout)

    parent.addWidget(group_box)


# outputs message with given color at a QTextEdit
def output_message(text_edit, msg, color):
    text_edit.setTextColor(color)
    text_edit.append(msg)


# outputs error_status msg at QTextEdit field
def output_status(text_edit, error_status):
    if error_status.error != 0:
        output_message(text_edit, error_status.error_msg, Qt.red)

    if error_status.warning != 0:
        output_message(text_edit, error_status.warning_msg, QColor(255, 165, 0))
