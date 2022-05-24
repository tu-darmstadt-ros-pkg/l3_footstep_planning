#!/usr/bin/env python

import rospy
import l3_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt, QObject, Signal, Slot
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget

from l3_footstep_planning_msgs.msg import ErrorStatus


# Logging System
class Logger(QObject):

    out_log_signal = Signal(str, QColor)

    def __init__(self, error_status_text_box=None):
        super(Logger, self).__init__()
        self.connect(error_status_text_box)

    def connect(self, error_status_text_box):
        if type(error_status_text_box).__name__ == "QErrorStatusTextBox":
            self.out_log_signal.connect(error_status_text_box.out_log)
        elif type(error_status_text_box).__name__ == "QErrorStatusWidget":
            self.out_log_signal.connect(error_status_text_box.get_text_box().out_log)

    def log(self, error_status):
        if error_status.error != 0:
            self.log_error(error_status.error_msg)
        if error_status.warning != 0:
            self.log_warn(error_status.warning_msg)

    def log_info(self, msg):
        self.out_log_signal.emit("[ INFO] " + msg, Qt.black)
        rospy.loginfo(msg)

    def log_debug(self, msg):
        self.out_log_signal.emit("[DEBUG] " + msg, Qt.yellow)
        rospy.logdebug(msg)

    def log_warn(self, msg):
        self.out_log_signal.emit("[ WARN] " + msg, QColor(255, 165, 0))
        rospy.logwarn(msg)

    def log_error(self, msg):
        self.out_log_signal.emit("[ERROR] " + msg, Qt.red)
        rospy.logerr(msg)


# widget including logger
class QWidgetWithLogger(QWidget):

    logger = Logger()

    def __init__(self, parent=None, logger=Logger()):
        QWidget.__init__(self, parent)
        self.set_logger(logger)

    def set_logger(self, logger):
        self.logger = logger
