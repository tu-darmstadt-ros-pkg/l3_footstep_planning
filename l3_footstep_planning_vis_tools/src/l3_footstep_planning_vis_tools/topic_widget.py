#!/usr/bin/env python

import rospy

from python_qt_binding.QtCore import Qt, QSize, QRegExp, Signal, Slot
from python_qt_binding.QtGui import QRegExpValidator, QIcon
from python_qt_binding.QtWidgets import QWidget, QHBoxLayout, QPushButton, QComboBox


# widget for topic selection
class QTopicWidget(QWidget):

    topic_changed_signal = Signal(str)

    def __init__(self, parent=None, topic_type=str(), is_action_topic=False):
        QWidget.__init__(self, parent)

        if is_action_topic:
            self.topic_type = topic_type + "Goal"
        else:
            self.topic_type = topic_type
        self.is_action_topic = is_action_topic

        # start widget
        hbox = QHBoxLayout()
        hbox.setContentsMargins(0, 0, 0, 0)

        # topic combo box
        self.topic_combo_box = QComboBox()
        self.topic_combo_box.setEnabled(False)
        self.topic_combo_box.blockSignals(True)
        self.topic_combo_box.setValidator(QRegExpValidator(QRegExp('((\d|\w|/)(?!//))*'), self))
        self.topic_combo_box.currentIndexChanged[str].connect(self.topic_changed)
        hbox.addWidget(self.topic_combo_box)

        # get system icon
        icon = QIcon.fromTheme("view-refresh")
        size = icon.actualSize(QSize(32, 32))

        # add refresh button
        refresh_topics_button = QPushButton()
        refresh_topics_button.clicked.connect(self.update_topic_list)
        refresh_topics_button.setIcon(icon)
        refresh_topics_button.setFixedSize(size.width()+2, size.height()+2)
        hbox.addWidget(refresh_topics_button)

        # end widget
        self.setLayout(hbox)

        # init widget
        self.update_topic_list()

    def emit_topic_name(self):
        self.topic_changed_signal.emit(self.current_topic())

    def set_editable(self, enable):
        self.topic_combo_box.setEditable(enable)

    def current_topic(self):
        if self.topic_combo_box.isEnabled():
            return self.topic_combo_box.currentText()
        else:
            return str()

    @Slot(str)
    def topic_changed(self, topic_name):
        self.topic_changed_signal.emit(topic_name)

    @Slot()
    def update_topic_list(self):
        self.topic_combo_box.clear()
        self.topic_combo_box.setEnabled(False)
        self.topic_combo_box.blockSignals(True)
        self.topic_combo_box.addItem('Updating...')

        # get topic list
        _, _, topic_type = rospy.get_master().getTopicTypes()
        topic_dict = dict(topic_type)
        # filter list
        topic_dict_filtered = dict()
        for k, v in topic_dict.items():
            if (len(topic_type) == 0) or (v == self.topic_type):
                if self.is_action_topic:
                    topic_dict_filtered[k[:-5]] = v
                else:
                    topic_dict_filtered[k] = v

        self.topic_combo_box.clear()
        self.topic_combo_box.addItems(sorted(topic_dict_filtered.keys()))

        if self.topic_combo_box.count() > 0:
            self.topic_combo_box.setEnabled(True)
            self.topic_combo_box.blockSignals(False)
            self.topic_changed(self.topic_combo_box.currentText())
        else:
            self.topic_combo_box.addItem('No topics available!')
