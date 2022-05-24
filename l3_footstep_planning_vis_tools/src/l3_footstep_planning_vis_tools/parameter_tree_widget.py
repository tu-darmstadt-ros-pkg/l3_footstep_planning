#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg
import l3_footstep_planning_msgs.msg

from python_qt_binding.QtCore import Qt, QPoint, QModelIndex, Slot
from python_qt_binding.QtGui import QColor, QIcon
from python_qt_binding.QtWidgets import QWidget, QTreeWidget, QTreeWidgetItem, QMenu, QAction, QAbstractItemView

from vigir_generic_params.msg import ParameterSetMsg
from l3_footstep_planning_msgs.parameter import *
from l3_footstep_planning_vis_tools.logging import *


class QParameterTreeWidget(QTreeWidget):

    logger = Logger()

    def __init__(self, parent=None, logger=Logger()):
        QTreeWidget.__init__(self, parent)
        self.set_logger(logger)

        # init tree
        self.setHeaderLabels(["Name", "Type", "Value"])
        self.sortItems(0, Qt.AscendingOrder)
        #self.setSelectionMode(QAbstractItemView.NoSelection)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.itemActivated.connect(self.edit_item)
        self.currentItemChanged.connect(self.current_item_changed)

        # context menu
        self.customContextMenuRequested.connect(self.context_menu_request)
        self._action_item_expand = QAction(QIcon.fromTheme('zoom-in'), 'Expand Selected', self)
        self._action_item_expand.triggered.connect(self._handle_action_item_expand)
        self._action_item_collapse = QAction(QIcon.fromTheme('zoom-out'), 'Collapse Selected', self)
        self._action_item_collapse.triggered.connect(self._handle_action_item_collapse)
        self._action_item_add = QAction(QIcon.fromTheme('list-add'), 'Add', self)
        self._action_item_add.setEnabled(False)  # TODO
        self._action_item_remove = QAction(QIcon.fromTheme('list-remove'), 'Remove', self)
        self._action_item_remove.setEnabled(False)  # TODO

    def set_logger(self, logger):
        self.logger = logger

    @Slot(QPoint)
    def context_menu_request(self, point):
        if self.selectionModel().hasSelection():
            menu = QMenu(self)
            menu.addAction(self._action_item_add)
            menu.addAction(self._action_item_remove)
            menu.addSeparator()
            menu.addAction(self._action_item_expand)
            menu.addAction(self._action_item_collapse)
            menu.exec_(self.mapToGlobal(point))

    @Slot()
    def _handle_action_item_collapse(self):
        self._handle_action_set_expanded(False)

    @Slot()
    def _handle_action_item_expand(self):
        self._handle_action_set_expanded(True)

    @Slot(bool)
    def _handle_action_set_expanded(self, expanded):
        def recursive_set_expanded(index):
            if (index != QModelIndex()) and (index.column() == 0):
                self.setExpanded(index, expanded)
                #for i in range(index.model().childCount()):
                #    index.model().child(i).setExpanded(expanded)
                #for i in range(index.model().rowCount()):
                #    recursive_set_expanded(index.child(i, 0))

        for index in self.selectedIndexes():
            recursive_set_expanded(index)

    @Slot(QTreeWidgetItem, int)
    def edit_item(self, item, column):
        if (column == 0) or (item.is_leaf() and (column == 2)):
            item.setFlags(item.flags() | Qt.ItemIsEditable)
            self.editItem(item, column)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)

    @Slot(QTreeWidgetItem, QTreeWidgetItem)
    def current_item_changed(self, prev, current):
        if prev is not None:
            if not prev.update_value():
                self.logger.log_error("Couldn't update value for '" + prev.get_name() + "' in '" + prev.get_namespace() + "'. Check input syntax!")
        if current is not None:
            if not current.update_value():
                self.logger.log_error("Couldn't update value for '" + current.get_name() + "' in '" + current.get_namespace() + "'. Check input syntax!")

    # set parameter set from msg
    def set_parameter_set(self, param_set_msg):
        self.clear()

        self.root = QParameterTreeWidgetItem(self, self.logger, name=param_set_msg.name.data)
        self.root.setExpanded(True)

        for p in param_set_msg.params:
            self.root.add_param(Parameter(msg=p))

    # get parameter set as msg
    def get_parameter_set(self):
        params = self.root.get_params()

        # remove top-level namespace
        top_len = len(self.root.get_name()) + 2
        for p in params:
            p.set_name(p.get_name()[top_len:])

        # generate msg
        param_set_msg = ParameterSet()
        param_set_msg.name.data = self.root.get_name()
        for p in params:
            param_set_msg.parameters.append(p.to_msg())

        return param_set_msg


class QParameterTreeWidgetItem(QTreeWidgetItem):

    logger = Logger()
    param = None
    namespace = str()

    def __init__(self, parent=None, logger=Logger(), param = None, name=str(), namespace=str()):
        QTreeWidgetItem.__init__(self, parent)
        self.set_logger(logger)

        if param is not None:
            self.set_param(param)
        elif len(name) > 0:
            self.set_name(name)
        self.namespace = namespace

    def set_logger(self, logger):
        self.logger = logger

    def is_leaf(self):
        return self.param is not None

    def update_value(self):
        result = True
        if self.is_leaf():
            self.param.set_name(self.text(0))
            result = self.param.set_value(self.text(2))
            #self.setText(2, str(self.param.get_value()))
            if result:
                for i in range(self.columnCount()):
                    self.setBackgroundColor(i, Qt.white)
            else:
                for i in range(self.columnCount()):
                    self.setBackgroundColor(i, QColor(255, 165, 0))
        return result

    def set_name(self, name):
        self.setText(0, name)
        if self.is_leaf():
            self.param.set_name(name)

    def get_name(self):
        return self.text(0)

    def get_namespace(self):
        return self.namespace

    def set_param(self, param, namespace = str()):
        self.param = param
        self.namespace = namespace
        if self.is_leaf():
            self.setText(0, self.param.get_name())
            self.setText(1, str(self.param.get_type_as_text()))
            self.setText(2, str(self.param.get_value()))

    def get_param(self):
        return self.param

    def get_params(self):
        params = []
        if self.is_leaf():
            # sync with recent changes from UI
            if not self.update_value():
                self.logger.log_warn("Value for '" + self.get_name() + "' in '" + self.get_namespace() + "' has invalid changes. Check input syntax!")

            p = self.get_param()
            p.set_name(self.get_namespace() + '/' + p.get_name())
            params.append(p)
        else:
            for i in range(self.childCount()):
                params.extend(self.child(i).get_params())
        return params

    # adds param recursively (based on namespace)
    def add_param(self, param):
        # finds branch matching the name
        def find_branch(name):
            for i in range(self.childCount()):
                if self.child(i).text(0) == name:
                    return self.child(i)
            return None

        # adds param to existing branch or create new branch recursively
        def add_param_to_branch(param, branch_name):
            branch = find_branch(branch_name)
            if branch is None:
                branch = QParameterTreeWidgetItem(self, self.logger, name = branch_name, namespace = self.get_namespace() + '/' + self.get_name())
            branch.add_param(param)

        if self.is_leaf():
            print("Error: Can't add parameter '" + param.get_name() + "' to the leaf '" + self.get_name() + "'!")
            return

        # check if param has namespace
        param.set_name(param.get_name().strip('/'))
        i = param.get_name().find('/')

        # add as leaf
        if i == -1:
            QParameterTreeWidgetItem(self, self.logger, param, namespace = self.get_namespace() + '/' + self.get_name())
        # add into branch
        else:
            branch_name = param.get_name()[:i]
            param.set_name(param.get_name()[i+1:])
            add_param_to_branch(param, branch_name)
