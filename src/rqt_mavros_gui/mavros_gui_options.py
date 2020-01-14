import os

import rospkg
import rospy
from importlib import import_module

from python_qt_binding import loadUi
from python_qt_binding.QtCore import qWarning
from python_qt_binding.QtWidgets import QDialog, QLabel, QLineEdit, QComboBox, QCheckBox
from rospkg.rospack import RosPack
import roslib.message
from rqt_py_common.topic_dict import TopicDict

from geometry_msgs.msg import Quaternion

class SimpleSettingsDialog(QDialog):
	"""Simple dialog that can show multiple settings groups and returns their combined results."""

	def __init__(self, title='Options', description=None):
		super(SimpleSettingsDialog, self).__init__()
		self.setObjectName('SimpleSettingsDialog')

		rp = RosPack()
		ui_file = os.path.join(rp.get_path('rqt_mavros_gui'), 'resource', 'MAVROSGUIOptions.ui')
		loadUi(ui_file, self)

		self.setWindowTitle(title)

		if description is not None:
			self.add_label(description)

	def add_label(self, text):
		self.group_area.layout().addWidget(QLabel(text))

	def add_checkbox(self, name, checked, title=""):
		checkbox = QCheckBox()
		checkbox.setObjectName(name)
		checkbox.setCheckState(bool(checked))
		checkbox.setTristate(False)
		checkbox.setText(title)

		self.group_area.layout().addWidget(checkbox)

	def add_lineedit(self, name, text, title=""):
		if title:
			self.add_label(title)

		line = QLineEdit(text)
		line.setObjectName(name)

		self.group_area.layout().addWidget(line)

	def add_combobox(self, name, options, current, title="", on_change=None):
		if title:
			self.add_label(title)

		combo = QComboBox()
		combo.setObjectName(name)

		for op in options:
			combo.addItem(op)

		cur_ind = 0
		if current in options:
			cur_ind = options.index(current)
		combo.setCurrentIndex(cur_ind)

		if on_change is not None:
			combo.activated.connect(on_change)

		self.group_area.layout().addWidget(combo)

	def add_combobox_empty(self, name, title, curtext=""):
		if title:
			self.add_label(title)

		combo = QComboBox()
		combo.setObjectName(name)
		combo.addItem(curtext)

		self.group_area.layout().addWidget(combo)
		self.empty_combo = combo

	def getKey(self,item):
		return item[0]

	def add_topic_list(self, name, current, title):
		topics = sorted(rospy.get_published_topics(), key=self.getKey)

		self.topics = [["",""]]
		for t in topics:
			self.topics.append(t)

		topic_names = []
		for t in self.topics:
			topic_names.append(t[0])

		self.add_combobox(name, topic_names, str(current), title, self.topic_selected)

	def topic_selected(self, ind):
		if ind is not 0:
			curtext = self.empty_combo.currentText()

			msg_children = TopicDict().get_topics()[self.topics[ind][0]]["children"]
			children_list = self.recursive_list_children(msg_children)
			#print(children_list)

			self.empty_combo.clear()
			for c in children_list:
				self.empty_combo.addItem(c)

			cur_ind = 0
			if curtext in children_list:
				cur_ind = children_list.index(curtext)
			self.empty_combo.setCurrentIndex(cur_ind)
		else:
			self.empty_combo.clear()

	def recursive_list_children(self, parents):
		children = []

		for child in parents.keys():
			if parents[child]["children"]:
				children.append(child)

				grandchildren = self.recursive_list_children(parents[child]["children"])
				for grandchild in grandchildren:
					children.append(child + "/" + grandchild)
			else:
				children.append(child)

		return children

	def get_settings(self):
		"""Returns the combined settings from all settings groups as a list."""

		if self.exec_() == QDialog.Accepted:
			results = [["",""]]

			for item in self.group_area.children():
				name = item.objectName()

				if type(item) is QComboBox:
					r = [name, item.currentText()]

					if results is None:
						results = [r]
					else:
						results.append(r)
				elif type(item) is QLineEdit:
					r = [name, item.text()]

					if results is None:
						results = [r]
					else:
						results.append(r)
				elif type(item) is QCheckBox:
					r = [name, item.checkState()]

					if results is None:
						results = [r]
					else:
						results.append(r)

			del results[0]

			return results
		return None
