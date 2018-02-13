import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import mavros
from mavros.utils import *
from mavros.param import *
from mavros import command

class MAVROSGUI(Plugin):
	def __init__(self, context):
		super(MAVROSGUI, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MAVROSGUI')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		#from argparse import ArgumentParser
		#parser = ArgumentParser()
		# Add argument(s) to the parser.
		#parser.add_argument("-q", "--quiet", action="store_true",
		#              dest="quiet",
		#              help="Put plugin in silent mode")
		#args, unknowns = parser.parse_known_args(context.argv())
		#if not args.quiet:
		#    print 'arguments: ', args
		#    print 'unknowns: ', unknowns

		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_mavros_gui'), 'resource', 'MAVROSGUI.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MAVROSGUIUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._widget)

		self._widget.button_safety_arm.clicked.connect(self.button_safety_arm_pressed)
		self._widget.button_safety_disarm.clicked.connect(self.button_safety_disarm_pressed)
		self._widget.button_param_refresh.clicked.connect(self.button_param_refresh_pressed)
		self._widget.button_param_set.clicked.connect(self.button_param_set_pressed)
		self._widget.button_update_namespace.clicked.connect(self.button_update_namespace_pressed)

		self._widget.button_safety_arm.currentIndexChanged.connect(self.combo_param_list_pressed)

		#mavros.set_namespace("/mavros")
		self.update_namespace()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def button_safety_arm_pressed(self):
		_arm(True)
		rospy.loginfo("DEBUG: Safety arm button pressed!")

	def button_safety_disarm_pressed(self):
		_arm(False)
		rospy.loginfo("DEBUG: Safety disarm button pressed!")

	def button_param_refresh_pressed(self):
		param_received, param_list = param_get_all(False)
		rospy.loginfo("Parameters received:", param_received)
		rospy.loginfo("DEBUG: Refresh params button pressed!")

		self._widget.combo_param_list.clear()
		for p in sorted(param_list):
			self._widget.combo_param_list.addItem(p)

	def button_param_set_pressed(self):
		param_id = self._widget.combo_param_list.currentText()
		val_str = self._widget.textbox_param_value.text()

		if '.' in val_str:
			val = float(val_str)
		else:
			val = int(val_str)

		rospy.loginfo(param_set(param_id, val_str))
		rospy.loginfo("DEBUG: Set param button pressed!")

	def combo_param_list_pressed(self,):
		param_id = self._widget.combo_param_list.currentText()
		self._widget.textbox_param_value.setText(param_get(param_id))

	def button_update_namespace_pressed(self):
		self.update_namespace()
		rospy.loginfo("DEBUG: Update namespace button pressed!")

	def _arm(self,state):
		try:
			ret = command.arming(value=state)
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

		if not ret.success:
			rospy.logerr("Request failed. Check mavros logs")

		rospy.loginfo("Command result:", ret.result)

	def update_namespace(self):
		ns = self._widget.textbox_namespace.text()
		mavros.set_namespace(ns)

