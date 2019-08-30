import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

import mavros
from mavros_msgs.srv import SetMode
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
		self._widget.button_flight_mode_set.clicked.connect(self.button_flight_mode_set_pressed)
		self._widget.button_update_namespace.clicked.connect(self.button_update_namespace_pressed)

		self._widget.combo_param_list.currentIndexChanged.connect(self.combo_param_list_pressed)

		self._widget.button_mixer_set.clicked.connect(self.button_mixer_set_pressed)

		self.flight_modes = sorted(["MANUAL",
						"ACRO",
						"ALTCTL",
						"POSCTL",
						"OFFBOARD",
						"STABILIZED",
						"RATTITUDE",
						"AUTO.MISSION",
						"AUTO.LOITER",
						"AUTO.RTL",
						"AUTO.LAND",
						"AUTO.RTGS",
						"AUTO.READY",
						"AUTO.TAKEOFF"])

		for fm in self.flight_modes:
			self._widget.combo_flight_mode_list.addItem(fm)

		#mavros.set_namespace("/mavros")
		self.update_namespace()

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		instance_settings.set_value('namespace', self._widget.textbox_namespace.text())
		instance_settings.set_value('mode_selection', self._widget.combo_flight_mode_list.currentText())

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		ns = str(instance_settings.value('namespace'))

		if ns:
			self._widget.textbox_namespace.setText(ns)

		self.update_namespace()

		mode = str(instance_settings.value('mode_selection'))
		if mode in self.flight_modes:
			self._widget.combo_flight_mode_list.setCurrentIndex(self.flight_modes.index(mode))



	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def button_safety_arm_pressed(self):
		self._arm(True)
		rospy.logdebug("Safety arm button pressed!")

	def button_safety_disarm_pressed(self):
		self._arm(False)
		rospy.logdebug("Safety disarm button pressed!")

	def button_param_refresh_pressed(self):
		param_received = 0

		try:
			param_received, param_list = param_get_all(False)
			rospy.logdebug("Parameters received: %s" % str(param_received))
		except IOError as e:
			rospy.logerr(e)

		self._widget.combo_param_list.clear()
		param_id_list = list()
		if param_received > 0:
			for p in param_list:
				param_id_list.append(p.param_id)

			for p in sorted(param_id_list):
				self._widget.combo_param_list.addItem(p)

	def button_param_set_pressed(self):
		param_id = str(self._widget.combo_param_list.currentText())
		val_str = str(self._widget.textbox_param_value.text())

		if '.' in val_str:
			val = float(val_str)
		else:
			val = int(val_str)

		try:
			rospy.loginfo(param_set(param_id, val))
		except IOError as e:
			rospy.logerr(e)

		rospy.logdebug("Set param button pressed!")

	def button_mixer_set_pressed(self):
		param_id = "SYS_AUTOSTART"
		mixer_str = str(self._widget.combo_mixer_list.currentText())
		mixer_val = 0;

		if mixer_str == "Generic Plane":
			mixer_val = 2100
		elif mixer_str == "Quadrotor x4":
			mixer_val = 4001
		elif mixer_str == "Quadrotor +4":
			mixer_val = 5001
		elif mixer_str == "Hexarotor x4":
			mixer_val = 6001

		try:
			rospy.loginfo(param_set(param_id, mixer_val))
		except IOError as e:
			rospy.logerr(e)

		rospy.logdebug("Set param button pressed!")

	def button_flight_mode_set_pressed(self):
		mode_req = str(self._widget.combo_flight_mode_list.currentText())

		try:
			set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)
			ret = set_mode(base_mode=0, custom_mode=mode_req)

			if not ret.mode_sent:
				rospy.logerr("Request failed. Check mavros logs")
		except (rospy.ServiceException, IOError) as e:
			rospy.logerr(e)

		rospy.logdebug("Set param button pressed!")

	def combo_param_list_pressed(self,):
		param_id = self._widget.combo_param_list.currentText()
		try:
			if param_id:
				self._widget.textbox_param_value.setText(str(param_get(param_id)))
			else:
				self._widget.textbox_param_value.setText("")
		except IOError as e:
			rospy.logerr(e)

	def button_update_namespace_pressed(self):
		self.update_namespace()
		rospy.logdebug("Update namespace button pressed!")

	def _arm(self,state):
		try:
			ret = command.arming(value=state)

			if not ret.success:
				rospy.logerr("Request failed. Check mavros logs")

			rospy.loginfo("Command result: %s" % str(ret.result))
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def update_namespace(self):
		ns = self._widget.textbox_namespace.text()
		mavros.set_namespace(ns)


