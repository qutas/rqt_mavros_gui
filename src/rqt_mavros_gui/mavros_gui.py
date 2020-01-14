import os
import math
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from rqt_mavros_gui.mavros_gui_options import SimpleSettingsDialog

import mavros
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
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

		self.mavros_namespace = "/mavros"

		self._widget.button_safety_arm.clicked.connect(self.button_safety_arm_pressed)
		self._widget.button_safety_disarm.clicked.connect(self.button_safety_disarm_pressed)
		self._widget.button_param_refresh.clicked.connect(self.button_param_refresh_pressed)
		self._widget.button_param_set.clicked.connect(self.button_param_set_pressed)
		self._widget.button_flight_mode_set.clicked.connect(self.button_flight_mode_set_pressed)

		self._widget.combo_param_list.currentIndexChanged.connect(self.combo_param_list_pressed)

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

		# Handled in the restore settings callback
		#self.refresh_mavros_backend()

		self.sub_state = None
		self.timer_state = None
		self.msg_state_last = rospy.Time(0)

	def shutdown_plugin(self):
		if self.sub_state:
			self.sub_state.unregister()

		if self.timer_state:
			self.timer_state.shutdown()

	def save_settings(self, plugin_settings, instance_settings):
		instance_settings.set_value('namespace', self.mavros_namespace)
		instance_settings.set_value('mode_selection', self._widget.combo_flight_mode_list.currentText())

	def restore_settings(self, plugin_settings, instance_settings):
		ns = instance_settings.value('namespace')
		if ns:
			self.mavros_namespace = str(ns)

		mode = str(instance_settings.value('mode_selection'))
		if mode in self.flight_modes:
			self._widget.combo_flight_mode_list.setCurrentIndex(self.flight_modes.index(mode))

		self._refresh_mavros_backend()

	def trigger_configuration(self):
		"""Present the user with a dialog for choosing the topic to view,
		the data type, and other settings used to generate the HUD.
		This displays a SimpleSettingsDialog asking the user to choose
		the settings as desired.

		This method is blocking"""

		dialog = SimpleSettingsDialog(title='Quaternion View Options')
		dialog.add_lineedit("namespace", str(self.mavros_namespace), "Namespace")

		settings = dialog.get_settings();
		if settings is not None:
			for s in settings:
				if s[0] == "namespace":
					self.mavros_namespace = str(s[1])

		self._refresh_mavros_backend()

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
			self._widget.textbox_param_value.setEnabled(True)
			self._widget.button_param_set.setEnabled(True)

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

	def _arm(self,state):
		try:
			ret = command.arming(value=state)

			if not ret.success:
				rospy.logerr("Request failed. Check mavros logs")

			rospy.loginfo("Command result: %s" % str(ret.result))
		except rospy.ServiceException as ex:
			rospy.logerr(ex)

	def _cb_state(self, msg_in):
		self.msg_state_last = rospy.Time.now()

		if msg_in.connected:
			self._widget.label_status_live_conn.setText("Connected")
		else:
			self._widget.label_status_live_conn.setText("Disconnected")

		if msg_in.armed:
			self._widget.label_status_live_armed.setText("Armed")
			self._widget.label_status_live_armed.setStyleSheet("QLabel{color: rgb(255, 255, 255);background-color: rgb(25, 116, 2);}")
		else:
			self._widget.label_status_live_armed.setText("Disarmed")
			self._widget.label_status_live_armed.setStyleSheet("QLabel{color: rgb(255, 255, 255);background-color: rgb(116,2,25);}")

		if msg_in.mode == "CMODE(0)":
			self._widget.label_status_live_mode.setText("UNKNOWN")
		else:
			self._widget.label_status_live_mode.setText(msg_in.mode)

	def _state_monitor(self, event):
		if self.msg_state_last != rospy.Time(0):
			if (event.current_real - self.msg_state_last) > rospy.Duration(5):
				self.msg_state_last = rospy.Time(0)
				self._widget.label_status_live_conn.setText("Disconnected")
				self._widget.label_status_live_armed.setText("Disconnected")
				self._widget.label_status_live_armed.setStyleSheet("")
				self._widget.label_status_live_mode.setText("Disconnected")

	def _refresh_mavros_backend(self):
		mavros.set_namespace(self.mavros_namespace)

		if self.sub_state:
			self.sub_state.unregister()

		if self.timer_state:
			self.timer_state.shutdown()

		#Timer should trigger immidiately
		self.timer_state = rospy.Timer(rospy.Duration(1), self._state_monitor)
		self.sub_state = rospy.Subscriber(self.mavros_namespace + "/state", State, self._cb_state)

