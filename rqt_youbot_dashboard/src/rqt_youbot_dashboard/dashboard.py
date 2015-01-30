import roslib;

roslib.load_manifest('rqt_youbot_dashboard')
import rospy

import diagnostic_msgs

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, IconToolButton
from QtGui import QMessageBox, QAction
from PyQt4 import QtGui, QtCore
from python_qt_binding.QtCore import QSize

from pr2_msgs.msg import PowerBoardState
from slaw_msgs.msg import SysInfo, BatteryStatus

from .battery_widget import BatteryWidget
from .smach_widget import SmachWidget
from .base_status_widget import BaseStatusWidget
from .arm_status_widget import ArmStatusWidget
from .drivers_status_widget import DriversStatusWidget
from .arm_tuck_widget import ArmTuckWidget
from .location_widget import LocationWidget
from .cancel_goals_widget import CancelGoalsWidget
from .clear_costmap_widget import ClearCostmapWidget
from .wifi_widget import WifiWidget
from .ethernet_widget import EthernetWidget
from .cpu_widget import CpuWidget


class YoubotDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        self._last_platform_state_message = 0.0
        self._last_sysinfo_message = 0.0
        self._last_power_state_message = 0.0

        # SmachViewer
        self._smach_viewer = SmachWidget()

        # BaseStatusWidget
        self._base_status_widget = BaseStatusWidget()

        # ArmStatusWidget
        self._arm_status_widget = ArmStatusWidget()

        # DriversStatusWidget
        self._drivers_status_widget = DriversStatusWidget()

        # ArmTuckWidget
        self._arm_tuck_widget = ArmTuckWidget()

        # CancelGoalsWidget
        self._cancel_goals_widget = CancelGoalsWidget()

        # ClearCostmapWidget
        self._clear_costmap_widget = ClearCostmapWidget()

        # LocationWidget
        self._location_widget = LocationWidget()

        # Base Battery
        self._base_bat = BatteryWidget("Base")
        # Youbot PC Battery
        self._youbot_pc_bat = BatteryWidget("Youbot PC")

        # System Info
        # Wifi Widget
        self._wifi_widget = WifiWidget("Youbot PC")

        # Ethernet Widget
        self._ethernet_widget = EthernetWidget()

        # Cpu Widget
        #_self._cpu_widget = CpuWidget("Youbot PC")

        # Laptop Battery Widget
        self._laptop_bat = BatteryWidget("Laptop")
        # Laptop CPU Widget
        #self._laptop_cpu_widget = CpuWidget("Laptop")
        # Laptop Wifi Widget
        self._laptop_wifi_widget = WifiWidget("Laptop")

        # Subscribers
        # Platform State
        self._sub_platform_state = rospy.Subscriber('dashboard/platform_state', PowerBoardState,
                                                    self.update_platform_state)
        # System Info
        self._sub_sysinfo = rospy.Subscriber('dashboard/sysinfo', SysInfo, self.update_sysinfo)
        # Laptop Status
        self._sub_laptop_status = rospy.Subscriber('/dashboard/laptopstatus', SysInfo, self.update_laptop_status)

        # Diagnostics Aggregator
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray,
                                                   self.dashboard_callback)

        # Timer
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(500)

    def get_widgets(self):
        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context)],
                [self._smach_viewer],
                [self._base_status_widget, self._arm_status_widget],
                [self._drivers_status_widget],
                [self._arm_tuck_widget, self._cancel_goals_widget, self._clear_costmap_widget],
                [self._location_widget],
                [self._base_bat],
                [self._youbot_pc_bat, self._wifi_widget, self._ethernet_widget], #, self._cpu_widget],
                [self._laptop_bat, self._laptop_wifi_widget] #, self._laptop_cpu_widget]
        ]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()

        base_battery_status = {}
        for status in msg.status:
            if status.name == "/Battery/Base":
                for value in status.values:
                    base_battery_status[value.key] = value.value

        if (base_battery_status):
            percentage = float(base_battery_status['battery percentage'])
            self._base_bat.update_perc(percentage)
            self._base_bat.update_time(percentage)
            base_charging_state = True if base_battery_status['external power connected'] == 'yes' else False
            self._base_bat.set_charging(base_charging_state)

        youbot_pc_battery_status = {}
        for status in msg.status:
            if status.name == "/Battery/PC":
                for value in status.values:
                    youbot_pc_battery_status[value.key] = value.value

        if (youbot_pc_battery_status):
            percentage = float(youbot_pc_battery_status['Percentage'])
            self._youbot_pc_bat.update_perc(percentage)
            self._youbot_pc_bat.update_time(percentage)
            youbot_pc_charging_state = True if youbot_pc_battery_status['Charging'] == 'True' else False
            self._youbot_pc_bat.set_charging(youbot_pc_charging_state)

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()
        self._sub_platform_state.unregister()
        self._sub_sysinfo.unregister()
        self._sub_laptop_status.unregister()

    def update_platform_state(self, msg):
        self._last_platform_state_message = rospy.get_time()

        self._base_status_widget.update_platform_state(msg)
        self._arm_status_widget.update_platform_state(msg)
        self._drivers_status_widget.update_platform_state(msg)

    def update_sysinfo(self, msg):
        self._last_sysinfo_message = rospy.get_time()

        self._wifi_widget.update(msg.network.wifi_signallevel)
        self._ethernet_widget.update(msg.network.ethernet_connected)
        #self._cpu_widget.update(msg.system.cpu_usage_average, msg.system.cpu_temp_average)

    def update_laptop_status(self, msg):
        percentage = msg.battery_pc.percent
        self._laptop_bat.update_perc(percentage)
        self._laptop_bat.update_time(percentage)
        self._laptop_bat.set_charging(msg.battery_pc.plugged_in)
        #self._laptop_cpu_widget.update(msg.system.cpu_usage_average, msg.system.cpu_temp_average)
        self._laptop_wifi_widget.update(msg.network.wifi_signallevel)

    def stall_sysinfo(self):
        self._wifi_widget.set_stall()
        self._ethernet_widget.set_stall()
        #self._cpu_widget.set_stall()

    def on_timer(self):
        if (rospy.get_time() - self._last_sysinfo_message > 5.0):
            self.stall_sysinfo()

        if (rospy.get_time() - self._last_platform_state_message > 5.0):
            self._arm_status_widget.set_stale()
            self._base_status_widget.set_stale()
            self._drivers_status_widget.set_stale()

            ctrls = [self._arm_status_widget, self._base_status_widget, self._drivers_status_widget]
            for ctrl in ctrls:
                ctrl.setToolTip("No platform_state message received in the last 5 seconds")
        else:
            self._drivers_status_widget.set_ok()
