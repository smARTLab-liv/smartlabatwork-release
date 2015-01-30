import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from pr2_msgs.msg import PowerBoardState
from std_srvs.srv import Empty


class ArmStatusWidget(MenuDashWidget):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        self._off_icon = [icons_path + 'arm-grey.png']
        self._green_icon = [icons_path + 'arm-green.png']
        self._yellow_icon = [icons_path + 'arm-yellow.png']
        self._red_icon = [icons_path + 'arm-red.png']

        icons = [self._off_icon, self._green_icon, self._yellow_icon, self._red_icon]
        super(ArmStatusWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.add_action('Disable Arm Motors', partial(self.disable_arm))
        self.add_action('Enable Arm Motors', partial(self.enable_arm))

        self.setToolTip("Arm Motors: Stale")
        self._platform_state_message = None

    def update_state(self, state):
        super(ArmStatusWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Arm Motors: Stale")
        elif state is 1:
            self.setToolTip("Arm Motors: Switched ON")
        elif state is 2:
            self.setToolTip("Arm Motors: Switched OFF")
        elif state is 3:
            self.setToolTip("Arm Motors: Not connected")
        else:
            self.setToolTip("Arm Motors: Stale")

    def close(self):
        pass

    def update_platform_state(self, msg):
        self._platform_state_message = msg
        self._last_platform_state_message = rospy.get_time()

        if (msg.circuit_state[0] == PowerBoardState.STATE_ENABLED):
            self.update_state(1)
        elif (msg.circuit_state[0] == PowerBoardState.STATE_STANDBY):
            self.update_state(2)
        elif (msg.circuit_state[0] == PowerBoardState.STATE_DISABLED):
            self.update_state(3)
        else:
            self.update_state(0)

    def set_stale(self):
        self.update_state(0)

    def enable_arm(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_STANDBY):
                switch_on_arm = rospy.ServiceProxy("/arm_1/switchOnMotors", Empty)

                try:
                    switch_on_arm()
                except rospy.ServiceException, e:
                    util.dasherr("Failed to switch ON arm motors: service call failed with error: %s" % (e), self)

            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_ENABLED):
                util.dasherr("Arm motors are already switched ON", self)
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_DISABLED):
                util.dasherr("Arm is not connected", self)

    def disable_arm(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_ENABLED):
                switch_off_arm = rospy.ServiceProxy("/arm_1/switchOffMotors", Empty)

                try:
                    switch_off_arm()
                except rospy.ServiceException, e:
                    util.dasherr("Failed to switch OFF arm motors: service call failed with error: %s" % (e), self)

            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_STANDBY):
                util.dasherr("Arm motors are already switched OFF", self)
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_DISABLED):
                util.dasherr("Arm is not connected", self)
