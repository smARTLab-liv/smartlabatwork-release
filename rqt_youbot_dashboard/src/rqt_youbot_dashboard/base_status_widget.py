import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from pr2_msgs.msg import PowerBoardState
from std_srvs.srv import Empty


class BaseStatusWidget(MenuDashWidget):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        self._off_icon = [icons_path + 'wheel-grey.png']
        self._green_icon = [icons_path + 'wheel-green.png']
        self._yellow_icon = [icons_path + 'wheel-yellow.png']
        self._red_icon = [icons_path + 'wheel-red.png']

        icons = [self._off_icon, self._green_icon, self._yellow_icon, self._red_icon]
        super(BaseStatusWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.add_action('Disable Base Motors', partial(self.disable_base))
        self.add_action('Enable Base Motors', partial(self.enable_base))

        self.setToolTip("Base Motors: Stale")
        self._platform_state_message = None

    def update_state(self, state):
        super(BaseStatusWidget, self).update_state(state)
        if state is 0:
            self.setToolTip("Base Motors: Stale")
        elif state is 1:
            self.setToolTip("Base Motors: Switched ON")
        elif state is 2:
            self.setToolTip("Base Motors: Switched OFF")
        elif state is 3:
            self.setToolTip("Base Motors: Not connected")
        else:
            self.setToolTip("Base Motors: Stale")

    def close(self):
        pass

    def update_platform_state(self, msg):
        self._platform_state_message = msg
        self._last_platform_state_message = rospy.get_time()

        # rospy.loginfo("Circuit state: %s", msg.circuit_state[0])

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

    def enable_base(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_STANDBY):
                switch_on_base = rospy.ServiceProxy("/base/switchOnMotors", Empty)

                try:
                    switch_on_base()
                except rospy.ServiceException, e:
                    util.dasherr("Failed to switch ON base motors: service call failed with error: %s" % (e), self)

            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_ENABLED):
                util.dasherr("Base motors are already switched ON", self)
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_DISABLED):
                util.dasherr("Base is not connected", self)

    def disable_base(self):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_ENABLED):
                switch_off_base = rospy.ServiceProxy("/base/switchOffMotors", Empty)

                try:
                    switch_off_base()
                except rospy.ServiceException, e:
                    util.dasherr("Failed to switch OFF base motors: service call failed with error: %s" % (e), self)

            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_STANDBY):
                util.dasherr("Base motors are already switched OFF", self)
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_DISABLED):
                util.dasherr("Base is not connected", self)
    
