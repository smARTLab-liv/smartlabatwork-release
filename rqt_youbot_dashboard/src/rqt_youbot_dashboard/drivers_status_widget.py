import rospy
import roslib
import subprocess
from functools import partial
from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_srvs.srv import Empty
from rqt_robot_dashboard import util


class DriversStatusWidget(IconToolButton):
    def __init__(self):
        self._not_ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/motor-grey.png")]
        self._ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/motor-green.png")]

        icons = [self._not_ok_icon, self._ok_icon]
        super(DriversStatusWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))
        self.update_state(0)

        self.setToolTip("Driver: Stale")

        self.clicked.connect(self.toggle)

        self._platform_state_message = None

    def update_state(self, state):
        super(DriversStatusWidget, self).update_state(state)
        pass

    def set_stale(self):
        self.update_state(0)

    def set_ok(self):
        self.update_state(1)
        self.setToolTip("Restart driver")

    def toggle(self):
        if (self._platform_state_message is not None):
            reconnect = rospy.ServiceProxy("/reconnect", Empty)
            try:
                reconnect()
            except rospy.ServiceException, e:
                util.dasherr("Failed to reconnect the driver: service call failed with error: %s" % (e))

    def close(self):
        try:
            self._srv_arm.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister reconnect drivers service")

    def update_platform_state(self, msg):
        self._platform_state_message = msg
        self._last_platform_state_message = rospy.get_time()
