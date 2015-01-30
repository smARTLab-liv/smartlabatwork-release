import rospy
import roslib
import subprocess
from functools import partial
from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_srvs.srv import Empty


class ArmTuckWidget(IconToolButton):
    def __init__(self):
        self._ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/arm-tuck-green.png")]
        self._not_ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/arm-tuck-grey.png")]

        icons = [self._ok_icon, self._not_ok_icon]
        super(ArmTuckWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))
        self.update_state(0)

        self.setToolTip("Tuck arm")

        self.clicked.connect(self.toggle)
        self._srv_arm = None

        try:
            rospy.wait_for_service('/tuck_arm', 1)
            self._srv_arm = rospy.ServiceProxy('/tuck_arm', Empty)
            self.update_state(0)
            self._arm_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("/tuck_arm service unavailable")
            self.update_state(1)
            self._arm_ok = False

    def update_state(self, state):
        super(ArmTuckWidget, self).update_state(state)
        pass

    def toggle(self):
        if not self._arm_ok:
            self._srv_arm = rospy.ServiceProxy('/tuck_arm', Empty)

        try:
            self._srv_arm()
            self._arm_ok = True
            rospy.loginfo("Arm tucked")
            self.update_state(0)
        except rospy.ServiceException, e:
            rospy.logwarn("/tuck_arm service unavailable")
            self._arm_ok = False
            self.update_state(1)

    def close(self):
        try:
            self._srv_arm.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister /tuck_arm service")
