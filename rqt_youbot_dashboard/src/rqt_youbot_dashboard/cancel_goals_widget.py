import rospy
import roslib
import subprocess
from functools import partial
from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction


class CancelGoalsWidget(IconToolButton):
    def __init__(self):
        self._ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/cancel-goal-green.png")]
        self._not_ok_icon = [
            path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/cancel-goal-grey.png")]

        icons = [self._ok_icon, self._not_ok_icon]
        super(CancelGoalsWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.setToolTip("Cancel goals")

        self.clicked.connect(self.toggle)

        self._goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        if self._goal_client.wait_for_server(rospy.Duration(1)):
            self.update_state(0)
            self._goal_ok = True
        else:
            rospy.logwarn("move base not running yet, can't cancel goal")
            self.update_state(1)
            self._goal_ok = False

    def update_state(self, state):
        super(CancelGoalsWidget, self).update_state(state)
        pass

    def toggle(self):
        if not self._goal_ok:
            if self._goal_client.wait_for_server(rospy.Duration(1)):
                self.update_state(0)
                self._goal_ok = True
            else:
                rospy.logwarn("move base not running yet, can't cancel goal")
                self.update_state(1)
                self._goal_ok = False
                return

        self._goal_client.cancel_all_goals()
        rospy.loginfo("All goals cancelled")

    def close(self):
        try:
            self._goal_client.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister cancel goal service")
