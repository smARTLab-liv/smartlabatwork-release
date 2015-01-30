import rospy
import roslib
import subprocess
from functools import partial
from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize
from std_srvs.srv import Empty


class ClearCostmapWidget(IconToolButton):
    def __init__(self):
        self._ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/costmap-green.png")]
        self._not_ok_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/costmap-grey.png")]

        icons = [self._ok_icon, self._not_ok_icon]
        super(ClearCostmapWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.setToolTip("Clear costmap")

        self.clicked.connect(self.toggle)

        try:
            rospy.wait_for_service('/move_base_node/clear_costmaps', 1)
            self._srv_costmap = rospy.ServiceProxy('/move_base_node/clear_costmaps', Empty)
            self.update_state(0)
            self._costmap_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self.update_state(1)
            self._costmap_ok = False

    def update_state(self, state):
        super(ClearCostmapWidget, self).update_state(state)
        pass

    def toggle(self):
        if not self._costmap_ok:
            self._srv_costmap = rospy.ServiceProxy('/move_base_node/clear_costmaps', Empty)

        try:
            self._srv_costmap()
            self.update_state(0)
            self._costmap_ok = True
            rospy.loginfo("Costmap cleared")
        except rospy.ServiceException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self.update_state(1)
            self._costmap_ok = False

    def close(self):
        try:
            self._srv_costmap.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister clear costmap service")
