import rospy
import roslib
import subprocess
from functools import partial
from os import path
import actionlib
from rqt_robot_dashboard.widgets import IconToolButton
from python_qt_binding.QtCore import QSize


class SmachWidget(IconToolButton):
    def __init__(self):
        self._off_icon = [path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/smach-grey.png")]

        icons = [self._off_icon]
        super(SmachWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))
        super(SmachWidget, self).update_state(0)

        self.setToolTip("Smach Viewer")

        self.clicked.connect(self.toggle)

    def update_state(self, state):
        pass

    def toggle(self):
        subprocess.call(["rosrun", "smach_viewer", "smach_viewer.py"])

    def close(self):
        pass
