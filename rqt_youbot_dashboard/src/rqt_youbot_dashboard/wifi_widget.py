import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from std_srvs.srv import Empty


class WifiWidget(MenuDashWidget):
    def __init__(self, name):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        icons = []
        for i in range(5):
            icons.append([icons_path + 'wifi-%d.png' % i])

        icons.append([icons_path + 'wifi-stall.png'])

        super(WifiWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self._name = name
        self._stall = True
        self.setToolTip("%s Wifi: stall" % self._name)
        self._signallevel = -1

    def update_state(self, state):
        super(WifiWidget, self).update_state(state)
        if state is 5:
            self._stall = True
            self._signallevel = -1
            self.setToolTip("%s Wifi: stall" % self._name)

    def close(self):
        pass

    def set_stall(self):
        self.update_state(5)

    def update(self, signallevel):
        self._stall = False
        self._signallevel = signallevel
        self.setToolTip("%s Wifi signal level: %.1f dBm" % (self._name, signallevel))

        if self._stall or self._signallevel == -1:
            self.update_state(5)
        else:
            level = - self._signallevel
            if level < 30:
                level = 30
            if level > 95:
                level = 95
            perc = 100 - (level - 30) * 100.0 / 65.0
            idx = int((perc + 12.5) / 25.0)
            self.update_state(idx)
