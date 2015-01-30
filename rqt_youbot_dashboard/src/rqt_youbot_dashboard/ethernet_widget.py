import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from std_srvs.srv import Empty


class EthernetWidget(MenuDashWidget):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        self._network_stall_icon = [icons_path + 'network-stall.png']
        self._network_up_icon = [icons_path + 'network-up.png']
        self._network_down_icon = [icons_path + 'network-down.png']

        icons = [self._network_stall_icon, self._network_up_icon, self._network_down_icon]

        super(EthernetWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self._stall = True
        self.setToolTip("Ethernet: stall")
        self._connected = False

    def update_state(self, state):
        super(EthernetWidget, self).update_state(state)
        if state is 0:
            self._connected = False
            self.setToolTip("Ethernet: stall")
        elif state is 1:
            self.setToolTip("Ethernet: up")
        elif state is 2:
            self.setToolTip("Ethernet: down")

    def close(self):
        pass

    def set_stall(self):
        self.update_state(0)

    def update(self, connected):
        self._stall = False
        self._connected = connected
        if self._connected:
            self.update_state(1)
        else:
            self.update_state(2)
