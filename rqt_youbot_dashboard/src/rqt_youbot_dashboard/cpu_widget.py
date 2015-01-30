import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize
from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QApplication, QLabel

from std_srvs.srv import Empty


class CpuWidget(MenuDashWidget, QtGui.QWidget):
    def __init__(self, name):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        self._name = name
        self._cpu_stall_icon = [icons_path + 'cpu-stall.png']
        self._cpu_bg_icon = [icons_path + 'cpu-bg.png']

        icons = [self._cpu_stall_icon, self._cpu_bg_icon]

        super(CpuWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.update_state(0)

    def update_state(self, state):
        super(CpuWidget, self).update_state(state)
        if state is 0:
            self._stall = True
            self._cpu_usage_hist = []
            self._cpu_usage = -1
            self.setToolTip("%s CPU: stall" % self._name)

    def close(self):
        pass

    def set_stall(self):
        self.update_state(0)

    def update(self, cpu_usage, cpu_temp):
        self._stall = False
        self._cpu_usage = cpu_usage
        self._cpu_usage_hist.insert(0, self._cpu_usage)
        if len(self._cpu_usage_hist) > 11:
            self._cpu_usage_hist.pop()
        self.setToolTip("%s CPU: %.0f%%, %.0fC" % (self._name, cpu_usage, cpu_temp))
        self.show()
        self.update_state(1)

    def draw(self, qp):
        color = QtGui.QColor(0, 0, 0)
        qp.fillRect(6, 6, 34, 30, color)
        for x in range(len(self._cpu_usage_hist)):
            pct = int(self._cpu_usage_hist[x] / 10.0)
            for y in range(pct):
                if y < 7:
                    color = QtGui.QColor(255, 0, 0)
                else:
                    color = QtGui.QColor(0, 255, 0)
                qp.setBrush(color)
                qp.drawRect(36 - x * 3, 36 - (y + 1) * 3, 2, 2)

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.draw(qp)
        qp.end()
