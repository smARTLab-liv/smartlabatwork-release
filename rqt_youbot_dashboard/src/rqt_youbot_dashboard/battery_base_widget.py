import rospy
import roslib
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from std_srvs.srv import Empty


class BatteryBaseWidget(MenuDashWidget):
    def __init__(self, name):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        icons = []
        for i in range(6):
            icons.append([icons_path + 'battery-%d.png' % i])

        icons.append([icons_path + 'battery-stall.png'])
        icons.append([icons_path + 'battery-charging.png'])

        super(BatteryBaseWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self._name = name
        self._stall = True
        self.setToolTip("%s: stall" % (self._name)))
        self._signallevel = -1
        self._pct = -1
        self._voltage = -1
        self._time = -1
        self._charging = False

    def update_state(self, state):
        super(BatteryBaseWidget, self).update_state(state)
        if state is 5:
            self._stall = True
            self._signallevel = -1
            self.setToolTip("Wifi: stall")

    def close(self):
        pass

    def set_stall(self):
        self.update_state(5)

    def update(self, signallevel):
        self._stall = False
        self._signallevel = signallevel
        self.setToolTip("Wifi signal level: %.1f dBm" % signallevel)

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


class BatteryControl(wx.Window):
    def __init__(self, parent, id, icons_path, name):
        wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(48, 48))
        self.SetBackgroundColour(parent.GetBackgroundColour())

        self._bitmaps = []
        self._bitmap_stall = wx.Bitmap(path.join(icons_path, "battery-stall.png"), wx.BITMAP_TYPE_PNG)
        self._bitmap_charging = wx.Bitmap(path.join(icons_path, "battery-charging.png"), wx.BITMAP_TYPE_PNG)
        for i in range(6):
            self._bitmaps.append(wx.Bitmap(path.join(icons_path, "battery-%d.png" % i), wx.BITMAP_TYPE_PNG))

        self._stall = True
        self._name = name
        self.SetToolTip(wx.ToolTip("%s: stall" % (self._name)))

        self._pct = -1
        self._voltage = -1
        self._time = -1
        self._charging = False
        self.Bind(wx.EVT_PAINT, self.on_paint)


    def set_stall(self):
        self._stall = True
        self._pct = -1
        self._voltage = -1
        self._watt = -1
        self._charging = False
        self.SetToolTip(wx.ToolTip("%s: stall" % (self._name)))
        self.Refresh()

    def update(self, pct, voltage, watt, charging):
        self._stall = False
        self._pct = pct
        self._voltage = voltage
        self._watt = watt
        self._charging = charging
        tooltip = "%s: %.0f%% (%.2f V, %.2f W)" % (self._name, pct, voltage, watt)
        self.SetToolTip(wx.ToolTip(tooltip))
        self.Refresh()


    def on_paint(self, evt):
        dc = wx.BufferedPaintDC(self)
        dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
        dc.Clear()
        if self._stall or self._pct == -1 or self._voltage == -1:
            dc.DrawBitmap(self._bitmap_stall, 0, 0, True)
        else:
            idx = int((self._pct + 10.0) / 20.0)
            dc.DrawBitmap(self._bitmaps[idx], 0, 0, True)
            if self._charging:
                dc.DrawBitmap(self._bitmap_charging, 0, 0, True)
