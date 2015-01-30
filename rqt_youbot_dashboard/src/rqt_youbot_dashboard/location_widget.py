import rospy
import roslib
import sys
from functools import partial
from os import path
from rqt_robot_dashboard.widgets import MenuDashWidget
from rqt_robot_dashboard import util
from python_qt_binding.QtCore import QSize

from std_srvs.srv import Empty

from PyQt4 import QtGui
from formlayout import fedit
from slaw_srvs.srv import *


class LocationWidget(MenuDashWidget):
    def __init__(self):
        icons_path = path.join(roslib.packages.get_pkg_dir('rqt_youbot_dashboard'), "icons/")

        self._ok_icon = [icons_path + 'nav-green.png']
        self._not_ok_icon = [icons_path + 'nav-grey.png']

        icons = [self._ok_icon, self._not_ok_icon]
        super(LocationWidget, self).__init__('', icons=icons)
        self.setFixedSize(QSize(40, 40))

        self.add_action('Save current location', partial(self.save_current_location))
        self.add_action('Delete location', partial(self.delete_location))
        self.add_action('Show locations', partial(self.show_locations))

        self.setToolTip("Locations")
        self._srv_store_location = None
        self._srv_delete_location = None
        self._srv_get_location = None
        try:
            rospy.wait_for_service('location_service/store_location', 1)
            self._srv_store_location = rospy.ServiceProxy('location_service/store_location', StoreLocation)
            self._srv_delete_location = rospy.ServiceProxy('location_service/delete_location', DeleteLocation)
            self._srv_get_location = rospy.ServiceProxy('location_service/get_location', GetLocation)
            self.update_state(0)
            self._location_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("location service unavailable")
            self.update_state(1)
            self._location_ok = False

    def update_state(self, state):
        super(LocationWidget, self).update_state(state)

    def close(self):
        try:
            self._srv_store_location.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister store location service")

        try:
            self._srv_delete_location.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister delete location service")

        try:
            self._srv_get_location.unregister()
        except AttributeError, e:
            rospy.logwarn("Failed to unregister get location service")

    def save_current_location(self):
        location_alignments = ['N', 'W', 'S', 'E']
        location_pickup_sides = ['left', 'front']
        datalist = [
            ('Location Name', ''),
            ('Location Align', [0] + location_alignments),
            ('Location Pickup Side', [0] + location_pickup_sides)
        ]

        result = fedit(datalist, title="New location", comment="")
        if result is None:
            return

        name = result[0]
        alignment = location_alignments[result[1]]
        pickup_side = location_pickup_sides[result[2]]

        if self._srv_store_location is None:
            self._srv_store_location = rospy.ServiceProxy('location_service/store_location', StoreLocation)

        try:
            result = self._srv_store_location(name, alignment, pickup_side)
            self.update_state(0)
            self._location_ok = True
            if result.success:
                rospy.loginfo("Location %s stored", name)
            else:
                rospy.logwarn("Storing location %s failed: %s", name, result.reason)
        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self.update_state(1)
            self._location_ok = False

    def delete_location(self):
        datalist = [
            ('Location Name', '')
        ]

        result = fedit(datalist, title="Delete location", comment="Leave empty to delete all locations")
        if result is None:
            return

        name = result[0]

        if self._srv_delete_location is None:
            self._srv_delete_location = rospy.ServiceProxy('location_service/delete_location', DeleteLocation)

        try:
            result = self._srv_delete_location(name)
            self.update_state(0)
            self._location_ok = True
            if result.success:
                rospy.loginfo(result.reason)
            else:
                rospy.logwarn("Deleting location %s failed: %s", name, result.reason)
        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self.update_state(1)
            self._location_ok = False

    def show_locations(self):
        if self._srv_get_location is None:
            self._srv_get_location = rospy.ServiceProxy('location_service/get_location', GetLocation)

        try:
            result = self._srv_get_location("")
            self.update_state(0)
            self._location_ok = True

            message = "Name:\t\t(x, y)\n=============\n\n"
            for location in result.locations:
                message = message + "%s:\t\t\t(%.2f, %.2f)\n" % (
                    location.name, location.pose.position.x, location.pose.position.y)

            util.dashinfo(message, self)

        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self.update_state(1)
            self._location_ok = False


class InputDialog(QtGui.QWidget):
    def __init__(self):
        super(InputDialog, self).__init__()

        self.initUI()

    def initUI(self):
        # self.btn = QtGui.QPushButton('Dialog', self)
        # self.btn.move(20, 20)
        #self.btn.clicked.connect(self.showDialog)

        self.le = QtGui.QLineEdit(self)
        self.le.move(130, 22)

        self.le2 = QtGui.QLineEdit(self)
        self.le2.move(130, 52)

        self.setGeometry(300, 300, 290, 150)
        self.setWindowTitle('Store location')
        self.showDialog()

    def showDialog(self):
        text, ok = QtGui.QInputDialog.getText(self, 'Input Dialog',
                                              'Enter your name:')

        if ok:
            self.le.setText(str(text))
    
