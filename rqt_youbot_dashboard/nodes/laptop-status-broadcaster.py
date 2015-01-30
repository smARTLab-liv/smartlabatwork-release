#!/usr/bin/env python
import rospy
import commands
import std_msgs.msg
import psutil
from slaw_msgs.msg import SysInfo, BatteryStatus, SystemStatus, NetworkStatus
from pr2_msgs.msg import PowerBoardState
from random import randint

def laptop_status():
    pub = rospy.Publisher('dashboard/laptopstatus', SysInfo)
    rospy.init_node('laptopstatusbroadcaster')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():

	# BatteryStatus
	# possible states
	# discharging
	# charging
	# fully-charged
	charging = parseUpowerMessage(commands.getoutput("upower -i /org/freedesktop/UPower/devices/battery_BAT0| grep -E 'state'"))
	# is_charging = charging == "charging"
	# time_to_full = "" if not is_charging else parseUpowerMessage(commands.getoutput("upower -i /org/freedesktop/UPower/devices/battery_BAT0| grep -E 'to\ full'"))

	percentage = parseUpowerMessage(commands.getoutput("upower -i /org/freedesktop/UPower/devices/battery_BAT0| grep -E 'percentage'"))
	percentage = float(percentage[:-1])

	voltage = parseUpowerMessage(commands.getoutput("upower -i /org/freedesktop/UPower/devices/battery_BAT0| grep -E 'voltage'"))
	voltage = float(voltage[:-2])

	watt = parseUpowerMessage(commands.getoutput("upower -i /org/freedesktop/UPower/devices/battery_BAT0| grep -E 'energy' | grep -v '-'"))
	watt = float(watt[:-3])

	battery_msg = BatteryStatus()

	battery_msg.voltage = voltage
	battery_msg.watt = watt
	battery_msg.percent = percentage
	battery_msg.temp = 0
	battery_msg.plugged_in = charging == "charging" or charging == "fully-charged"

	# rospy.loginfo(battery_msg)

	# NetworkStatus
	network_msg = NetworkStatus()

	network = commands.getoutput("iwconfig | grep -E 'ESSID'")
	network = network[network.index("ESSID:") + 6:].rstrip().replace("\"", "")
	signal_level = getSignalLevel(network)
	network_msg.wifi_signallevel = signal_level

	# SystemStatus
	system_msg = SystemStatus()

	system_msg.cpu_usage_average = psutil.cpu_percent(interval=None)

	sysinfo_msg = SysInfo()
	sysinfo_msg.header = std_msgs.msg.Header()
	sysinfo_msg.header.stamp = rospy.Time.now()

	sysinfo_msg.hostname = commands.getoutput("hostname")

	sysinfo_msg.battery_pc = battery_msg
	sysinfo_msg.network = network_msg
	sysinfo_msg.system = system_msg

	pub.publish(sysinfo_msg)
	r.sleep()

def getSignalLevel(network):
    cells=[[]]
    parsed_cells=[]

    iwlist = commands.getoutput("iwlist scan")

    for line in iwlist.split("\n"):
        cell_line = match(line,"Cell ")
	#rospy.loginfo("%s MATCH: %s",line,cell_line)
        if cell_line != None:
            cells.append([])
            line = cell_line[-27:]
        cells[-1].append(line.rstrip())

    cells=cells[1:]

    for cell in cells:
        parsed_cells.append(parse_cell(cell))

    for cell in parsed_cells:
	if cell['Name'] == network:
	    signal = cell['Signal']
            signal = signal[:signal.index(" ")]
	    return float(signal)

    return -1

def parse_cell(cell):
    parsed_cell={}
    
    rules={
	"Name":get_name,
	"Signal":getCellSignal
    }
    for key in rules:
        rule=rules[key]
        parsed_cell.update({key:rule(cell)})
    return parsed_cell

def get_name(cell):
    return matching_line(cell,"ESSID:")[1:-1]

def getCellSignal(cell):
    try:
        return matching_line(cell,"Quality=").split('Signal level=')[1]
    except:
	return -1

def matching_line(lines, keyword):
    for line in lines:
        matching=match(line,keyword)
        if matching!=None:
            return matching
    return None

def match(line,keyword):
    line=line.lstrip()
    length=len(keyword)
    if line[:length] == keyword:
        return line[length:]
    else:
        return None

def parseUpowerMessage(message):
    try:
        return message[message.index(":") + 1:].lstrip()
    except ValueError:
	return message

if __name__ == '__main__':
    try:
        laptop_status()
    except rospy.ROSInterruptException:
        pass
