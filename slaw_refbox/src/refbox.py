#!/usr/bin/python
import rospy
import rosparam
from std_srvs.srv import Empty
# from refereeBoxClient import *
from os import getenv
import subprocess
import socket

# SERVER = "10.10.16.118" #"192.168.51.117" #"10.10.16.207"
SERVER = "192.168.2.2"
#SERVER = "10.10.16.167"
PORT = "11111"
TEAM = "smartlab@work"

MESSAGE = "connection Test"
errorPrinted = False

ON = True  # False
#ON = False

BNT_PLAN_FILE = getenv("HOME") + "/ros/src/smartlabatwork/slaw_bnt/config/plan.yaml"
BMT_PLAN_FILE = getenv("HOME") + "/ros/src/smartlabatwork/slaw_bmt/config/plan.yaml"
BTT_PLAN_FILE = getenv("HOME") + "/ros/src/smartlabatwork/slaw_btt/config/plan.yaml"
PPT_PLAN_FILE = getenv("HOME") + "/ros/src/smartlabatwork/slaw_ppt/config/plan.yaml"
CBT_PLAN_FILE = getenv("HOME") + "/ros/src/smartlabatwork/slaw_cbt/config/plan.yaml"


def parse_msg(msg):
    test = msg[0:msg.index("<")]
    goals = msg[msg.index("<") + 1:-1]  # remove closing bracket

    print "\nTEST: %s\n" % test

    if test == "BNT":
        parse_bnt(goals)

    if test == "BMT":
        parse_bmt(goals)

    if test == "BTT":
        parse_btt(goals)

    if test == "PPT":
        parse_ppt(goals)

    if test == "CBT":
        parse_cbt(goals)


def parse_ppt(goals):
    source_place = goals.split(",")[0]
    destination_place = goals[goals.find(")") + 2:]
    obj_list = goals[goals.find("(") + 1:goals.find(")")].split(",")

    # source location
    out = "locations:\n"
    out += "  - {name: %s, type: Pickup}\n" % source_place
    out += "\n"

    # Objects at location
    out += "%s:\n" % source_place
    out += "  objects:\n"
    for obj in obj_list:
        out += "    - {name: %s, destination_location: %s, type: PPT}\n" % (obj, destination_place)
    out += "\n"

    # destination location
    # for obj in obj_list:
    # out += "%s:\n"%obj
    # 	out += "  destination_location: %s\n"%destination_place
    # 	out += "\n"

    print out

    print "\nsaving plan to file (%s)...\n" % PPT_PLAN_FILE
    write_and_start(PPT_PLAN_FILE, out)


def parse_cbt(goals):
    source_place = goals

    # source location
    out = "locations:\n"
    out += "  - {name: %s, type: CBT}\n" % source_place
    out += "\n"

    print out

    print "\nsaving plan to file (%s)...\n" % CBT_PLAN_FILE

    write_and_start(CBT_PLAN_FILE, out)


def parse_bnt(goals):
    global ON
    goal_list = goals.split("(")  # split at "("
    del (goal_list[0])  # delete empty first item
    for i, g in enumerate(goal_list):
        goal_list[i] = g[0:-1]  # delete ")"

    out = "locations:\n"
    for g in goal_list:
        label, direction, sleep = tuple(g.split(","))
        out += "  - {name: %s, type: BNT, dir: %s, sleep: %s}\n" % (label, direction, sleep)

    # End location
    #out += "  - {name: D0, type: End}\n"

    print out

    print "\nsaving plan to file (%s)...\n" % BNT_PLAN_FILE
    write_and_start(BNT_PLAN_FILE, out)


def parse_bmt(goals):
    print goals

    goal_list = goals.split(",")

    initial = goal_list[0]
    source_place = goal_list[1]
    destination_place = goal_list[2]
    configuration = goal_list[3][0:goal_list[3].find("(")]
    object_list = goals[goals.find("(") + 1:goals.find(")")].split(",")
    final_place = goals[goals.find(")") + 2:len(goals)]

    print "_____________________________"
    print "initial: %s" % initial
    print "source_place: %s" % source_place
    print "destination_place: %s" % destination_place
    print "configuration: %s" % configuration
    print "objects:" % object_list
    for o in object_list:
        print "\t%s" % o
    print "final_place: %s" % final_place
    print "_____________________________"

    # Locations
    out = "locations:\n"
    out += "  - {name: %s, type: Pickup}\n" % source_place
    out += "  - {name: %s, type: End}\n" % final_place
    out += "\n"

    # Objects at locations with extra infos
    out += "%s:\n" % source_place
    out += "  objects:\n"
    for o in object_list:
        out += "    - {name: %s, destination_location: %s, type: Normal}\n" % (o, destination_place)
    out += "\n"

    # Object destionations
    # for o in object_list:
    # out += "%s:\n"%o
    # 	out += "  destination_location: %s\n"%destination_place
    # 	out += "\n"

    print out

    print "\nsaving plan to file (%s)...\n" % BMT_PLAN_FILE
    write_and_start(BMT_PLAN_FILE, out)


def parse_btt(msg):
    print msg
    msg = msg.strip()

    initial_msg, goal_msg = msg.split(';')
    initial_msg = initial_msg.strip()
    goal_msg = goal_msg.strip()

    initial_msg = initial_msg[17:-1]
    goal_msg = goal_msg[14:-1]

    print"\n"
    print initial_msg
    print goal_msg
    print"\n"

    initial_list = initial_msg.split('<')
    del (initial_list[0])
    for i, g in enumerate(initial_list):
        initial_list[i] = g[0:-1]

    goal_list = goal_msg.split('<')
    del (goal_list[0])
    for i, g in enumerate(goal_list):
        goal_list[i] = g[0:-1]

    # Initial states
    no_of_states = 0
    initial_states = {}
    for g in initial_list:
        state_name = g[0:g.index(',')]
        configuration = g[g.index(',') + 1:g.find("(")]
        obj = g[g.find("(") + 1:g.find(")")]
        if state_name in initial_states.keys():
            state = initial_states[state_name]
            state.append((obj, configuration))
        else:
            obj_list = []
            obj_list.append((obj, configuration))
            initial_states[state_name] = obj_list
            no_of_states += 1

    # Goal states
    goal_states = []
    for g in goal_list:
        name = g[0:g.index(',')]
        configuration = g[g.index(',') + 1:g.find("(")]
        obj = g[g.find("(") + 1:g.find(")")]
        objects = obj.split(",")

        goal_states.append((name, configuration, objects))

    destination = goal_states[0][0]

    # sort initial_states
    items_sorted = []
    item_to_sort = ""
    while (len(items_sorted) < no_of_states):
        no_of_items = 0
        for (name, obj_list) in initial_states.items():
            print "Name: %s" % name
            for obj in obj_list:
                print "Obj: %s" % obj[0]
                no_of_objects = len(obj[0].split(","))
                print "No of objects %s" % no_of_objects
            if not name in items_sorted:
                if no_of_objects > no_of_items:
                    item_to_sort = name
                    no_of_items = no_of_objects
        items_sorted.append(item_to_sort)
        print "Adding %s" % item_to_sort
        print "No of objects when adding %s" % no_of_items

    out = "locations:\n"
    for item in items_sorted:
        out += "  - {name: %s, type: Pickup}\n" % item
    out += "\n"

    # print initial_states.items()
    # Objects at locations
    for (name, object_list) in initial_states.items():
        out += "%s:\n" % name
        out += "  objects:\n"
        for obj in object_list:
            obj_name = obj[0]
            objects = obj_name.split(",")
            place_loc = ''
            for object_at_location in objects:
                # Object destinations
                print object_at_location, goal_states
                for (goal, configuration, objects_goal) in goal_states:
                    if object_at_location in objects_goal:
                        place_loc = goal
                        objects_goal.remove(object_at_location)
                        break
                place_type = 'Normal'
                #####
                # German Open 2014 Finals BTT hack
                if place_loc == 'S13':
                    place_type = 'PPT'

                out += "    - {name: %s, destination_location: %s, type: %s}\n" % (
                    object_at_location, place_loc, place_type)

    print "\n%s" % out

    print "\nsaving plan to file (%s)...\n" % BTT_PLAN_FILE
    write_and_start(BTT_PLAN_FILE, out)


def write_and_start(filename, buffer_string):
    f = open(filename, "w+")
    f.write(buffer_string)
    f.close()

    paramlist = rosparam.load_file(filename)
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)

    if ON:
        # send start command to smach!
        start = rospy.ServiceProxy("/start_SMACH", Empty)
        try:
            rospy.wait_for_service('/start_SMACH')
            start()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e



            # start launch file
            #process = subprocess.Popen(["roslaunch", "slaw_btt", "btt.launch"])
            #process.wait()
    else:
        process = subprocess.Popen(["ping", "127.0.0.1"])
        process.wait()


def obtainTaskSpecFromServer(ServerIP, ServerPort, TeamName):
    # context = zmq.Context()
    # connection_address = "tcp://" + ServerIP + ":" + ServerPort
    # print "Start connection to " + connection_address
    # # Socket to talk to server
    # print "Connecting to server..."
    # socket = context.socket(zmq.REQ)
    # socket.connect(connection_address)
    #
    # print "Sending request ..."
    # socket.send(TeamName)
    #
    # # Get the reply.
    # message = socket.recv()
    # socket.send("ACK")
    # socket.close()
    # print "Received message: ", message
    # return message
    while True:
        global errorPrinted
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((ServerIP, int(ServerPort)))
            errorPrinted = False
        except socket.error, err:
            if not errorPrinted:
                rospy.loginfo("Connection to %s:%s not succsesfull: %s", ServerIP, ServerPort, err)
                rospy.loginfo("retrying ...")
                errorPrinted = True
            rospy.sleep(1.)
        else:
            s.send(MESSAGE)
            waitForData(s)
        rospy.loginfo("Connected to %s : %s", ServerIP, ServerPort)


def waitForData(s):
    rospy.loginfo("Connection Successfull")
    BUFFER_SIZE = 1024
    while True:
        try:
            s.send("ALIVE")
            data = s.recv(BUFFER_SIZE)
            parse_msg(data)
        except socket.error, err:
            rospy.loginfo("Connection closed")
            s.close()
            rospy.sleep(2.)
            return


if __name__ == "__main__":
    rospy.init_node('slaw_refbox')
    obtainTaskSpecFromServer(SERVER, PORT, TEAM)
    # msg = "BTT<initialsituation(<S4,line(M20_100,R20)>);goalsituation(<S3,line(M20_100)><S5,line(R20)>)>"
    #msg = "BTT<initialsituation(<D1,line(M20_100,S40_40_B)><S1,line(M20_100)>);goalsituation(<D1,line(S40_40_B,M20_100)><D2,line(M20_100)>)>"
    #msg = "PPT<D1,(M20_100,F20_20_B,F20_20_B),D2>"
    #print msg
    #parse_msg(msg)
