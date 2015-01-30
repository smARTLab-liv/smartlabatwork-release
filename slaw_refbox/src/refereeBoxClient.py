#!/usr/bin/python
#
# Python Client
#   Needs the tcp//... address of the server as argument 
#	(e.g. $ pyton2.7 hwclient.py tcp://localhost.5555
#   Sends the robot name ( in our case b-it-bots) 
#   and waits for reply from the server
#	
import zmq
import sys
import socket
import rospy

MESSAGE = "connection Test"
errorPrinted = False

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
        try:
           socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
           socket.connect((ServerIP, ServerPort))
           errorPrinted = False
        except socket.error, err:
           if not errorPrinted:
               rospy.loginfo("Connection to %s:%s not succsesfull: %s", ServerIP, ServerPort, err)
               rospy.loginfo("retrying ...")
               errorPrinted = True
           rospy.sleep(1.)
        else:
            socket.send(MESSAGE)
            return waitForData(socket)
        rospy.loginfo("Connected to %s : %s", ServerIP, ServerPort)



def waitForData(s):
   rospy.loginfo("Connection Successfull")
   BUFFER_SIZE = 1024
   while True:
       try:
           s.send("ALIVE")
           data = s.recv(BUFFER_SIZE)
           return data
       except socket.error, err:
           rospy.loginfo("Connection closed")
           s.close()
           rospy.sleep(2.)
           return



