#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from slaw_msgs.msg import JointDualState
from slaw_actions.msg import GripOrPlaceAction, GripOrPlaceResult
from std_srvs.srv import Empty, EmptyResponse
import actionlib

gripper_open = 0.4
min_close = 0.1
max_close = -0.3
max_effort_light = 0.15
max_effort_heavy = 0.3
time_out = 4.0
eps = 0.005
step_size = 0.025
WARN_TEMP = 60
ABOVE_EFFORT = 3

class GripperController:
  
  def __init__(self):
    rospy.on_shutdown(self.shutdown_hook)

    self.received_state = False
    self.configuration = 0.0
    self.effort = 0.0
    self.temperatures = [0.0, 0.0]
    
    rospy.Subscriber("/gripper_controller/state", JointDualState, self.joint_states_callback)
    
    self.pub = rospy.Publisher("/gripper_controller/command", Float64)
    
    self.action = actionlib.SimpleActionServer("gripper_action", GripOrPlaceAction, self.execute_cb, False)
    self.action.start()
    
    
    self.serv_open = rospy.Service("/open_gripper", Empty, self.open)
    self.serv_light = rospy.Service("/close_gripper_light", Empty, self.close_light)
    self.serv_heavy = rospy.Service("/close_gripper_heavy", Empty, self.close_heavy)

    
    self.is_moving = False
    #how often does the correct reading have to be there
    self.count = 0
    rate = rospy.Rate(20)

    while not self.received_state:
      if rospy.is_shutdown():
        return
      rate.sleep()
      print "No Data yet"

    
    self.close_light()
    rospy.sleep(0.1)
    self.open()
    
  def joint_states_callback(self, msg):
    self.configuration = msg.current_pos[0]
    self.effort = (abs(msg.load[0]) + abs(msg.load[1])) / 2.0
    self.temperatures = msg.motor_temps
    if self.temperatures[0] > WARN_TEMP or self.temperatures[1] > WARN_TEMP:
      rospy.logerr("Temperature is getting hot of one the grippers, %s", str(self.temperatures))
    self.is_moving = msg.is_moving[0] and msg.is_moving[1]
    self.received_state = True
		

  def execute_cb(self, goal):
    result = GripOrPlaceResult()

    if goal.set_point:
      self.send_command(goal.width)
    else:
      if not goal.close:
        self.open()
      else:
        if goal.heavy:
          self.close_heavy()
        else:
          self.close_light()
    self.action.set_succeeded(result)
      
    
  def send_command(self,cmd):
    rate = rospy.Rate(20)


    #print cmd
    while self.is_moving:
      rate.sleep()
   
    self.pub.publish(cmd)
    self.is_moving = True
  

  def shutdown_hook(self):
    if self.received_state:
      self.open()
    
  def open(self, req = None):
    self.send_command(gripper_open)
    if req is not None:
      return EmptyResponse()

  def close(self, effort):
    self.counter = 0
    self.send_command(min_close)
    start = rospy.Time.now()
    next_step = min_close
    while (not rospy.is_shutdown()):
      if (self.is_goal_reached(effort, self.effort)):
        break
          
      if ((rospy.Time.now() - start).to_sec() > time_out):
        return "gripper_timed_out"
      next_step -= step_size
      if (next_step < max_close):
        return "gripper_closed_too_much"
      self.send_command(next_step)
      #rospy.sleep(0.1)

    self.send_command(next_step+step_size)  
    return "success"
    
  def close_light(self, req = None):
    res = self.close(max_effort_light)
    print res
    if req is not None:
      return EmptyResponse()
      
  def close_heavy(self, req = None):
    self.close(max_effort_heavy)
    if req is not None:
      return EmptyResponse()

          
  def is_goal_reached(self, target_effort, cur_effort):
    if  (cur_effort >  target_effort-eps):
      self.counter += 1
      return self.counter >= ABOVE_EFFORT
    return False


if __name__ == "__main__":
  rospy.init_node("gripper_controller_action")
   
  try:
    gripper = GripperController()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
