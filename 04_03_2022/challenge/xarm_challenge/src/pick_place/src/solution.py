#!/usr/bin/env python
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp
import geometry_msgs.msg
import time

class Planner():
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.arm_group = moveit_commander.MoveGroupCommander("xarm6")
    self.hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.attach_srv = rospy.ServiceProxy('AttachObject', AttachObject)

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    pass

  def addObstacles(self):
    #TODO: Add obstables in the world
    pass

	
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

  def goToPose(self,pose_goal):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose_goal.transform.translation.x
    pose_target.position.y = pose_goal.transform.translation.y
    pose_target.position.z = pose_goal.transform.translation.z + 0.1
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0 
    pose_target.orientation.w = 0
   
    self.arm_group.set_pose_target(pose_target)
    act = self.arm_group.go(wait = True) 

  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    self.hand_group.set_named_target("open")
    self.attach_srv(False, box_name)
    time.sleep(2)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.z = 0.3


  def attachBox(self,box_name):
    self.hand_group.set_named_target("close")
    self.attach_srv(True, box_name)
    time.sleep(2)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.z = 0.3
  

class myNode():
  def __init__(self):
    rospy.init_node('pick_place', anonymous=True)
    self.rate = rospy.Rate(5)
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def getGoal(self,action):
    #Call the service that will provide you with a suitable target for the movement
    req_goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
    return req_goal(action)

  def tf_goal(self, goal):
    trans = None
    while not trans:
      try:
        trans = self.planner.tfBuffer.lookup_transform("link_base", goal , rospy.Time.now(), rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        self.rate.sleep()

    return trans


  def main(self):
    self.planner = Planner()
    goal = self.getGoal('place')
    goal_pose = self.tf_goal("GreenBox")
    self.planner.goToPose(goal_pose)
    time.sleep(2)
    self.planner.attachBox("GreenBox")

    rospy.signal_shutdown("Task Completed")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
