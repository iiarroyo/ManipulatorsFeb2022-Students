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

class Planner():

  def __init__(self):
    moveit_commander.roscpp_initialize()
    #TODO: Initialise move it interface

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
    #TODO: Code used to move to a given position using move it
    pass


  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    pass


  def attachBox(self,box_name):
    pass
  #TODO: Close the gripper and call the service that releases the box



class myNode():
  def __init__(self):
    rospy.init_node('pick_place', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def getGoal(self,action):
    #Call the service that will provide you with a suitable target for the movement
    goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
    return goal


  def tf_goal(self, goal):
    pass
    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame


  def main(self):
    pass
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.
    # node.main()

  except rospy.ROSInterruptException:
    pass
