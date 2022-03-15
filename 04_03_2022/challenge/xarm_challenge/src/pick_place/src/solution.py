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
from trajectory_msgs.msg import *
from moveit_msgs.msg import Grasp
import geometry_msgs.msg
import time

CLOSED_JOINT_VALUE = {"drive_joint": 0.2}
OPEN_JOINT_VALUE = {"drive_joint": 0.0}

class Planner():
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.arm_group = moveit_commander.MoveGroupCommander("xarm6")
    self.hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.attach_srv = rospy.ServiceProxy('AttachObject', AttachObject)
    self.scene = moveit_commander.PlanningSceneInterface()

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=1):
    """
    Whenever we change something in moveit we need to make sure that the interface has been updated properly
    """
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      box_is_known = box_name in scene.get_known_object_names()
      print(scene.get_known_object_names())
      print(scene.get_attached_objects())

      # Test if we are in the expected state
      if (box_is_attached == box_is_attached) and (box_is_known == box_is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def addObstacles(self):
    #TODO: Add obstables in the world
    #Cargo names
    size = 3
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "link_base"
    pose.pose.orientation.w = 1.0
    box_name = targets[0]
    self.scene.add_box(box_name, pose, size=(0.1, 0.1, 0.1))

  def goToPose(self, pose_goal):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose_goal.transform.translation.x
    pose_target.position.y = pose_goal.transform.translation.y
    pose_target.position.z = pose_goal.transform.translation.z+0.15
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0 
    pose_target.orientation.w = 0
   
    self.arm_group.set_pose_target(pose_target)
    act = self.arm_group.go(wait = True) 
    time.sleep(2)

  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    time.sleep(1)
    self.hand_group.set_named_target("open")
    act_2 = self.hand_group.go(wait = True)
    self.attach_srv(False, box_name)

  def attachBox(self, box_name, pose_goal):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose_goal.transform.translation.x
    pose_target.position.y = pose_goal.transform.translation.y
    pose_target.position.z = pose_goal.transform.translation.z+0.015
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0 
    pose_target.orientation.w = 0    
    self.arm_group.set_pose_target(pose_target)
    act_3 = self.arm_group.go(wait = True) 
    self.hand_group.set_joint_value_target(CLOSED_JOINT_VALUE)
    
    act_2 = self.hand_group.go(wait = True) 
    self.attach_srv(True, box_name)

    pose_target.position.x = pose_goal.transform.translation.x
    pose_target.position.y = pose_goal.transform.translation.y
    pose_target.position.z = 0.3
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0 
    pose_target.orientation.w = 0    
    self.arm_group.set_pose_target(pose_target)
    act_4 = self.arm_group.go(wait = True)


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
        trans = self.planner.tfBuffer.lookup_transform("link_base", goal, rospy.Time.now(), rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        self.rate.sleep()

    return trans


  def main(self):
    self.planner = Planner()
    self.planner.addObstacles()
    end = None
    while not end:
      pick_goal = self.getGoal("pick")
      if pick_goal.goal == "End":
        break
      pick_goal_pose = self.tf_goal(pick_goal.goal)
      self.planner.goToPose(pick_goal_pose)
      self.planner.attachBox(pick_goal.goal, pick_goal_pose)

      place_goal = self.getGoal("place")
      place_goal_pose = self.tf_goal(place_goal.goal)
      self.planner.goToPose(place_goal_pose)
      self.planner.detachBox(pick_goal.goal)


      # print(self.planner.arm_group.get_current_state())

    rospy.signal_shutdown("Task Completed")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
