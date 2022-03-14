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
    pose.header.frame_id = "world"
    pose.pose.orientation.w = 1.0
    box_name = targets[0]
    self.scene.add_box(box_name, pose)

  def goToPose(self,pose_goal):
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
    return self.attach_srv(False, box_name)

  def attachBox(self, box_name, pose_goal):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = pose_goal.transform.translation.x
    pose_target.position.y = pose_goal.transform.translation.y
    pose_target.position.z = pose_goal.transform.translation.z+0.02
    pose_target.orientation.x = 1
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0 
    pose_target.orientation.w = 0    
    self.arm_group.set_pose_target(pose_target)
    act_3 = self.arm_group.go(wait = True) 
    joint_target = moveit_msgs.msg.Grasp(10)
    # self.hand_group.set_named_target("close")
    self.hand_group.set_joint_value_target("close")
    
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

  def open_gripper(self,pose_goal):

    # self.hand_group.set_named_target("open")
    # act_4 = self.hand_group.go(wait = True)

    grasp = Grasp()
    # grasp.self.arm_group.get_current_pose()
    grasp.grasp_pose.header.frame_id = "link_base"
    grasp.grasp_pose.pose.position.x = pose_goal.transform.translation.x
    grasp.grasp_pose.pose.position.y = pose_goal.transform.translation.y
    grasp.grasp_pose.pose.position.z = pose_goal.transform.translation.z + 0.02
    grasp.grasp_pose.pose.orientation.x = 1
    grasp.grasp_pose.pose.orientation.y = 0
    grasp.grasp_pose.pose.orientation.z = 0#pose_goal.transform.rotation.z
    grasp.grasp_pose.pose.orientation.w = 0#pose_goal.transform.rotation.w
    # print(pose_goal)

    grasp.pre_grasp_approach.direction.header.frame_id = "link_base"
    grasp.pre_grasp_approach.direction.vector.z = 1.0
    grasp.pre_grasp_approach.min_distance = 0.1
    grasp.pre_grasp_approach.desired_distance = 0.0


    # grasp.post_grasp_retreat.direction.header.frame_id = "link_base"
    # grasp.post_grasp_retreat.direction.vector.z = 1.0
    # grasp.post_grasp_retreat.min_distance = 0.1
    # grasp.post_grasp_retreat.desired_distance = 0.25

    point_pre = JointTrajectoryPoint()
    grasp.pre_grasp_posture.points.append(point_pre)
    # print(grasp.pre_grasp_posture.points[0].positions)
    grasp.pre_grasp_posture.joint_names.append("drive_joint")
    grasp.pre_grasp_posture.points[0].positions.append(0.0)
    grasp.pre_grasp_posture.points[0].time_from_start = rospy.Duration(2)

    point = JointTrajectoryPoint()
    grasp.grasp_posture.points.append(point)
    # print(grasp.pre_grasp_posture.points[0].positions)
    grasp.grasp_posture.joint_names.append("drive_joint")
    grasp.grasp_posture.points[0].positions.append(0.35)
    grasp.grasp_posture.points[0].time_from_start = rospy.Duration(2)

    self.arm_group.pick("obj",grasp)

    self.attach_srv(True, "BlueBox")

    time.sleep(3)

  def close_gripper():
    pass

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

    point = JointTrajectoryPoint()
    # point.joint_names.append("drive_joint")
    # point.positions.append(0.35)
    # point.time_from_start = rospy.Duration(2)
    hand_joints = {"drive_joint": 0.35}

    self.planner.hand_group.set_joint_value_target(hand_joints)
    act_2 = self.planner.hand_group.go(wait = True) 


    # goal_pose = self.tf_goal("RedBox")
    # self.planner.goToPose(goal_pose)
    # self.planner.open_gripper(goal_pose)
    # self.planner.hand_group.set_named_target("open")


    # goal = self.getGoal("pick")
    # goal_pose = self.tf_goal("RedBox")
    # self.planner.goToPose(goal_pose)
    # self.planner.attachBox("RedBox",goal_pose)
   

    # goal = self.getGoal("place")
    # goal_pose = self.tf_goal("DepositBoxRed")
    # self.planner.goToPose(goal_pose)
    # self.planner.detachBox("RedBox")
    # print(self.planner.arm_group.get_current_state())
    # print(goal_pose)

    
    # goal = self.getGoal("pick")
    # goal_pose = self.tf_goal("BlueBox")
    # self.planner.goToPose(goal_pose)
    # self.planner.attachBox("BlueBox",goal_pose)
    # print(self.planner.arm_group.get_current_state())


    # goal = self.getGoal("place")
    # goal_pose = self.tf_goal("DepositBoxBlue")
    # self.planner.goToPose(goal_pose)
    # self.planner.detachBox("BlueBox")
    # print(self.planner.arm_group.get_current_state())


    # goal = self.getGoal("pick")
    # goal_pose = self.tf_goal("GreenBox")
    # self.planner.goToPose(goal_pose)
    # self.planner.attachBox("GreenBox",goal_pose)
    # print(self.planner.arm_group.get_current_state())

    # goal = self.getGoal("place")
    # goal_pose = self.tf_goal("DepositBoxGreen")
    # self.planner.goToPose(goal_pose)
    # self.planner.detachBox("GreenBox")
    # print(self.planner.arm_group.get_current_state())


    # while goal!="end"
    # 1 ciclo
    # goal = self.getGoal('place')
    # 1.2 ciclo
    # goal = self.getGoal('pick')

    rospy.signal_shutdown("Task Completed")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
