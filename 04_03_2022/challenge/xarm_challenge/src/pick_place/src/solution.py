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

    def wait_for_state_update(self, box_name, box_is_known=False,
                              box_is_attached=False, timeout=1):
        """
        Whenever we change something in moveit we need to make sure that the
        interface has been updated properly
        """
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def addObstacles(self):
        """
        Add obstables in the world
        """
        #TO DO: Add obstables in the world
        #Cargo names
        targets = ["RedBox",
                "BlueBox",
                "GreenBox"]
        targets_state = True
        # Red box
        rbox_pose = geometry_msgs.msg.PoseStamped()
        rbox_pose.header.frame_id = targets[0]
        rbox_pose.pose.orientation.w = 1.0
        rbox_pose.pose.position.z = 0
        rbox_name = targets[0]
        self.scene.add_box(rbox_name, rbox_pose, size=(0.06, 0.06, 0.06))
        targets_state = targets_state and self.wait_for_state_update(targets[0], box_is_known=True)
        # Blue box
        bbox_pose = geometry_msgs.msg.PoseStamped()
        bbox_pose.header.frame_id = targets[1]
        bbox_pose.pose.orientation.w = 2.0
        bbox_pose.pose.position.z = 0
        bbox_name = targets[1]
        self.scene.add_box(bbox_name, bbox_pose, size=(0.06, 0.06, 0.06))
        targets_state = targets_state and self.wait_for_state_update(targets[1], box_is_known=True)
        # Green box
        gbox_pose = geometry_msgs.msg.PoseStamped()
        gbox_pose.header.frame_id = targets[2]
        gbox_pose.pose.orientation.w = 3.0
        gbox_pose.pose.position.z = 0
        gbox_name = targets[2]
        self.scene.add_box(gbox_name, gbox_pose, size=(0.06, 0.06, 0.06))
        targets_state = targets_state and self.wait_for_state_update(targets[2], box_is_known=True)
        #goal names
        boxes = ["DepositBoxGreen",
                "DepositBoxRed",
                "DepositBoxBlue"]
        return targets_state

    def goToPose(self, pose_goal):
        """
        Moves xarm6 group to given pose
        :pose_goal:
        """
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = pose_goal.transform.translation.x
        pose_target.position.y = pose_goal.transform.translation.y
        pose_target.position.z = pose_goal.transform.translation.z+0.15
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0

        self.arm_group.set_pose_target(pose_target)
        act = self.arm_group.go(wait=True)
        time.sleep(2)

    def detachBox(self, box_name):
        """
        Open the gripper and call the service that releases the box
        """
        time.sleep(1)
        self.hand_group.set_named_target("open")
        act_2 = self.hand_group.go(wait=True)
        self.attach_srv(False, box_name)

    def attachBox(self, box_name, pose_goal):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = pose_goal.transform.translation.x
        pose_target.position.y = pose_goal.transform.translation.y
        pose_target.position.z = pose_goal.transform.translation.z - 0.01
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0
        self.arm_group.set_pose_target(pose_target)
        act_3 = self.arm_group.go(wait=True)

        self.hand_group.set_joint_value_target(CLOSED_JOINT_VALUE)
        act_2 = self.hand_group.go(wait=True)
        self.attach_srv(True, box_name)

        pose_target.position.x = pose_goal.transform.translation.x
        pose_target.position.y = pose_goal.transform.translation.y
        pose_target.position.z = 0.3
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0
        self.arm_group.set_pose_target(pose_target)
        act_4 = self.arm_group.go(wait=True)


class myNode():
    def __init__(self):
        rospy.init_node('pick_place', anonymous=True)
        self.rate = rospy.Rate(5)
        # TODO: Initialise ROS and create the service calls

        # Good practice trick, wait until the required services are online
        # before continuing with the aplication
        rospy.wait_for_service('RequestGoal')
        rospy.wait_for_service('AttachObject')

    def getGoal(self, action):
        # Call the service that will provide you with a suitable target for
        # the movement
        req_goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
        return req_goal(action)

    def tf_goal(self, goal):
        trans = None
        while not trans:
            try:
                buffer = self.planner.tfBuffer
                trans = buffer.lookup_transform("link_base",
                                                goal,
                                                rospy.Time.now(),
                                                rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self.rate.sleep()

        return trans

    def main(self):
        self.planner = Planner()
        self.planner.addObstacles() # a
        while True:
            pick_goal = self.getGoal("pick") # b
            if pick_goal.goal == "End":
                break
            pick_goal_pose = self.tf_goal(pick_goal.goal) # c
            self.planner.goToPose(pick_goal_pose)
            self.planner.attachBox(pick_goal.goal, pick_goal_pose)

            place_goal = self.getGoal("place")
            place_goal_pose = self.tf_goal(place_goal.goal)
            self.planner.goToPose(place_goal_pose)
            self.planner.detachBox(pick_goal.goal)
        rospy.signal_shutdown("Task Completed")
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        node = myNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
