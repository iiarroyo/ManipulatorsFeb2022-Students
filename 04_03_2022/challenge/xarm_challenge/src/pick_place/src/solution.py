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
        """
        Initialise move it interface
        """
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
        #Cargo names
        targets = ["RedBox",
                   "BlueBox",
                   "GreenBox",
                   "RedBox"]
        state = True
        for target in targets:
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = target
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.z = 0
            box_name = target + "rviz"
            self.scene.add_box(box_name, box_pose, size=(0.06, 0.06, 0.06))
            state &= self.wait_for_state_update(target, box_is_known=True)
        # boxes = ["DepositBoxGreen",
        #          "DepositBoxRed",
        #          "DepositBoxBlue"]
        return state

    def goToPose(self, pose_goal):
        """
        Moves xarm6 group to given pose
        :pose_goal: pose of box in PoseStamped() format
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
        self.arm_group.go(wait=True)
        time.sleep(2)

    def detachBox(self, box_name):
        """
        Open the gripper and call the service that releases the box
        :box_name: frame name that will be detached
        """
        time.sleep(1)
        self.hand_group.set_named_target("open")
        act_2 = self.hand_group.go(wait=True)
        self.attach_srv(False, box_name)

    def attachBox(self, box_name, pose_goal):
        """
        Close the gripper and call the service that releases the box
        :box_name: frame name that will be attached
        :pose_goal: pose of box
        """
        # Lower gripper
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = pose_goal.transform.translation.x
        pose_target.position.y = pose_goal.transform.translation.y
        pose_target.position.z = pose_goal.transform.translation.z - 0.01
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0
        self.arm_group.set_pose_target(pose_target)
        self.arm_group.go(wait=True)

        # Close gripper
        self.hand_group.set_joint_value_target(CLOSED_JOINT_VALUE)
        self.hand_group.go(wait=True)
        # Call AttachObject service
        self.attach_srv(True, box_name)

        # Raise gripper
        pose_target.position.x = pose_goal.transform.translation.x
        pose_target.position.y = pose_goal.transform.translation.y
        pose_target.position.z = 0.3
        pose_target.orientation.x = 1
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0
        self.arm_group.set_pose_target(pose_target)
        self.arm_group.go(wait=True)


class myNode():
    def __init__(self):
        """
        Initialise ROS and create the service calls
        """
        rospy.init_node('pick_place', anonymous=True)
        self.rate = rospy.Rate(5)
        rospy.wait_for_service('RequestGoal')
        rospy.wait_for_service('AttachObject')

    def getGoal(self, action):
        """
        Call the service that will provide you with a suitable target for
        the movement
        :action: Attach service will be called with given action (pick, place)
        """
        req_goal = rospy.ServiceProxy('RequestGoal', RequestGoal)
        return req_goal(action)

    def tf_goal(self, goal):
        """
        Use tf2 to retrieve the position of the target with respect to
        the proper reference frame
        :goal: string goal name
        :return: PoseStamped()
        """
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
        self.planner.addObstacles()  # 1
        while True:
            pick_goal = self.getGoal("pick")  # 2
            if pick_goal.goal == "End":
                break
            pick_goal_pose = self.tf_goal(pick_goal.goal)  # 3
            self.planner.goToPose(pick_goal_pose)  # 4
            self.planner.attachBox(pick_goal.goal, pick_goal_pose)  # 5

            place_goal = self.getGoal("place")  # 6
            place_goal_pose = self.tf_goal(place_goal.goal)  # 7
            self.planner.goToPose(place_goal_pose)  # 8
            self.planner.detachBox(pick_goal.goal)  # 9
        rospy.signal_shutdown("Task Completed")
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        node = myNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
