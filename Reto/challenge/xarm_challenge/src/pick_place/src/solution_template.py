#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

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
import math


class Planner():

  def __init__(self):

    #TODO: Initialise move it interface
    moveit_commander.roscpp_initialize(sys.argv)
    print("Move it commander Initialized")
    #TODO: Initialize rosnode -> which one?
    rospy.init_node('myNode', anonymous=True)
    #Python node

    # Instantiate a robot commander
      # Provides information such as the robot’s kinematic model and the robot’s current joint states
    self.robot = moveit_commander.RobotCommander()
    # Instantiate a robot commander
      # This provides a remote interface for getting, setting, and updating the robot’s internal understanding of the surrounding world:
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group_name = "xarm6"
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    
    pass
  
  def addObstacles(self):

    #TODO: Add obstables in the world
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_name = "box"
    self.scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
	
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
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -math.pi/4
    joint_goal[2] = 0
    joint_goal[3] = -math.pi/2
    joint_goal[4] = 0
    joint_goal[5] = math.pi/3
    joint_goal[6] = 0

    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    self.move_group.set_pose_target(pose_goal)

    """
    ELSE USE THIS TO PLAN AND PERFORM
    """

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    self.move_group.execute(plan, wait=True)

  def detachBox(self,box_name):

    #TODO: Open the gripper and call the service that releases the box
    self.scene.remove_attached_object(eef_link, name=box_name)


  def attachBox(self,box_name):

    #TODO: Close the gripper and call the service that releases the box
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(eef_link, box_name, touch_links=touch_links)


class myNode():
  def __init__(self):
    # #TODO: Initialise ROS and create the service calls
    # pub = rospy.Publisher('RequestGoal', String, queue_size=10)
    # pub = rospy.Publisher('AttachObject', String, queue_size=10)
    
    # rospy.init_node('RequestGoal', anonymous=True)
    # rospy.init_node('AttachObject', anonymous=True)
    # Do we actually need it?
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #   hello_str = "hello world %s" % rospy.get_time()
    #   rospy.loginfo(hello_str)
    #   pub.publish(hello_str)
    #   rate.sleep()    
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def getGoal(self,action):

    #TODO: Call the service that will provide you with a suitable target for the movement
    pass

  def tf_goal(self, goal):

    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    pass

  def main(self):
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
