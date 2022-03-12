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
    #TODO: Initialize rosnode -> The class defined downwards
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
    # Start a channel for the trayectories msgs
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # Name of the reference name of the robot 
    self.planning_frame = self.move_group.get_planning_frame()
    print ("============ Planning frame:", self.planning_frame)

    # Name of the endeffector
    self.eef_link = self.move_group.get_end_effector_link()
    print( "============ End effector link:", self.eef_link)

    # We can get a list of all the groups in the robot:
    group_names = self.robot.get_group_names()
    print ("============ Available Planning Groups:", self.robot.get_group_names())
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (self.robot.get_current_state())
    print ("")

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly

    pass
  
  def addObstacles(self):

    #TODO: Add obstables in the world
    box_pose = PoseStamped()
    box_pose.header.frame_id = str(self.eef_link) 
      # In this case, we wanna have the box appear near the endeffector, so that's why we use it as a reference plane
      # The frame_id in a message specifies the point of reference for data contained in that message. 
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

    #self.move_group.go(joint_goal)

    """
    ELSE USE THIS TO PLAN AND PERFORM
    """

    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = self.robot.get_current_state()
    # display_trajectory.trajectory.append(plan)
    # # Publish
    # display_trajectory_publisher.publish(display_trajectory)

    # self.move_group.execute(plan, wait=True)

  def detachBox(self,box_name):

    #TODO: Open the gripper and call the service that releases the box
    self.scene.remove_attached_object(self.eef_link, name=box_name)


  def attachBox(self,box_name):

    #TODO: Close the gripper and call the service that releases the box
    grasping_group = 'hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)


class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls
    
    # Good practice trick, wait until the required services are online before continuing with the aplication
      # These are already initalized elsewhere. 
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
