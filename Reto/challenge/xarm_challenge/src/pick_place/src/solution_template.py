#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

from fileinput import filename
import re
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose

from tf.transformations import *
from moveit_msgs.msg import Grasp
import math
# Tried to read xml files
import xml.dom.minidom
# Import services definitions
from path_planner.srv import RequestGoal,RequestGoalResponse,AttachObject,AttachObjectResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse



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

    # Tolerance per computation
    self.move_group.set_goal_position_tolerance(0.001)
    self.move_group.set_goal_orientation_tolerance(0.001)

    ## Comander for gripper
    # Instantiate a robot commander

    # Instantiate a robot commander
    self.group_name_Eef = "xarm_gripper"
    self.move_group_Eef = moveit_commander.MoveGroupCommander(self.group_name_Eef)
    # Start a channel for the trayectories msgs

    # Name of the reference name of the robot
    self.planning_frame = self.move_group.get_planning_frame()
    # Tho, we actually want the robot base
    self.planning_frame = "link_base"
    print ("============ Planning frame:", self.planning_frame)


    # Name of the endeffector
    self.eef_link = self.move_group.get_end_effector_link()
    # Change the eef link
    self.eef_link = "link_eef"
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
    # I'm not totally sure, but I believe, here comes the Attach message.
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

    #TODO: Add obstables in the world I belive this is only in rViz
    #Cargo names

    # The method to obtain the boxes spawm is the following:
      # From the Gazebo simulation, calculate a transform
      # respective to the link_base.
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1.0))
    listener = tf2_ros.TransformListener(tfBuffer)

    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]

    for t in targets:

      pose = tfBuffer.lookup_transform("link_base", t,   rospy.Time(), rospy.Duration(1.0))
      # Once we have registered the pose, store it in a structure PoseStamped
      box_pose = PoseStamped()
      box_pose.header.frame_id = self.robot.get_planning_frame()
      box_pose.pose.position.x = pose.transform.translation.x
      box_pose.pose.position.y = pose.transform.translation.y
      # Move the z position up, half the size of the box, or it will be crossing the table
      box_pose.pose.position.z = 0.03

      box_pose.pose.orientation = pose.transform.rotation
      # box_pose.pose.orientation = pose.transform.rotation

      # This loop spawn the boxes for the planner
      while not self.wait_for_state_update(t, box_is_known=True, timeout=1.0):

        print(t)
        # Tells the planner that a box with a certain name in a certain pose
        # the size is defined in the gazebo_world file xarm_pickplace_test.world
        self.scene.add_box(t, box_pose, size=(0.06, 0.06, 0.06))

    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"
               ]

    # Same for the containers, adds them for the planner
    for box in boxes:
      pose = tfBuffer.lookup_transform("link_base", box,  rospy.Time(), rospy.Duration(1.0))
      box_pose = PoseStamped()

      box_pose.header.frame_id = self.robot.get_planning_frame()
      box_pose.pose.position.x = pose.transform.translation.x
      box_pose.pose.position.y = pose.transform.translation.y
      box_pose.pose.position.z = 0.15/2

      box_pose.pose.orientation = pose.transform.rotation

      while not self.wait_for_state_update(box, box_is_known=True, timeout=0.1):

          print(box)
          self.scene.add_box(box, box_pose, size=(0.36, 0.15, 0.1))


  def goToPose(self,pose_goal):
    #TODO: Code used to move to a given position using move it

    # First, we plan the motion path to get to the desired pose
    self.move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = self.move_group.go(wait=True)
    #self.move_group.execute(plan, wait=True)

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()

     # Clear your targets after planning with poses.
    self.move_group.clear_pose_targets()


  def detachBox(self,box_name):

    #TODO: Open the gripper and call the service that releases the box

    # Release box:
    self.scene.remove_attached_object(self.eef_link, name=box_name)


  def attachBox(self,box_name):

    #TODO: Close the gripper and call the service that releases the box
    grasping_group = "xarm_gripper"
    touch_links = self.robot.get_link_names(group=grasping_group)
    print("Touch links del attachBox")
    print(touch_links)
    print("-----")
    self.scene.attach_box(self.eef_link, box_name, touch_links=["left_finger","right_finger"])
    self.moveEndeffector(True)

  def moveEndeffector (self, set2open):
    j = self.move_group_Eef.get_active_joints()
    print(j)
    """
    if set2open:
      pose_goal
    else:
      pass
    set_joint_value_target (self, arg1, arg2=None, arg3=None)
    self.move_group_Eef.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    self.move_group_Eef.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group_Eef.stop()
    # Clear your targets after planning with poses.
    self.move_group_Eef.clear_pose_targets()
    """

class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls

    # Good practice trick, wait until the required services are online before continuing with the aplication
      # These are already initalized elsewhere.
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

  def getGoal(self, action):

    #TODO: Call the service that will provide you with a suitable target for the movement
    # After reading the GoalPlanner programm, we realized the following:
      # There is a service called RequestGoal, with which we can ask for an action.
      # With the method _sendGoal_, we can ask for a position starting/goal to pick or place a box.
      # The pre-defined actions are: place, and pick.
      # This will increment automatically to change boxes

    pickPlace = rospy.ServiceProxy('RequestGoal', RequestGoal)
    resp = pickPlace(action)

    return resp

  def tf_goal(self, goal, closeGrip):

    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    # Similar to the getGoal method, we use this to
    # send a message to the service AttachObject
      # This time, the inputs is a boolean to attach
      # the objecto to the eef and the frame name to attach
    pickPlace = rospy.ServiceProxy('AttachObject', AttachObject)
    resp = pickPlace(closeGrip, goal)

    return resp
  def moveObject_a2b(self, start, goal):
    pose_start = PoseStamped()
    pose_start.header.frame_id = "link_base"

    pose_trans = self.tfBuffer.lookup_transform("link_base", start, rospy.Time(0), rospy.Duration(0.5)) # "xarm_gripper_base_link"

    pose_start.pose.orientation.w = 0 #- pose_trans.transform.rotation.w
    pose_start.pose.orientation.x = 1 #- pose_trans.transform.rotation.x # To look one
    pose_start.pose.orientation.y = 0 #- pose_trans.transform.rotation.y
    pose_start.pose.orientation.z = 0 #- pose_trans.transform.rotation.z

    pose_trans.transform.translation.z += 0.05
    pose_start.pose.position = pose_trans.transform.translation
    #---------------------
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"

    pose_trans = self.tfBuffer.lookup_transform("link_base", goal, rospy.Time(0), rospy.Duration(0.5)) # "xarm_gripper_base_link"

    pose_goal.pose.orientation.w = 0 #- pose_trans.transform.rotation.w
    pose_goal.pose.orientation.x = 1 #- pose_trans.transform.rotation.x # To look one
    pose_goal.pose.orientation.y = 0 #- pose_trans.transform.rotation.y
    pose_goal.pose.orientation.z = 0 #- pose_trans.transform.rotation.z

    pose_trans.transform.translation.z += 0.2
    pose_goal.pose.position = pose_trans.transform.translation

    print("-#--#--#-")
    print("Moving object with object attached")
    print("Point A:")
    print(pose_start)
    print("------")
    print("Point B:")
    print(pose_goal)
    print("-#--#--#-")

    # Actually moving the robot
    self.planner.goToPose(pose_start)

    self.planner.attachBox(start)
    self.planner.goToPose(pose_goal)
    self.planner.detachBox(start)
    

  def main(self):
    #TODO: Main code that contains the aplication

    # Init robot instrunction planner
    self.planner = Planner()
    self.planner.addObstacles()
    # Create a buffer to read the transformation results
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    start = self.getGoal("pick")
    endpoint = self.getGoal("place")
    print("----")
    print(start)
    print(endpoint)
    print("----")

    # self.robot.get_planning_frame()
    self.moveObject_a2b(start.goal, endpoint.goal)
    

    # Add an obstacle to the environment
    
    rospy.signal_shutdown("Task Completed")

    ## end effector, ponerlo a mirar hacia abajo (last param). Debe quedar un poco arriba, y después bajarla a que toque

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
