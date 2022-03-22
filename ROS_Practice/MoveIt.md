# MoveIt implementation:


Documentation: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html
ROSPY API DOCs: http://docs.ros.org/en/melodic/api/rospy/html/

Move Group Commander: **This is litterally the most important one, the only one to command robot movements**
Planning interface  http://docs.ros.org/en/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html

Robot commander: http://docs.ros.org/en/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
## Requirements:

* sudo apt-get install ros-melodic-franka-description

* sudo apt install ros-melodic-moveit-visual-tools

* sudo apt install ros-melodic-moveit

* sudo apt install ros-melodic-tf2-tools
* sudo apt-get install ros-melodic-moveit-kinematics
* sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy

## Basic commands:
roslaunch moveit_setup_assistant setup_assistant.launch


roslaunch xarm_gazebo xarm6_challenge.launch

roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch

rosrun path_planner GoalPlanner.py

rosrun pick_place solution_template.py


## To run the project:

rosrun pick_place solution_template.py