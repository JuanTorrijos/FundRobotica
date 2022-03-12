#!/usr/bin/env python
import copy
import math
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
    #TODO: Initialise move it interface
    #============================Lo que hice===================
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    
    robot = moveit_commander.RobotCommander() #No sabemos si es correcto
    scene = moveit_commander.PlanningSceneInterface()#No sabemos si es correcto
    
    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name) #No sabemos si aplica a este robot
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link
    
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

##    print "============ Printing robot state"
##    print robot.get_current_state()
##    print ""

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
##    pose_goal = Pose()
##    pose_goal.orientation.w = 1.0
##    pose_goal.position.x = 0.4
##    pose_goal.position.y = 0.1
##    pose_goal.position.z = 0.4
##    print pose_goal
##
##    move_group.set_pose_target(pose_goal)
    #print pose_goal
##    waypoints = []
##    scale = 1.0
##
##    wpose = move_group.get_current_pose().pose
##    wpose.position.z -= scale * 0.1  # First move up (z)
##    wpose.position.y += scale * 0.2  # and sideways (y)
##    waypoints.append(copy.deepcopy(wpose))
##
##    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
##    waypoints.append(copy.deepcopy(wpose))
##
##    wpose.position.y -= scale * 0.1  # Third move sideways (y)
##    waypoints.append(copy.deepcopy(wpose))
##    
##    (plan, fraction) = move_group.compute_cartesian_path(
##                                       waypoints,   # waypoints to follow
##                                       0.01,        # eef_step
##                                       0.0)         # jump_threshold
##    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
##    display_trajectory.trajectory_start = robot.get_current_state()
##    display_trajectory.trajectory.append(plan)
##    display_trajectory_publisher.publish(display_trajectory);
    
##    move_group.execute(plan, wait=True)
##    move_group.go(pose_goal, wait=True)
##    move_group.stop()
##    move_group.clear_pose_targets()
##    joint_goal = move_group.get_current_joint_values()
##    print joint_goal
##    joint_goal[0] = 0
##    joint_goal[1] = -math.pi/4
##    joint_goal[2] = 0
##    joint_goal[3] = -math.pi/2
##    joint_goal[4] = 0
##    joint_goal[5] = math.pi/3
##    move_group.go(joint_goal, wait=True)
##    move_group.stop()
    #============================Lo que hice===================

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    #============================Lo que hice===================
    
    #============================Lo que hice===================
      pass

  def addObstacles(self):

    #TODO: Add obstables in the world

        
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
    #============================Lo que hice===================
    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go()
##    plan = self.move_group.go(wait=True)
##    self.move_group.stop()
##    self.move_group.clear_pose_targets()
##    waypoints = []
##    scale = 1.0
##
##    wpose = self.move_group.get_current_pose().pose
##    wpose.position.z -= scale * 0.1  # First move up (z)
##    wpose.position.y += scale * 0.2  # and sideways (y)
##    waypoints.append(copy.deepcopy(wpose))
##
##    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
##    waypoints.append(copy.deepcopy(wpose))
##
##    wpose.position.y -= scale * 0.1  # Third move sideways (y)
##    waypoints.append(copy.deepcopy(wpose))
##    
##    (plan, fraction) = self.move_group.compute_cartesian_path(
##                                       waypoints,   # waypoints to follow
##                                       0.01,        # eef_step
##                                       0.0)         # jump_threshold
##    self.move_group.execute(plan, wait=True)
    #============================Lo que hice===================


  def detachBox(self,box_name):

    #TODO: Open the gripper and call the service that releases the box
    pass


  def attachBox(self,box_name):

    #TODO: Close the gripper and call the service that releases the box
    pass



class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls

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
    
    #============================Lo que hice===================
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.position.x = 0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 0.5
    self.planner.goToPose(pose_goal)
    #============================Lo que hice===================

    rospy.signal_shutdown("Task Completed")



if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
