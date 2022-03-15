#!/usr/bin/env python
#from asyncio.base_subprocess import ReadSubprocessPipeProto
from cgitb import lookup
import copy
import math
#from multiprocessing.connection import wait
from re import T, X
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
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    
    robot = moveit_commander.RobotCommander() #No sabemos si es correcto
    scene = moveit_commander.PlanningSceneInterface()#No sabemos si es correcto
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    self.attach_srv = rospy.ServiceProxy('AttachObject', AttachObject)

    #trans = tf.TransformListener()

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

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.tfBuffer = tfBuffer


  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):
    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
      start = rospy.get_time()
      seconds = rospy.get_time()
      while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = self.scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = self.box_name in self.scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
      return False

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
    self.move_group.set_pose_target(pose_goal)
    plan = self.move_group.go()


  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    self.attach_srv(False, box_name)


  def attachBox(self,box_name):
    #TODO: Close the gripper and call the service that releases the box
##      attach = path_planner.srv.AttachObject("BlueBox")
##      print (attach)
##      grasping_group = 'xarm_gripper'
##      touch_links = self.robot.get_link_names(group=grasping_group)
##      self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links
      
      self.attach_srv(True, box_name)
##      hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")
##      hand_group.set_named_target("close")
##      plan2 = hand_group.go()
      
      #return self.wait_for_state_update(box_name, False, True)


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
    pos_box = None
    rate = rospy.Rate(1.0)
    while not pos_box:
      try: 
          pos_box = self.planner.tfBuffer.lookup_transform("link_base",goal, rospy.Time.now(),rospy.Duration(1.0))
      except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
          rate.sleep()
    #pos_box
    #print()
    return pos_box


  def main(self):
    #TODO: Main code that contains the aplication
    self.planner = Planner()
    self.planner.addObstacles()
    box = "RedBox"
    
    #self.planner.detachBox("BlueBox")
    hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")
    hand_group.set_named_target("open")
    plan2 = hand_group.go()
    pos_box = self.tf_goal(box)
    deposit = self.tf_goal("DepositBoxRed")
    print("pos in x",pos_box.transform.translation.x)
    print("pos in y",pos_box.transform.translation.y)
    print("pos in z",pos_box.transform.translation.z)
    print("rot in w",pos_box.transform.rotation.w)
    print("rot in x",pos_box.transform.rotation.x)
    print("rot in y",pos_box.transform.rotation.y)
    print("rot in z",pos_box.transform.rotation.z)
    #print("pos_box",pos_box)
    #pos_box.TransformStamped(X)
    pose_goal = Pose()
    dep_pose = Pose()
    
    Tbox_x = pos_box.transform.translation.x
    Tbox_y = pos_box.transform.translation.y
    Tbox_z = pos_box.transform.translation.z
    Rbox_x = pos_box.transform.rotation.x

    pose_goal.position.x = Tbox_x
    pose_goal.position.y = Tbox_y
    pose_goal.position.z = Tbox_z
    pose_goal.orientation.x = 1.0
    self.planner.goToPose(pose_goal)
    self.planner.attachBox(box)
    pose_goal.position.z += 0.3
    self.planner.goToPose(pose_goal)
    
    Tdep_x = deposit.transform.translation.x
    Tdep_y = deposit.transform.translation.y
    Tdep_z = deposit.transform.translation.z
    Rdep_x = deposit.transform.rotation.x
    dep_pose.position.x = Tdep_x
    dep_pose.position.y = Tdep_y
    #dep_pose.position.z = Tdep_z
    dep_pose.orientation.x = 1.0
    self.planner.goToPose(dep_pose)
    self.planner.detachBox(box)

    rospy.signal_shutdown("Task Completed")

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass