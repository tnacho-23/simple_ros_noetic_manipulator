#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
  rospy.init_node("arm_and_gripper_demo")
  moveit_commander.roscpp_initialize(sys.argv)

  # --- Arm setup ---
  robot = moveit_commander.RobotCommander()
  arm_group = moveit_commander.MoveGroupCommander("arm_group")  # planning group for arm
  arm_group.set_planning_time(10.0)
  arm_group.allow_replanning(True)

  rospy.loginfo("Planning frame: %s", arm_group.get_planning_frame())
  rospy.loginfo("End effector link: %s", arm_group.get_end_effector_link())
  rospy.loginfo("Available groups: %s", robot.get_group_names())

  # Use the current robot state as the start
  arm_group.set_start_state_to_current_state()

  # --- Pose goal for the arm (your coordinates) ---
  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x = -0.5851694891744713
  pose_goal.position.y = -0.1487640354713417
  pose_goal.position.z =  0.25202566688245875
  pose_goal.orientation.x = -0.7780372685982454
  pose_goal.orientation.y = -0.13787368898745353
  pose_goal.orientation.z =  0.024262565702074964
  pose_goal.orientation.w =  0.6124215724994714

  # Set the pose target (for the group's tip link)
  arm_group.set_pose_target(pose_goal)

  success = arm_group.go(wait=True)  # plan + execute
  rospy.loginfo("Arm move success: %s", success)
  arm_group.stop()
  arm_group.clear_pose_targets()

  rospy.sleep(1.0)

  # --- Gripper control ---
  gripper_pub = rospy.Publisher(
      "/hand_ee_controller/command",
      JointTrajectory,
      queue_size=10
  )

  rospy.sleep(1.0)  # wait for publisher to connect

  # Example: open gripper
  open_cmd = JointTrajectory()
  open_cmd.joint_names = ["joint_6", "joint_7"]
  pt_open = JointTrajectoryPoint()
  pt_open.positions = [0.03, 0.04]  # adjust values for "open"
  pt_open.time_from_start = rospy.Duration(2.0)
  open_cmd.points.append(pt_open)

  rospy.loginfo("Opening gripper...")
  gripper_pub.publish(open_cmd)
  rospy.sleep(2.5)

  # Example: close gripper
  close_cmd = JointTrajectory()
  close_cmd.joint_names = ["joint_6", "joint_7"]
  pt_close = JointTrajectoryPoint()
  pt_close.positions = [0.0006, 0.0]  # adjust values for "closed"
  pt_close.time_from_start = rospy.Duration(2.0)
  close_cmd.points.append(pt_close)

  rospy.loginfo("Closing gripper...")
  gripper_pub.publish(close_cmd)
  rospy.sleep(2.5)

  rospy.loginfo("Demo finished.")

if __name__ == "__main__":
  main()
