#!/usr/bin/env python3


import rospy, sys, moveit_commander
rospy.init_node("sample_pose")
moveit_commander.roscpp_initialize(sys.argv)

group = moveit_commander.MoveGroupCommander("arm_group")

# random joint target
group.set_random_target()
group.go(wait=True)
group.stop()

# get the EEF pose after moving
eef_pose = group.get_current_pose().pose
print("Random valid EEF pose:", eef_pose)
