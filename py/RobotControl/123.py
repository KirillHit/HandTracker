import rospy
from moveit_commander import MoveGroupCommander

rospy.init_node('message', anonymous=True)
group = MoveGroupCommander("manipulator")
exec_vel = 0.5
rospy.loginfo("start")
rate = rospy.Rate(1)



while not rospy.is_shutdown():
    rospy.loginfo("joint1 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_joint_value_target([0.1, 0.3, 0.1, 0.9, 0.1, 0.1])
    group.go()
    rospy.loginfo("joint1 end")
    rate.sleep()
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_joint_value_target([0.8, 0.1, 0.1, 0.5, 0.1, 0.4])
    group.go()
    rate.sleep()