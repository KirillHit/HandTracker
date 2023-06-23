from PyQt5.QtCore import QThread, pyqtSignal
import time

import rospy
from moveit_commander import MoveGroupCommander


class RobotObject(QThread):
    GetHand = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.Hand = [0,0,0,0,0,0]

        rospy.init_node('message', anonymous=True)
        self.group = MoveGroupCommander("manipulator")
        self.exec_vel = 0.5
        rospy.loginfo("start")
        self.rate = rospy.Rate(1)

    def run(self):
        while self._run_flag and not rospy.is_shutdown():
            self.GetHand.emit()
            rospy.loginfo("joint1 start")
            self.group.set_max_velocity_scaling_factor(self.exec_vel)
            self.group.set_joint_value_target(self.Hand)
            print(self.Hand)
            self.group.go()
            rospy.loginfo("joint1 end")
            self.rate.sleep()

        #self.GetHand.emit()

    def SetHand(self, Hand):
        self.Hand = Hand

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()