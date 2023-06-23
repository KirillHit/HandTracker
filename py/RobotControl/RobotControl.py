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

    def run(self):
        rospy.init_node('message', anonymous=True)
        group = MoveGroupCommander("manipulator")
        exec_vel = 0.5

        while self._run_flag and not rospy.is_shutdown():
            rospy.loginfo("joint1 start")
            group.set_max_velocity_scaling_factor(exec_vel)
            group.set_joint_value_target(self.Hand)
            group.go()
            rospy.loginfo("joint1 end")



        #self.GetHand.emit()

    def SetHand(self, Hand):
        self.Hand = Hand

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()