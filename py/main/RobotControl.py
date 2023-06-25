import math
from PyQt5.QtCore import QThread, pyqtSignal

import os

os.system(
    "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
    "roscore; exec bash\"'")

QThread.sleep(5)

import rospy
import geometry_msgs.msg
from khi_robot_msgs.srv import *
import moveit_commander

service = '/khi_robot_command_service'
planner = 'RRTConnectkConfigDefault'


class RobotObject(QThread):
    GetHand = pyqtSignal()
    RobotMessage = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = False
        self.Home_pose = [0.3, 0, 0.3]
        self.Hand = self.Home_pose
        self.max_cord = [0.8, 0.3, 0.6]
        self.min_cord = [0.3, -0.3, 0]

        rospy.init_node('message', anonymous=True)

        ##############################set parameter############################
        # region
        self.accuracy_jt = 0.01  # joint accuracy
        # accuracy_pos = 0.01    # position accuracy
        # accuracy_ori = 0.01    # orientation accuracy

        self.max_vel = 1.0  # max velocity scale
        self.max_acc = 1.0  # max acceleration scale
        self.min_vel = 0.5  # min velocity scale
        self.min_acc = 0.5  # min acceleration scale
        # endregion

        # Частота передачи
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)

        rospy.loginfo("Init successful")

    def RobotConnect(self, RobotModel, SimulationFlag, RobotIp="192.168.0.2"):
        if SimulationFlag:
            os.system(
                "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch simulation:=true; exec bash\"'")
        else:
            os.system(
                "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch ip:={RobotIp}; exec bash\"'")
        self.sleep(5)
        try:
            get_driver_state()
        except:
            self.RobotMessage.emit("Не удалось подключиться к роботу")
            return
        self.InitMoveGroupCommander()

    def InitMoveGroupCommander(self):
        os.system(
            f"gnome-terminal -e 'bash -c \"roslaunch khi_{RobotModel}_moveit_config moveit_planning_execution.launch; exec bash\"'")
        self.sleep(5)

        ######################################### set range of move #####################################################
        # region
        # self.khi_robot = KhiRobot()
        group = '/manipulator'

        # RobotCommander
        # rc = moveit_commander.RobotCommander()

        # MoveGroupCommander
        self.mgc = moveit_commander.MoveGroupCommander(group)

        # mgc setting
        self.mgc.set_planner_id(planner)
        self.mgc.set_goal_joint_tolerance(self.accuracy_jt)
        # mgc.set_goal_position_tolerance(accuracy_pos)
        # mgc.set_goal_orientation_tolerance(accuracy_ori)

        self.mgc.set_max_velocity_scaling_factor(self.min_vel)
        self.mgc.set_max_acceleration_scaling_factor(self.min_acc)
        self.mgc.set_max_velocity_scaling_factor(self.max_vel)
        self.mgc.set_max_acceleration_scaling_factor(self.max_acc)
        # endregion

    def RobotStart(self):
        if not self._run_flag:
            ret = get_driver_state()
            if ret.cmd_ret == 'ERROR' or ret.cmd_ret == 'HOLDED':
                cmdhandler_client('driver', 'restart')
                rospy.sleep(3)
            ret = get_driver_state()
            self.RobotMessage.emit(f"Установлен режим {ret.cmd_ret}")
            self._run_flag = True
            self.start()

    def run(self):
        while self._run_flag and not rospy.is_shutdown():
            self.GetHand.emit()

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = self.Hand[0]
            pose_goal.position.y = self.Hand[1]
            pose_goal.position.z = self.Hand[2]

            # cmdhandler_client("as", "OPEN/CLOSE")

            '''
            pose_goal.orientation.x = self.Hand[3]
            pose_goal.orientation.y = self.Hand[4]
            pose_goal.orientation.z = self.Hand[5]
            '''

            self.mgc.set_pose_target(pose_goal)
            self.mgc.go()
            self.mgc.clear_pose_targets()

            self.rate.sleep()

    def SetHand(self, Hand, width, height, PrecisionParam, CalibDist):
        self.Hand = [((self.max_cord[0] - self.min_cord[0]) / (height * PrecisionParam)) * (height * PrecisionParam // 2 - Hand[1]),
                     ((self.max_cord[1] - self.min_cord[1]) / (width * PrecisionParam)) * Hand[0],
                     (Hand[2] - CalibDist) / 100 + 0.3]

        for i, cord in enumerate(self.Hand):
            if cord > self.max_cord[i]:
                self.Hand[i] = self.max_cord[i]
            elif cord < self.min_cord[i]:
                self.Hand[i] = self.min_cord[i]

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()
        cmdhandler_client('driver', 'hold')
        rospy.sleep(3)
        ret = get_driver_state()
        self.RobotMessage.emit(f"Установлен режим {ret.cmd_ret}")



'''
class KhiRobot:
    arm_name = ''
    arm_num = 1
    max_jt = 6
    group = '/manipulator'
    min_pos_list = []
    max_pos_list = []
    max_vel_list = []
    max_acc_list = []
    base_pos_list = []

    def __init__(self):
        self.min_pos_list = []
        self.max_pos_list = []
        self.max_vel_list = []
        self.max_acc_list = []
        self.arm_name = rospy.get_param('/khi_robot_param/robot') # RS007L
        limits = rospy.get_param('/'+self.arm_name+'/joint_limits')
        self.arm_num = 1
        self.max_jt = 6
        self.group = 'manipulator'
        self.base_pos_list = [ 90*math.pi/180, 0, 90*math.pi/180, 0, 90*math.pi/180, 0 ]
        for jt in range(self.max_jt):
            self.min_pos_list.append(limits['joint'+str(jt+1)]['min_position'])
            self.max_pos_list.append(limits['joint'+str(jt+1)]['max_position'])
            self.max_vel_list.append(limits['joint'+str(jt+1)]['max_velocity'])
            self.max_acc_list.append(limits['joint'+str(jt+1)]['max_acceleration'])
'''


def cmdhandler_client(type_arg, cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)


def get_driver_state():
    ret = cmdhandler_client('driver', 'get_status')
    return ret
