import math
from PyQt5.QtCore import QThread, pyqtSignal

import os

os.system("gnome-terminal --tab -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
          "roscore\"")

QThread.sleep(3)

import rospy
import geometry_msgs.msg
from moveit_msgs.msg import JointConstraint
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
        self.Home_pose = [0.4, 0, 0.3]
        self.HomePoseFlag = False
        self.Hand = self.Home_pose
        self.Compress = False
        self.CompressFlag = False

        self.PrevHand = None
        self.PrevCompress = None

        self.max_cord = [0.8, 0.3, 0.6]
        self.min_cord = [0.3, -0.3, 0]

        self.ConnectFlag = False

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
                "gnome-terminal --tab -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch simulation:=true\"")
        else:
            os.system(
                "gnome-terminal --geometry=0x0 --tab -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch ip:={RobotIp}\"")
        self.sleep(3)
        # rospy.Subscriber("rs007l_arm_controller/state", str, lambda a: print(a))
        self.InitMoveGroupCommander(RobotModel)
        self.ConnectFlag = True

    def InitMoveGroupCommander(self, RobotModel):
        os.system(f"gnome-terminal --tab -- bash -c \"roslaunch khi_{RobotModel}_moveit_config moveit_planning_execution.launch\"")
        self.sleep(10)

        ######################################### set range of move #####################################################
        # region
        #self.khi_robot = KhiRobot()
        group = 'manipulator'

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

        self.mgc.set_workspace([*self.min_cord, *self.max_cord])
        self.mgc.set_num_planning_attempts(2)

        limits = rospy.get_param('/' + RobotModel.upper() + '/joint_limits')
        limits["joint1"]["min_position"] = -1.9198621
        limits["joint1"]["max_position"] = -1.2217304
        limits["joint4"]["min_position"] = -0.3490658
        limits["joint4"]["max_position"] = 0.3490658
        rospy.set_param('/' + RobotModel.upper() + '/joint_limits', limits)

        '''
        empty_joint_constraints = JointConstraint()
        empty_joint_constraints.joint_name = "joint1"
        empty_joint_constraints.position = -70
        empty_joint_constraints.empty_joint_constraints.tolerance_above = 1.5
        empty_joint_constraints.weight = 1
        empty_joint_constraints1 = JointConstraint()
        empty_joint_constraints1.joint_name = "joint1"
        empty_joint_constraints1.position = -110
        empty_joint_constraints1.empty_joint_constraints.tolerance_below = 1.5
        empty_joint_constraints1.weight = 1
        self.mgc.set_path_constraints(self, [empty_joint_constraints, empty_joint_constraints1])
        '''
        # endregion

    def RobotStart(self):
        if not self._run_flag and self.ConnectFlag:
            ret = get_driver_state()
            if ret.cmd_ret == 'ERROR' or ret.cmd_ret == 'HOLDED':
                cmdhandler_client('driver', 'restart')
                rospy.sleep(3)

            ret = get_driver_state()
            self.RobotMessage.emit(f"Установлен режим {ret.cmd_ret}")

            self.PrevHand = None
            self.PrevCompress = None

            self._run_flag = True
            self.start()

    def run(self):
        while self._run_flag and not rospy.is_shutdown():
            self.GetHand.emit()

            # Пропуск идентичных запросов
            if self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                self.rate.sleep()
                continue
            self.PrevCompress = self.Compress
            self.PrevHand = self.Hand

            if self.Compress and not self.CompressFlag:
                cmdhandler_client("as", "OPEN")
                self.CompressFlag = True
                self.mgc.stop()
                self.sleep(5)
            elif not self.Compress and self.CompressFlag:
                cmdhandler_client("as", "CLOSE")
                self.CompressFlag = False
                self.mgc.stop()
                self.sleep(5)

            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = self.Hand[0]
            pose_goal.position.y = self.Hand[1]
            pose_goal.position.z = self.Hand[2]

            pose_goal.orientation.x = 1
            '''
            pose_goal.orientation.y = self.Hand[4]
            pose_goal.orientation.z = self.Hand[5]
            '''
            self.mgc.set_pose_target(pose_goal)
            self.mgc.go()
            self.mgc.clear_pose_targets()

            self.rate.sleep()

    def SetHand(self, CamInfo, PrecisionParam):
        x = ((self.max_cord[0] - self.min_cord[0]) / (CamInfo["height"] * PrecisionParam))\
            * (CamInfo["height"] * PrecisionParam // 2 + CamInfo["Hand"][1]) + self.min_cord[0]
        y = -((self.max_cord[1] - self.min_cord[1]) / (CamInfo["width"] * PrecisionParam)) * CamInfo["Hand"][0]
        z = (CamInfo["CalibDist"] - CamInfo["Hand"][2]) / 1000 + self.Home_pose[2]

        self.Hand = [x, y, z]
        self.Compress = CamInfo["Compress"]

        '''
        for i, cord in enumerate(self.Hand):
            if cord > self.max_cord[i]:
                self.Hand[i] = self.max_cord[i]
            elif cord < self.min_cord[i]:
                self.Hand[i] = self.min_cord[i]
        if self.HomePoseFlag:
            self.HomePoseFlag = False
        '''

    def GoHome(self):
        self.Hand = self.Home_pose
        self.Compress = False
        self.HomePoseFlag = True

    def stop(self):
        if self._run_flag:
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
