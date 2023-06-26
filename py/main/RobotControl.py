from PyQt5.QtCore import QThread, pyqtSignal

import os
import math

os.system(
    "gnome-terminal --tab -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
    "roscore\"")

QThread.sleep(2)

import rospy
import geometry_msgs.msg
import moveit_msgs.msg
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
        self.InitFlag = False

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
        self.frequency = 15
        self.rate = rospy.Rate(self.frequency)

    def RobotConnect(self, RobotModel, SimulationFlag, RobotIp="192.168.0.2"):
        if not self.ConnectFlag:
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
            os.system(
                f"gnome-terminal --tab -- bash -c \"roslaunch khi_{RobotModel}_moveit_config moveit_planning_execution.launch\"")
            self.sleep(5)
            self.ConnectFlag = True
        if not self.InitFlag:
            self.InitMoveGroupCommander(RobotModel, SimulationFlag)

    def InitMoveGroupCommander(self, RobotModel, SimulationFlag):
        ######################################### set range of move #####################################################
        # region
        # self.khi_robot = KhiRobot()
        self.group = 'manipulator'

        # RobotCommander
        self.rc = moveit_commander.RobotCommander()

        # MoveGroupCommander
        self.mgc = moveit_commander.MoveGroupCommander(self.group)

        if SimulationFlag:
            # Для симуляции переводим робота домашнее положение
            self.mgc.set_joint_value_target({"joint1": -90 * math.pi / 180, "joint2": -30 * math.pi / 180,
                                             "joint3": 90 * math.pi / 180, "joint4": 0,
                                             "joint5": 0, "joint6": 0})
            self.mgc.go()
            self.mgc.clear_pose_targets()
        else:
            # Проверяем, что робот находится внутри установленной рабочей области
            self.mgc.set_start_state(self.rc.get_current_state())
            current_state = self.mgc.get_current_joint_values()
            if (-140*math.pi/180) < current_state[0] < (-40 * math.pi / 180)\
                    and (-90 * math.pi / 180) < current_state[1] < (40 * math.pi / 180)\
                    and (0 * math.pi / 180) < current_state[2] < (2 * math.pi)\
                    and (-10 * math.pi / 180) < current_state[3] < (10 * math.pi / 180):
                pass
            else:
                self.RobotMessage.emit("Робот находится за пределами установленной рабочей области.")
                return

        # mgc setting
        self.mgc.set_planner_id(planner)
        # self.mgc.set_goal_joint_tolerance(self.accuracy_jt)
        # mgc.set_goal_position_tolerance(accuracy_pos)
        # mgc.set_goal_orientation_tolerance(accuracy_ori)

        self.mgc.set_max_velocity_scaling_factor(self.min_vel)
        self.mgc.set_max_acceleration_scaling_factor(self.min_acc)
        self.mgc.set_max_velocity_scaling_factor(self.max_vel)
        self.mgc.set_max_acceleration_scaling_factor(self.max_acc)

        self.mgc.set_workspace([*self.min_cord, *self.max_cord])
        # self.mgc.set_num_planning_attempts(2)
        # self.mgc.set_planning_time(1)

        # Ограничения врашения звеньев
        joint_constraint_list = [joint_limits(self.mgc.get_joints()[0], -90, 30, 30),
                                 joint_limits(self.mgc.get_joints()[3], 0, 1, 1)]
        constraint_list = moveit_msgs.msg.Constraints()
        constraint_list.name = 'middle_of_travel'
        constraint_list.joint_constraints = joint_constraint_list
        self.mgc.set_path_constraints(constraint_list)

        self.InitFlag = True
        self.RobotMessage.emit("Успешно подключено!")
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
            self.GoHome()

            self._run_flag = True
            self.start()

    def run(self):
        while self._run_flag and not rospy.is_shutdown():
            self.GetHand.emit()
            self.rate.sleep()

            # Пропуск идентичных запросов
            if self.HomePoseFlag:
                if self.CurHomePoseFlag:
                    # rospy.loginfo("Continue")
                    continue
                else:
                    self.CurHomePoseFlag = True
            elif self.PrevCompress == self.Compress and self.PrevHand == self.Hand:
                # rospy.loginfo("Continue")
                continue
            elif self.CurHomePoseFlag:
                self.CurHomePoseFlag = False

            self.PrevCompress = self.Compress
            self.PrevHand = self.Hand

            self.mgc.set_start_state(self.rc.get_current_state())

            if self.Compress and not self.CompressFlag:
                cmdhandler_client("as", "CLOSE")
                self.CompressFlag = True
                self.mgc.stop()
                self.sleep(2)
            elif not self.Compress and self.CompressFlag:
                cmdhandler_client("as", "OPEN")
                self.CompressFlag = False
                self.mgc.stop()
                self.sleep(2)

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
            #rospy.loginfo("Success")

    def SetHand(self, CamInfo, PrecisionParam):
        x = ((self.max_cord[0] - self.min_cord[0]) / (CamInfo["height"] * PrecisionParam))\
            * (CamInfo["height"] * PrecisionParam // 2 - CamInfo["Hand"][1]) + self.min_cord[0]
        y = ((self.max_cord[1] - self.min_cord[1]) / (CamInfo["width"] * PrecisionParam)) * CamInfo["Hand"][0]
        z = (CamInfo["CalibDist"] - CamInfo["Hand"][2]) / 1000 + self.Home_pose[2]
        '''
        x = CamInfo["height"]/100
        y = CamInfo["Hand"][0]/100
        z = (CamInfo["CalibDist"] - CamInfo["Hand"][2]) / 100 + self.Home_pose[2]
        '''

        self.Hand = [x, y, z]
        self.Compress = CamInfo["Compress"]

        for i, cord in enumerate(self.Hand):
            if cord > self.max_cord[i]:
                self.Hand[i] = self.max_cord[i]
            elif cord < self.min_cord[i]:
                self.Hand[i] = self.min_cord[i]

        if self.HomePoseFlag:
            self.HomePoseFlag = False

    def GoHome(self):
        self.Hand = self.Home_pose
        self.Compress = False
        self.HomePoseFlag = True
        self.CurHomePoseFlag = False

    def stop(self):
        if self._run_flag:
            """Sets run flag to False and waits for thread to finish"""
            self._run_flag = False
            self.wait()
            cmdhandler_client('driver', 'hold')
            ret = get_driver_state()
            self.RobotMessage.emit(f"Установлен режим {ret.cmd_ret}")
            rospy.sleep(3)

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

def joint_limits(joint, pos, below, above):
    joint_constraint = moveit_msgs.msg.JointConstraint()
    joint_constraint.joint_name = joint
    joint_constraint.position = pos * math.pi / 180
    joint_constraint.tolerance_below = below * math.pi / 180
    joint_constraint.tolerance_above = above * math.pi / 180
    joint_constraint.weight = 1
    return joint_constraint