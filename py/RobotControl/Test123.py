from itertools import chain
'''
os.system(
    "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
    "roscore; exec bash\"'")
os.system(
                "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch simulation:=true; exec bash\"'") 

os.system(
                "gnome-terminal -e 'bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
                f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch ip:={RobotIp}; exec bash\"'")
                
'''
'''
os.system("gnome-terminal -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
          "roscore\"")

os.system("gnome-terminal -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
          f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch simulation:=true\"")
os.system("gnome-terminal -- bash -c \"source /home/user/Documents/GitHub/HandTracker/py/ros_resources/devel/setup.bash; "
          f"roslaunch khi_robot_bringup {RobotModel}_bringup.launch ip:={RobotIp}\"")

os.system(f"gnome-terminal --tab -- bash -c \"roslaunch khi_{RobotModel}_moveit_config moveit_planning_execution.launch\"")
'''
