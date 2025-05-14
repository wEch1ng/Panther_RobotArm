# U2C接入

先插入1-3关节的U2C弹出设备后连接至虚拟机 再插入4-7关节的U2C

ls /dev/ttyACM*
sudo chmod -R 777 /dev/ttyACM*


使用第一条指令

查看是不是弹出

/dev/ttyACM0  /dev/ttyACM1

弹出后再输入第二条指令获得设备使用权限



# 编译工作空间

cd panther_ws/
catkin_make

# 重力补偿

开两个终端分别输入

roscore

rosrun dm_drive ZeroGraMode


# 拖动示教

开两个终端分别输入

roscore

rosrun dm_drive teach

# Moveit控制

开两个终端分别输入

rosrun dm_drive dm_controller

roslaunch panther_moveit_config demo.launch

