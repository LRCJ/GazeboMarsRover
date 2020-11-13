Mars Rover Simulation In Gazebo
===
　　参考了项目[curiosity_mars_rover](https://bitbucket.org/theconstructcore/curiosity_mars_rover.git)/[Curiosity Mars Rover](https://rds.theconstructsim.com/r/bb25b9334032/curiosity_mars_rover_1/)， 
基于Gazebo和ROS搭建火星车地形感知仿真平台，
结合urdf和sdf描述文件实现了摇臂转向架式火星车的悬架差速效果(参考[Mars Rover Rocker-Bogie Differential](http://alicesastroinfo.com/2012/07/mars-rover-rocker-bogie-differential/)，
受限于URDF/SDF本身的缺陷，只能实现Differential Bar，而无法实现Differential Gearbox)；同时添加了激光雷达传感器，模拟出3D激光雷达的效果，
添加激光雷达传感器时参考了GitHub项目[velodyne_simulator](https://github.com/lmark1/velodyne_simulator.git)，
地形模型来源于NASA的[3D资源库](https://nasa3d.arc.nasa.gov/)。<br>
　　由于火星车没有设计转向机构，驱动车运动使用左右车轮差速转向，使用插件"libgazebo_ros_skid_steer_drive.so"(位于ROS安装目录下的lib目录下)，源码见[gazebo_ros_skid_steer_drive.cpp](http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__skid__steer__drive_8cpp_source.html)，通过给Topic /cmd_vel设置linear.x和angular.z即可驱动车运动。另外，通过在本地修改编译该插件的源码，添加了键盘控制车运动的功能，参考ros例程turtlesim的turtle_teleop_key节点[源码](https://docs.ros.org/en/melodic/api/turtlesim/html/teleop__turtle__key_8cpp_source.html)和[linux下C实现对键盘事件的监听（按下键盘的时候程序立刻读取）](https://blog.csdn.net/u013467442/article/details/51173441)；以及游戏手柄控制车运动的功能，程序参考了[[joysticker]使用Ubuntu读取USB手柄/方向盘的输出控制](https://blog.csdn.net/weixin_39449466/article/details/80628534)。
* 关于键盘、手柄控制车运动功能的描述：上下左右键控制车以恒定的速度、角速度前进或者转弯，前进即↑键，左转即←键，同时按下是无法实现既前进又左转的，速度可通过设定ROS参数服务器中的"Speed"和"RotationSpeed"来改变；手柄控制，左摇杆控制前进后退，右摇杆控制左转右转，根据摇杆摇动幅度改变速度、角速度，其最大速度也是通过设定ROS参数服务器中的那两个参数来改变的；更详细的说明参考程序代码，功能实现在"plugins/SkidSteerDrive/src/SkidSteerDrive.cpp"的scanKeyboard()和scanXBOX()函数中。
* 还有，编写该README时参考了博客[GitHub上README.md排版样式教程_需要时间-CSDN博客](https://blog.csdn.net/u012067966/article/details/50736647)，
[github中README.md书写规范](https://www.cnblogs.com/guchunli/p/6371040.html)，[kaivin/markdown](https://github.com/kaivin/markdown/blob/master/readme.md)，[一个支持实时预览的在线Markdown编辑器 - Markdoc](http://weareoutman.github.io/markdoc/)，首行缩进是把输入法设置为全角再输入空格。
* 这里吐槽一下，编写这个README的时候，碰到比如这样一段"/opt/ros/melodic/lib"，如果恰好位于行尾而不能全部显示时会将这一整段放到下一行，导致本行末尾一大段的空缺！！！
