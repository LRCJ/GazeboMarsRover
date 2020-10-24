Mars Rover Simulation In Gazebo
===
　　参考了项目[curiosity_mars_rover](https://bitbucket.org/theconstructcore/curiosity_mars_rover.git)/[Curiosity Mars Rover](https://rds.theconstructsim.com/r/bb25b9334032/curiosity_mars_rover_1/)， 
基于gazebo和ros搭建火星车地形感知仿真平台，
结合urdf和sdf描述文件实现了摇臂转向架式火星车的悬架差速效果(参考[Mars Rover Rocker-Bogie Differential](http://alicesastroinfo.com/2012/07/mars-rover-rocker-bogie-differential/)，
受限于urdf/sdf本身的缺陷，只能实现Differential Bar，而无法实现Differential Gearbox)；同时添加了激光雷达传感器，模拟出3D激光雷达的效果，
添加激光雷达传感器时参考了GitHub项目[velodyne_simulator](https://github.com/lmark1/velodyne_simulator.git)，
地形模型来源于NASA的[3D资源库](https://nasa3d.arc.nasa.gov/)。<br>
　　由于火星车没有涉及转向机构，驱动车运动使用左右车轮差速转向，使用插件"libgazebo_ros_skid_steer_drive.so"(位于"/opt/ros/melodic/lib/")，源码见[gazebo_ros_skid_steer_drive.cpp](http://docs.ros.org/en/jade/api/gazebo_plugins/html/gazebo__ros__skid__steer__drive_8cpp_source.html)，通过给Topic /cmd_vel设置linear.x和angular.z即可驱动车运动。另外，添加了键盘驱动车辆运动的功能，参考ros例程turtlesim的turtle_teleop_key节点[源码](https://docs.ros.org/en/melodic/api/turtlesim/html/teleop__turtle__key_8cpp_source.html)和[linux下C实现对键盘事件的监听（按下键盘的时候程序立刻读取）_借你一秒-CSDN博客](https://blog.csdn.net/u013467442/article/details/51173441) 。
* 还有，编写该README时参考了博客[GitHub上README.md排版样式教程_需要时间-CSDN博客](https://blog.csdn.net/u012067966/article/details/50736647)，
[github中README.md书写规范](https://www.cnblogs.com/guchunli/p/6371040.html)。
