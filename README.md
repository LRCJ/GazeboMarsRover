Mars Rover Simulation In Gazebo
===
　　参考了项目[curiosity_mars_rover](https://bitbucket.org/theconstructcore/curiosity_mars_rover.git)/[Curiosity Mars Rover](https://rds.theconstructsim.com/r/bb25b9334032/curiosity_mars_rover_1/)， 
基于gazebo和ros搭建火星车地形感知仿真平台，
结合urdf和sdf描述文件实现了摇臂转向架式火星车的悬架差速效果(参考[Mars Rover Rocker-Bogie Differential](http://alicesastroinfo.com/2012/07/mars-rover-rocker-bogie-differential/)，
受限于urdf/sdf本身的缺陷，只能实现Differential Bar，而无法实现Differential Gearbox)；同时添加了激光雷达传感器，模拟出3D激光雷达的效果，
添加激光雷达传感器时参考了GitHub项目[velodyne_simulator](https://github.com/lmark1/velodyne_simulator.git)，
地形模型来源于NASA的[3D资源库](https://nasa3d.arc.nasa.gov/)。
* 还有，编写该README时参考了博客[GitHub上README.md排版样式教程_需要时间-CSDN博客](https://blog.csdn.net/u012067966/article/details/50736647)，
[github中README.md书写规范](https://www.cnblogs.com/guchunli/p/6371040.html)。
