/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_skid_steer_drive.cpp
 *
 * \brief A skid steering drive plugin. Inspired by gazebo_ros_diff_drive and SkidSteerDrivePlugin
 *
 * \author  Zdenek Materna (imaterna@fit.vutbr.cz) (modified by LR renliaocn@gmail.com)
 *
 * $ Id: 06/25/2013 11:23:40 AM materna $
 */

#include "SkidSteerDrive.h"

namespace gazebo
{

    enum
    {
        RIGHT_FRONT=0,
        LEFT_FRONT=1,
        RIGHT_REAR=2,
        LEFT_REAR=3,
    };

    GazeboRosSkidSteerDrive::GazeboRosSkidSteerDrive()
    {
        // Initialize velocity stuff
        wheel_speed_[RIGHT_FRONT] = 0;
        wheel_speed_[LEFT_FRONT] = 0;
        wheel_speed_[RIGHT_REAR] = 0;
        wheel_speed_[LEFT_REAR] = 0;

        x_ = 0;
        rot_ = 0;
        //alive_ = true;
    }

    GazeboRosSkidSteerDrive::~GazeboRosSkidSteerDrive()
    {
        
        delete rosnode_;
        delete transform_broadcaster_;
    }

    //扫描键盘输入，控制小车前进转弯
    void GazeboRosSkidSteerDrive::scanKeyboard()
    {
        double x__,rot__;
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(0,&stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= ~ICANON;//关闭标准输入处理，即默认的回车和换行符之间的映射已经不存在了
        //new_settings.c_lflag &= ~ECHO;//关闭回显功能
        //以下两行用于设置键入字符后无需按下回车即可使getchar立刻返回
        new_settings.c_cc[VTIME] = 0;
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);
        while(true)
        {   int c = getchar();//阻塞式的
            ros::param::get("Speed",x__);
            ros::param::get("RotationSpeed",rot__);
            switch(c)
            {
                case 0x44:
                this->x_ = 0.0;
                this->rot_ = rot__;
                ROS_INFO("TurnLeft!!!Linear Speed:%.4lfm/s,Rotation Speed:%.4lfrad/s!",this->x_,this->rot_);
                break;
                case 0x43:
                this->x_ = 0.0;
                this->rot_ = -rot__;
                ROS_INFO("TurnRight!!!Linear Speed:%.4lfm/s,Rotation Speed:%.4lfrad/s!",this->x_,this->rot_);
                break;
                case 0x41:
                ROS_DEBUG("UP");
                this->x_ = x__;
                this->rot_ = 0.0;
                ROS_INFO("Forward!!!Linear Speed:%.4lfm/s,Rotation Speed:%.4lfrad/s!",this->x_,this->rot_);
                break;
                case 0x42:
                this->x_ = -x__;
                this->rot_ = 0.0;
                ROS_INFO("BackOff!!!Linear Speed:%.4lfm/s,Rotation Speed:%.4lfrad/s!",this->x_,this->rot_);
                break;
                case 0x53:
                this->x_ = 0.0;
                this->rot_ = 0.0;
                ROS_INFO("STOP!!!Linear Speed:%.4lfm/s,Rotation Speed:%.4lfrad/s!",this->x_,this->rot_);
                break;
            }
        }
    }

    //read XBOX input
    //扫描手柄输入，控制小车前进转弯及其速度
    void GazeboRosSkidSteerDrive::scanXBOX()
    {
        double x__,rot__;
        int xbox_fd,len;
        xbox_map_t map;
        ros::param::get("Speed",x__);
        ros::param::get("RotationSpeed",rot__);
        while(1)
        {
            usleep(1000*1000);
            //ROS_INFO("started to open XBOX file!");
            xbox_fd = xbox_open("/dev/input/js0");
            if(xbox_fd!=-1)
            {
                //ROS_INFO("started to read XBOX input!");
            }
            else
            {
                //ROS_INFO("open XBOX file failed!");
                continue;
            }
            while(1)
            { 
                len = xbox_map_read(xbox_fd, &map);//阻塞式的
                if (len == -1)  
                {
                    //ROS_INFO("read XBOX input failed!");
                    break;
                }
                this->x_ = map.ly/-32767.0*x__;
                this->rot_ = map.rx/-32767.0*rot__;
            }
        }
        xbox_close(xbox_fd);
    }


    // Load the controller
    void GazeboRosSkidSteerDrive::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

        this->parent = _parent;
        this->world = _parent->GetWorld();

        //获取sdf文件中调用plugin时输入的自定义参数
        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace"))
            ROS_INFO("GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",this->robot_namespace_.c_str());
        else
            this->robot_namespace_ =_sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    
        this->broadcast_tf_ = false;
        if (!_sdf->HasElement("broadcastTF"))
        {
            if (!this->broadcast_tf_)
                ROS_INFO("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
            else 
                ROS_INFO("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());
        }
        else
            this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();

        // TODO write error if joint doesn't exist!
        this->left_front_joint_name_ = "left_front_joint";
        if(!_sdf->HasElement("leftFrontJoint"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
        else
            this->left_front_joint_name_ = _sdf->GetElement("leftFrontJoint")->Get<std::string>();

        this->right_front_joint_name_ = "right_front_joint";
        if(!_sdf->HasElement("rightFrontJoint"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
        else 
            this->right_front_joint_name_ = _sdf->GetElement("rightFrontJoint")->Get<std::string>();

        this->left_rear_joint_name_ = "left_rear_joint";
        if(!_sdf->HasElement("leftRearJoint"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
                  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
        else
            this->left_rear_joint_name_ = _sdf->GetElement("leftRearJoint")->Get<std::string>();

        this->right_rear_joint_name_ = "right_rear_joint";
        if(!_sdf->HasElement("rightRearJoint"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
        else
            this->right_rear_joint_name_ = _sdf->GetElement("rightRearJoint")->Get<std::string>();


        // This assumes that front and rear wheel spacing is identical
        /*this->wheel_separation_ = this->parent->GetJoint(left_front_joint_name_)->GetAnchor(0).Distance(
            this->parent->GetJoint(right_front_joint_name_)->GetAnchor(0));*/

        this->wheel_separation_ = 0.4;
        if(!_sdf->HasElement("wheelSeparation"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
                this->robot_namespace_.c_str(), this->wheel_separation_);
        else
            this->wheel_separation_ = _sdf->GetElement("wheelSeparation")->Get<double>();

        // TODO get this from robot_description
        this->wheel_diameter_ = 0.15;
        if(!_sdf->HasElement("wheelDiameter"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <wheelDiameter>, defaults to %f",
                this->robot_namespace_.c_str(), this->wheel_diameter_);
        else
            this->wheel_diameter_ = _sdf->GetElement("wheelDiameter")->Get<double>();

        this->torque = 5.0;
        if(!_sdf->HasElement("torque"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <torque>, defaults to %f",
                this->robot_namespace_.c_str(), this->torque);
        else 
            this->torque = _sdf->GetElement("torque")->Get<double>();

        this->command_topic_ = "cmd_vel";
        if(!_sdf->HasElement("commandTopic"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->command_topic_.c_str());
        else 
            this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();

        this->path_topic_ = "path";
        if(!_sdf->HasElement("PathTopic"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->path_topic_.c_str());
        else
            this->path_topic_ = _sdf->GetElement("PathTopic")->Get<std::string>();

        this->odometry_topic_ = "odometry";
        if(!_sdf->HasElement("odometryTopic"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_topic_.c_str());
        else
            this->odometry_topic_ = _sdf->GetElement("odometryTopic")->Get<std::string>();

        this->odometry_frame_ = "odom";
        if(!_sdf->HasElement("odometryFrame"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->odometry_frame_.c_str());
        else
            this->odometry_frame_ = _sdf->GetElement("odometryFrame")->Get<std::string>();

        this->robot_base_frame_ = "base_footprint";
        if(!_sdf->HasElement("robotBaseFrame"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
        else
            this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();

        this->update_rate_ = 100.0;
        if(!_sdf->HasElement("updateRate"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
                this->robot_namespace_.c_str(), this->update_rate_);
        else
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

        this->covariance_x_ = 0.0001;
        if(!_sdf->HasElement("covariance_x"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_x>, defaults to %f",
                this->robot_namespace_.c_str(), covariance_x_);
        else
            covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();

        this->covariance_y_ = 0.0001;
        if(!_sdf->HasElement("covariance_y"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_y>, defaults to %f",
                this->robot_namespace_.c_str(), covariance_y_);
        else
            covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();

        this->covariance_yaw_ = 0.01;
        if(!_sdf->HasElement("covariance_yaw"))
            ROS_WARN("GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
                this->robot_namespace_.c_str(), covariance_yaw_);
        else
            covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();

        // Initialize update rate stuff
        if(this->update_rate_ > 0.0)
            this->update_period_ = 1.0 / this->update_rate_;//update_rate_ = 100,update_period_ = 0.01
        else
            this->update_period_ = 0.0;
        last_update_time_ = this->world->SimTime();

        //获取各个驱动关节的指针并进行相关设置
        joints[LEFT_FRONT] = this->parent->GetJoint(left_front_joint_name_);
        joints[RIGHT_FRONT] = this->parent->GetJoint(right_front_joint_name_);
        joints[LEFT_REAR] = this->parent->GetJoint(left_rear_joint_name_);
        joints[RIGHT_REAR] = this->parent->GetJoint(right_rear_joint_name_);

        if(!joints[LEFT_FRONT])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
            gzthrow(error);
        }

        if(!joints[RIGHT_FRONT])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
            gzthrow(error);
        }

        if(!joints[LEFT_REAR])
        {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
            gzthrow(error);
       }

       if(!joints[RIGHT_REAR])
       {
            char error[200];
            snprintf(error, 200,"GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
                this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
            gzthrow(error);
       }

        #if GAZEBO_MAJOR_VERSION > 2
            joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
            joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
            joints[LEFT_REAR]->SetParam("fmax", 0, torque);
            joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
        #else
            joints[LEFT_FRONT]->SetMaxForce(0, torque);
            joints[RIGHT_FRONT]->SetMaxForce(0, torque);
            joints[LEFT_REAR]->SetMaxForce(0, torque);
            joints[RIGHT_REAR]->SetMaxForce(0, torque);
        #endif

        // Make sure the ROS node for Gazebo has already been initialized
        if(!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        else
        {
            rosnode_ = new ros::NodeHandle(this->robot_namespace_);

            ROS_INFO("Starting GazeboRosSkidSteerDrive Plugin (ns = %s)!", this->robot_namespace_.c_str());

            transform_broadcaster_ = new tf::TransformBroadcaster();

            //发布相对世界坐标系的位姿信息
            odometry_publisher_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 2);
            //累计相对世界坐标系的位姿信息，可在rviz订阅该数据，可视化累积的位姿信息形成轨迹
            path_publisher_ = rosnode_->advertise<nav_msgs::Path>(path_topic_, 2);
            path_.header.frame_id = robot_base_frame_;
            path_.header.stamp = ros::Time::now();

            //订阅/cmd_vel topic，这里似乎不需要调用ros::spin/spinOnce就能执行订阅话题的回调函数
            // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
            //ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
            //    boost::bind(&GazeboRosSkidSteerDrive::cmdVelCallback, this, _1),ros::VoidPtr(), &queue_);
            cmd_vel_subscriber_ = rosnode_->subscribe<geometry_msgs::Twist>(command_topic_, 2, &GazeboRosSkidSteerDrive::cmdVelCallback, this);

            //创建新的线程处理
            //read key board input
            ROS_INFO("started to create a thread to process keyboard input!");
            boost::thread f1 = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::scanKeyboard, this));
            f1.detach();

            //read XBOX input
            ROS_INFO("started to create a thread to process XBOX input!");
            boost::thread f2 = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::scanXBOX, this));
            f2.detach();

            // start custom queue for diff drive
            //this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosSkidSteerDrive::QueueThread, this));

            // listen to the update event (broadcast every simulation iteration)
            this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboRosSkidSteerDrive::UpdateChild, this));

            //body->PandarQT，激光雷达PandarQT在body坐标系下的位姿
            // //30度
            // T_bp_.rotate(Eigen::Quaterniond(0.683, -0.683, 0.183, -0.183));
            // T_bp_.pretranslate(Eigen::Vector3d(.677, 0.370, 0.000));
            //65度
            T_bp_.rotate(Eigen::Quaterniond(0.596, -0.596, 0.380, -0.380));
            T_bp_.pretranslate(Eigen::Vector3d(0.716, 0.361, 0.000));
        }
    }

  // Update the controller
    void GazeboRosSkidSteerDrive::UpdateChild()
    {
        common::Time current_time = this->world->SimTime();//这是Gazebo仿真时间，等于仿真步长乘以步数
        double seconds_since_last_update = (current_time - last_update_time_).Double();
        if(seconds_since_last_update > update_period_)
        {
            publishOdometry(seconds_since_last_update);
            // Update robot in case new velocities have been requested
            getWheelVelocities();
            #if GAZEBO_MAJOR_VERSION > 2
            joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
            joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
            joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
            joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
            #else
            joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
            joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
            joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
            joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
            #endif

            last_update_time_+= common::Time(update_period_);
        }
    }
    /*
    void GazeboRosSkidSteerDrive::QueueThread()
    {
        static const double timeout = 0.01;
        while(alive_ && rosnode_->ok())
            queue_.callAvailable(ros::WallDuration(timeout));
    }
    // Finalize the controller
    void GazeboRosSkidSteerDrive::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        rosnode_->shutdown();
        callback_queue_thread_.join();
    }*/
    void GazeboRosSkidSteerDrive::getWheelVelocities()
    {
        boost::mutex::scoped_lock scoped_lock(lock);

        double vr = x_;
        double va = rot_;

        wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
        wheel_speed_[RIGHT_REAR] = vr + va * wheel_separation_ / 2.0;

        wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
        wheel_speed_[LEFT_REAR] = vr - va * wheel_separation_ / 2.0;
    }

    void GazeboRosSkidSteerDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        x_ = cmd_msg->linear.x;
        rot_ = cmd_msg->angular.z;
    }

    void GazeboRosSkidSteerDrive::publishOdometry(double step_time)
    {
        //这是ROS的时间，如果设置了参数use_sim_time为true，则ROS会使用/clock数据作为时间基准
        //此处/clock由gazebopublish，因此ROS时间与gazebo仿真时间同步
        //如果要获得真实时间，或者说操作系统时间，需要使用ros::WallTime::now()
        ros::Time current_time = ros::Time::now();

        // TODO create some non-perfect odometry!
        // getting data for robot_base_frame_ to odometry_frame_
        ignition::math::Pose3<double> pose = this->parent->WorldPose();

        //设置累计位姿信息，并使用“path_topic_”Topic发布累计位姿信息，rivz可使用该数据可视化运动轨迹
        //该位姿信息是车体body相对于世界坐标系world而言的，通过位姿转换可将轨迹转换为Lidar的轨迹
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.frame_id = "PandarQT";
        this_pose.header.stamp = current_time;
        //world->body
        Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
        T_wb.rotate(Eigen::Quaterniond(pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z()));
        T_wb.pretranslate(Eigen::Vector3d(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z()));
        //world->PandarQT
        Eigen::Isometry3d T_wp = T_wb*T_bp_;//右乘，T_bp_是在body坐标系下Lidar的位姿描述
        this_pose.pose.position.x =  T_wp.translation()(0);
        this_pose.pose.position.y =  T_wp.translation()(1);
        this_pose.pose.position.z =  T_wp.translation()(2);
        this_pose.pose.orientation.x = Eigen::Quaterniond(T_wp.rotation()).x();
        this_pose.pose.orientation.y = Eigen::Quaterniond(T_wp.rotation()).y();
        this_pose.pose.orientation.z = Eigen::Quaterniond(T_wp.rotation()).z();
        this_pose.pose.orientation.w = Eigen::Quaterniond(T_wp.rotation()).w();
        path_.poses.push_back(this_pose);
        path_publisher_.publish(path_);

        //将odometry_frame_和robot_base_frame_之间的位姿信息广播到tf tree
        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
        if(this->broadcast_tf_)
        {
            transform_broadcaster_->sendTransform(
            tf::StampedTransform(tf::Transform(qt, vt), current_time, robot_base_frame_, odometry_frame_));
        }

        //将odometry_frame_和robot_base_frame_之间的位姿信息利用“odometry_topic_”Topic发布出去
        odom_.header.stamp = current_time;
        odom_.header.frame_id = robot_base_frame_;
        odom_.child_frame_id = odometry_frame_;

        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();
        odom_.pose.pose.position.z = pose.Pos().Z();
        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();
        odom_.pose.covariance[0] = this->covariance_x_;
        odom_.pose.covariance[7] = this->covariance_y_;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
        odom_.pose.covariance[35] = this->covariance_yaw_;

        ignition::math::Vector3<double> linear,angular;
        linear = this->parent->WorldLinearVel();
        angular = this->parent->WorldAngularVel();
        odom_.twist.twist.linear.x = linear.X();
        odom_.twist.twist.linear.y = linear.Y();
        odom_.twist.twist.linear.z = linear.Z();
        odom_.twist.twist.angular.x = angular.X();
        odom_.twist.twist.angular.y = angular.Y();
        odom_.twist.twist.angular.z = angular.Z();
        odom_.twist.covariance[0] = this->covariance_x_;
        odom_.twist.covariance[7] = this->covariance_y_;
        odom_.twist.covariance[14] = 1000000000000.0;
        odom_.twist.covariance[21] = 1000000000000.0;
        odom_.twist.covariance[28] = 1000000000000.0;
        odom_.twist.covariance[35] = this->covariance_yaw_;

        odometry_publisher_.publish(odom_);
    }
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosSkidSteerDrive)
}
