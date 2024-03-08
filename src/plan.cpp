#include "traj_plan_wx/plan.h"


namespace plan_wx{
    plan::plan(/* args */)
    {

    }


    
    plan::~plan()
    {
    }

    plan::plan(ros::NodeHandle &nh):buffer_(new tf2_ros::Buffer())
    {
        this->nh_ = nh;
        // 创建tf监听器
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, nh_);
        std::string topic_name_cmd_vel;
        std::string topic_odom;
        int dof,global_num_particles,global_num_waypoints,global_iterations;
        double rate =50;
        
        if(!nh_.getParam("guass_plan_wx/topic/cmd_vel",topic_name_cmd_vel))
        {
            ROS_ERROR_STREAM("get name of topic cmd_vel error");
            topic_name_cmd_vel = "/cmd_vel";
        }
        if(!nh_.getParam("guass_plan_wx/topic/odom",topic_odom))
        {
            ROS_ERROR_STREAM("get name of topic topic_odom error");
            topic_odom = "/odom";
        }
        int default_val = 2;
        if(!nh_.getParam("guass_plan_wx/plan/dof",dof))
        {
            ROS_ERROR_STREAM("get name of dof error");
            dof = 2;
        }
        if(!nh_.getParam("guass_plan_wx/plan/global_num_particles",global_num_particles))
        {
            ROS_ERROR_STREAM("get name of global_num_waypoints error");
            global_num_particles = 200;
        }if(!nh_.getParam("guass_plan_wx/plan/global_num_waypoints",global_num_waypoints))
        {
            ROS_ERROR_STREAM("get name of global_iterations error");
            global_num_waypoints = 8;
        }if(!nh_.getParam("guass_plan_wx/plan/global_iterations",global_iterations))
        {
            ROS_ERROR_STREAM("get name of plan cmd_vel error");
            global_iterations = 1;
        }
        if(!nh_.getParam("guass_plan_wx/plan/rate",rate))
        {
            ROS_ERROR_STREAM("get rate of plan error");
            rate = 50.0;
        }


        // 获取坐标系
        if(!nh_.getParam("guass_plan_wx/frame/robot",frame_robot_))
        {
            ROS_ERROR_STREAM("get name of frame_robot_ error");
            frame_robot_ = "base_footprint";
        }
        if(!nh_.getParam("guass_plan_wx/frame/map",frame_map_))
        {
            ROS_ERROR_STREAM("get name of frame map error");
            frame_map_ = "map";
        }
        
        

        // 速度指令topic
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name_cmd_vel, 10);
        path_pub_good_ = nh_.advertise<nav_msgs::Path>("path_topic_good", 10);
        path_pub_all_ = nh_.advertise<nav_msgs::Path>("path_topic_all", 10);
        path_pub_cubic_ = nh_.advertise<nav_msgs::Path>("path_cubic", 10);


        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(topic_odom, 10, &plan::odomCallback,this);
        goal_subscriber_ = nh_.subscribe("/move_base_simple/goal", 1, &plan::goalCallback, this);
        
        //监听当前机器人位姿
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // 初始化高斯规划过程
        // plan_ = new plan_wx::guass_plan(dof,global_num_particles,global_num_waypoints,global_iterations);
        // plan_wx::guass_plan plan(dof,global_num_particles,global_num_waypoints,global_iterations);
        // 使用智能指针分配内存，这样就不用去析构函数中delet了
        plan_ = std::make_shared<plan_wx::guass_plan>(dof,global_num_particles,global_num_waypoints,global_iterations);
        
        rate_ = std::make_shared<ros::Rate>(rate);

        // 从参数服务器中加载参数
        plan_->init_param_ros(nh_);

        state_ = WAIT;

    }


    void plan::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        // 解析Odometry消息内容
        // geometry_msgs::Pose pose = odom_msg->pose.pose;
        // geometry_msgs::Point position = pose.position;
        // geometry_msgs::Quaternion orientation = pose.orientation;
        // geometry_msgs::Twist twist = odom_msg->twist.twist;
        // geometry_msgs::Vector3 linear_velocity = twist.linear;
        // geometry_msgs::Vector3 angular_velocity = twist.angular;
        std::mutex mutex;
        mutex.lock();
        // 将数据读进来
        this->robot_state_now_ = *odom_msg;
        mutex.unlock();
        

    }


    bool plan::lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                       const ros::Time& time, geometry_msgs::TransformStamped& transform) 
    {
        try {
        // 获取目标坐标系相对于源坐标系的变换
        transform = buffer_->lookupTransform(target_frame, source_frame, time);
        return true;
        } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
        }
    }
    void plan::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
    ROS_INFO("Received new navigation goal from RViz:");
    ROS_INFO_STREAM("Position: (" << msg->pose.position.x << ", " << msg->pose.position.y << ")");
    ROS_INFO_STREAM("Orientation: (" << msg->pose.orientation.x << ", " << msg->pose.orientation.y << ", "
                                       << msg->pose.orientation.z << ", " << msg->pose.orientation.w << ")");
    target_pose_ = msg->pose;
    // 在这里添加处理目标点的逻辑，例如转发给导航栈或者执行其他动作
    std::mutex mutex;

    mutex.lock();
    if (state_ == State::WAIT)
    {
        state_ = RUNNING;
    }
    mutex.unlock();
    }

    void plan::run()
    {

        while (ros::ok())
        {

        
        geometry_msgs::Twist cmd_vel;
        
        // 获得坐标变换
        geometry_msgs::TransformStamped transformStamped = this->buffer_->lookupTransform(
            frame_map_,  // 源坐标系
            frame_robot_,  // 目标坐标系
            ros::Time(0), // 查询当前时间或指定时间戳的变换
            ros::Duration(0.001)); // 超时等待时间，这里设置为1秒
        // 给当前和目标状态赋值
        Eigen::Vector2d pose_now,pose_end; 
        pose_now(0) = transformStamped.transform.translation.x;
        pose_now(1) = transformStamped.transform.translation.y;

        pose_end(0) = this->target_pose_.position.x;
        pose_end(1) = this->target_pose_.position.y;

        

        // Eigen::Vector2d pose_now,pose_end;


        switch (this->state_)
        {
        case WAIT:
        {
            // 如果切到了等待状态，那么就等待
            std::cout<<"等待状态"<<std::endl;
            if (state_last_ != WAIT)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.linear.z = 0;
                cmd_vel.angular.x = 0;
                cmd_vel.angular.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_.publish(cmd_vel);
            }
            
            break;
        }
        case RUNNING:
        {
            std::cout<<"运行状态"<<std::endl;

            plan_->global_mpc_planner(pose_now,pose_end,1);
            // 获取最优轨迹,并发布
            nav_msgs::Path path_mean = plan_->get_path_mean();
            path_pub_good_.publish(path_mean);

            // 
            geometry_msgs::Pose pose_now_tf;
            pose_now_tf.orientation = transformStamped.transform.rotation;
            pose_now_tf.position.x = transformStamped.transform.translation.x;
            pose_now_tf.position.y = transformStamped.transform.translation.y;

            Eigen::Vector2d vel_now;
            vel_now(0) = this->robot_state_now_.twist.twist.linear.x;
            vel_now(1) = this->robot_state_now_.twist.twist.linear.y;

            cmd_vel = cmd_.line2cmd_vel(path_mean,
                                        target_pose_,
                                        pose_now_tf,
                                        vel_now,
                                        plan_->map_.get_min_cost());
            path_pub_cubic_.publish(cmd_.path_cubic_);

            // 获取控制优化后的新点
            Eigen::MatrixXd new_point_cubic = cmd_.get_new_point();
            // std::cout<<"new_point_cubic = "<<new_point_cubic<<std::endl;
            plan_->updata_opt_path(new_point_cubic);


            cmd_vel_pub_.publish(cmd_vel);
            // 如果到达了目标，则转化为等待状态，然后讲cmdvel发出0速指令
            if (cmd_.if_get_target())
            {
                cmd_vel.angular.z = 0;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                state_ = WAIT;
                cmd_vel_pub_.publish(cmd_vel);
            }
            break;
        }
        case STOP:
        {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0;

            cmd_vel_pub_.publish(cmd_vel);
            break;
        }
        default:
            break;
        }
        state_last_ = state_;
        ;
        rate_->sleep();
        }
    }

}