#ifndef PLAN_H
#define PLAN_H

#include <ros/ros.h>
#include "traj_plan_wx/guass_plan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "traj_plan_wx/controller.h"
#include <nav_msgs/Odometry.h>

#include <mutex>

#include <ros/ros.h>


namespace plan_wx{
    class plan
    {
    private:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub_;

        // 当前的机器人状态
        nav_msgs::Odometry robot_state_now_;
        
        // 里程计状态Subscriber,主要用于速度
        ros::Subscriber odom_sub_;

        std::string frame_map_,frame_robot_;

        // tf
        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;

        // rate
        std::shared_ptr<ros::Rate> rate_;



        // 接收rviz的目标
        ros::Subscriber goal_subscriber_;

        enum State{
            INIT =0,
            WAIT,
            RUNNING,
            STOP
        }state_,state_last_;
        geometry_msgs::Pose target_pose_;

        // publisher
        // 最优路径显示
        ros::Publisher path_pub_good_;
        ros::Publisher path_pub_all_ ;
        ros::Publisher path_pub_cubic_;

        plan_wx::controller cmd_;


    public:
        plan(/* args */);
        plan(ros::NodeHandle &nh);
        // plan_wx::guass_plan* plan_;
        std::shared_ptr<plan_wx::guass_plan> plan_;

        /**
         * @brief 主要运行线程
         * 
         */
        void run();

        /**
         * @brief 里程计topic更新，主要用来获取速度
         * 
         * @param odom_msg 
         */
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        ~plan();
        bool lookupTransform(const std::string& target_frame, const std::string& source_frame, 
                            const ros::Time& time, geometry_msgs::TransformStamped& transform);
        
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    };
    

    
}

#endif