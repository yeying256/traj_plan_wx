/*
 * @Author: your name
 * @Date: 2024-02-03 17:05:15
 * @LastEditTime: 2024-02-05 17:37:41
 * @LastEditors: wangxiao-pc
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/traj_plan_wx/include/traj_plan_wx/controller.h
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H


#include "traj_plan_wx/include_common.h"
#include "traj_plan_wx/map_guass.h"
#include "map_guass.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include "Cubic_Spline.h"

#include <tf/tf.h>

namespace plan_wx{
    class controller
    {
    private:
        xj_dy_ns::PID_controller pid_[3];
    public:
        controller(/* args */);
        ~controller();
        nav_msgs::Path path_cubic_;
        /**
         * @brief 
         * 
         * @param path 传入的路径 
         * @param pos_d 最终目标位姿
         * @param pos_now 当前的位姿 
         * @param vel 当前速度
         * @return geometry_msgs::Twist cmd_vel 
         */
        geometry_msgs::Twist line2cmd_vel(const nav_msgs::Path& path,
        const geometry_msgs::Pose& pos_d,
        const geometry_msgs::Pose& pos_now,
        const Eigen::Vector2d& vel);

        /**
         * @brief 将二维的机器人在世界坐标系下的控制指令描述出来
         * 
         * @param traj 
         * @param mean_vel 
         * @param vel_now 
         * @param target_time 目标时间是多少，规划出来时间之后取哪一个时间点作为控制点
         * @return Eigen::Array2d 
         */
        Eigen::VectorXd cubic_2d_target(Eigen::MatrixXd traj,
                                    double mean_vel,
                                    Eigen::Vector2d vel_now,
                                    double target_time);
        
    };
    

    
}



#endif