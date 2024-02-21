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

#include <tf/tf.h>

namespace plan_wx{
    class controller
    {
    private:
        xj_dy_ns::PID_controller pid_[3];
    public:
        controller(/* args */);
        ~controller();

        
        geometry_msgs::Twist line2cmd_vel(const nav_msgs::Path& path,
        const geometry_msgs::Pose& pos_d,
        const geometry_msgs::Pose& pos_now);
    };
    

    
}



#endif