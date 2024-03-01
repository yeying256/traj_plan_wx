/*
 * @Author: your name
 * @Date: 2024-02-03 17:05:01
 * @LastEditTime: 2024-02-05 17:53:43
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ws/src/traj_plan_wx/src/controller..cpp
 */
#include "traj_plan_wx/controller.h"
namespace plan_wx{

    controller::controller(/* args */):pid_{{1.0,0.0,0.0,1.0},{1.0,0.0,0.0,1.0},{1.0,0.0,0.0,1.0}}
    {
        geometry_msgs::Twist cmd_vel;
        
        ;
    }
    
    
    controller::~controller()
    {
        ;
    }

    
    /**
     * @brief 
     * 
     * @param path 传入的路径 
     * @param pos_d 最终目标位姿
     * @param pos_now 当前的位姿 
     * @return geometry_msgs::Twist cmd_vel 
     */
    geometry_msgs::Twist controller::line2cmd_vel(const nav_msgs::Path& path,
                                                const geometry_msgs::Pose& pos_d,
                                                const geometry_msgs::Pose& pos_now)
    {
        // 
        geometry_msgs::Twist cmd_vel;

        // 
        Eigen::Vector3d target_pos;
        tf2::Quaternion qd_tmp;

        // 初始化当前姿态
        tf2::Quaternion pos_now_tf(pos_now.orientation.x,pos_now.orientation.y,pos_now.orientation.z,pos_now.orientation.w);

        std::cout<<"pos_now  = "<<pos_now.position<<std::endl;
        std::cout<<"pos_d  = "<<pos_d.position<<std::endl;


        // 计算距离
        double distance_d_e = sqrt(pow((pos_d.position.x - pos_now.position.x),2) 
                            + pow((pos_d.position.y - pos_now.position.y),2));


        
        /*
        
        
        
        int idenx_target = 20;
        if (path.poses.size()>idenx_target)
        {
            target_pos[0] = path.poses[idenx_target].pose.position.x;
            target_pos[1] = path.poses[idenx_target].pose.position.y;
            // qd_tmp.setW(path.poses[idenx_target].pose.orientation.w);
            // qd_tmp.setX(path.poses[idenx_target].pose.orientation.x);
            // qd_tmp.setY(path.poses[idenx_target].pose.orientation.y);
            // qd_tmp.setZ(path.poses[idenx_target].pose.orientation.z);

            qd_tmp.setRPY(0,0,atan(     (target_pos[1] - path.poses[idenx_target-1].pose.position.y) 
                                    /(target_pos[0] - path.poses[idenx_target-1].pose.position.x) )
                                    );

            
        }else
        {
            target_pos[0] = (path.poses.end()-1)->pose.position.x;
            target_pos[1] = (path.poses.end()-1)->pose.position.y;
            // qd_tmp.setW((path.poses.end()-1)->pose.orientation.w);
            // qd_tmp.setX((path.poses.end()-1)->pose.orientation.x);
            // qd_tmp.setY((path.poses.end()-1)->pose.orientation.y);
            // qd_tmp.setZ((path.poses.end()-1)->pose.orientation.z);
            
            if(path.poses.size()<=2)
            {
                std::cout<<"出错了"<<std::endl;
            }else
            {
                qd_tmp.setRPY(0,0,atan(     (target_pos[1] - (path.poses.end()-2)->pose.position.y) 
                    /(target_pos[0] - (path.poses.end()-2)->pose.position.x) )  );
            }
            
        }

        */

        if (distance_d_e>0.3)
        {
            target_pos[0] = path.poses[1].pose.position.x;
            target_pos[1] = path.poses[1].pose.position.y;
            std::cout<<"distance_d_e>1.0"<<std::endl;

            // qd_tmp.setRPY(0,0,atan(     ( (path.poses[2].pose.position.y)-target_pos[1])
            // /( (path.poses[2].pose.position.x)- target_pos[0])  ) );
        }else
        {
            target_pos[0] = pos_d.position.x;
            target_pos[1] = pos_d.position.y;
        }
        qd_tmp.setValue(pos_d.orientation.x,pos_d.orientation.y,pos_d.orientation.z,pos_d.orientation.w);




        // 世界坐标系下的目标

        
        //设置

        // 计算旋转误差
        // tf::getYaw(qd_tmp);
        
        // double qd_yaw = tf2::getYaw(qd_tmp.x(),qd_tmp.y(),qd_tmp.z(),qd_tmp.w());

        tf2::Matrix3x3 qd_matrix(qd_tmp);
        tf2::Matrix3x3 qnow_matrix(pos_now_tf);


        tf2::Matrix3x3 err = qd_matrix * qnow_matrix.inverse();
        double tmp,tmp2, yaw_err;
        err.getRPY(tmp,tmp2,yaw_err);
        // std::cout<< "err_angle = "<<yaw_err <<std::endl;
        // err_angle>M_PI ? (err_angle =  err_angle - 2*M_PI) : err_angle;
        //计算移动误差
        tf2::Transform T_w2r;//世界坐标系转化为机器人坐标系
        T_w2r.setRotation(pos_now_tf);
        // T_w2r.setOrigin(tf2::Vector3(pos_now.position.x,pos_now.position.y,0));
        // 世界坐标系移动误差
        tf2::Vector3 err_trans;
        err_trans[0] = target_pos[0] - pos_now.position.x;
        err_trans[1] = target_pos[1] - pos_now.position.y;

        //将世界坐标系下的误差描述转换为机器人坐标系下
        err_trans = T_w2r.inverse() * err_trans;


        cmd_vel.linear.x = pid_[0].PID(err_trans[0]);
        cmd_vel.linear.y =pid_[1].PID(err_trans[1]);
        cmd_vel.angular.z = pid_[2].PID(yaw_err);
        
        // std::cout<< "cmd_vel x = "<< cmd_vel.linear.x<<"cmd_vel y ="<<cmd_vel.linear.y<<"cmd_vel wz = "<<cmd_vel.angular.z<<std::endl;
        return cmd_vel;

    }




}