#ifndef GUSS_PLAN_H
#define GUSS_PLAN_H
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include "traj_plan_wx/include_common.h"
#include "traj_plan_wx/map_guass.h"
#include "map_guass.h"
#include "nav_msgs/Path.h"

namespace plan_wx{
    class guass_plan
    {
    private:
        
        struct
        {
            int DOF;//自由度数
            int global_num_particles;//生成的随机曲线的条数
            int global_num_waypoints;//中间点的数量
            int global_iterations;//迭代次数
            bool prior_update_flag = false;
            bool goal_change_index = false;
            bool init_meanPath_flag = false;
        }guass_base_param_;

        // 高斯核参数
        struct KernelParam
        {
            double l=0.05;
            double sigma = 4e-2;
            double sigma_max = 4;
            double sigma_min = 1e-3;
            double sigma_scale = 1; 
            /* data */
        }guass_kernelParam_;
        
        struct Guass_data{
            Eigen::MatrixXd L_posterior_;
            std::vector<Eigen::MatrixXd> path_generate_;
            // 生成的样本点，用来生成轨迹
            std::vector<Eigen::MatrixXd> normal_batch_samples_;
                    
            Eigen::MatrixXd path_mean_;// 平均路径
        }guass_data_;

        // 代价地图
        MapGuass map_;

        // 先验高斯过程
        Multi_val_dist gp_prior_;
        

        
        
 
        Eigen::VectorXd start_pos_,target_pos_;//设置目标位置和当前位置
        std::vector<Eigen::MatrixXd> global_mean_path_all_list_;    //均值路径
        std::vector<Eigen::MatrixXd> global_sample_all_list_;   //中间点
        double dt_;//控制间隔时间？
        
    public:
        
        guass_plan(/* args */);
        /**
         * @brief 构造函数， 
         * 
         * @param dof 自由度数目
         * @param global_num_particles 粒子数，进行高斯过程采样点的数量 
         * @param global_num_waypoints 采样时将一整条线段分成多少段
         * @param global_iterations 迭代次数
         */
        guass_plan(int dof,int global_num_particles,int global_num_waypoints, int global_iterations);

        /**
         * @brief 规划主函数
         * 
         * @param start 开始点
         * @param end 结束点
         * @param iterations 迭代次数 
         * @return std::vector<Eigen::VectorXd> 返回规划的轨迹
         */
        std::vector<Eigen::VectorXd> global_mpc_planner(Eigen::VectorXd start,Eigen::VectorXd end,int iterations);

        /**
         * @brief Get the initial mean and samples object
         * 
         * @param target 目标
         * @param path_mean 传出参数 平均轨迹，也就是中间那条轨迹
         * @param path_simpal 传出参数 所有生成的高斯轨迹
         */
        void get_initial_mean_and_samples(Eigen::VectorXd target);

        /**
         * @brief 高斯先验分布 将内部参数gp_prior_初始化
         * 
         */
        void global_gp_prior();

        /**
         * @brief 使用内部参数求解协方差矩阵 高斯核
         * 
         * @param a 一个向量
         * @param b 另一个向量
         * @return Eigen::MatrixXd 协方差矩阵 
         */
        Eigen::MatrixXd RBF_kernel(Eigen::VectorXd a,Eigen::VectorXd b);

        /**
         * @brief 更新内部参数 选出代价最低的轨迹
         * 
         */
        void optimize_time_normalized();

        /**
         * @brief 更新参数和轨迹
         * 
         * @param data 
         */
        void optimize_path(struct Guass_data &data);

        /**
         * @brief 更新guass_kernelParam_.sigma 和 guass_kernelParam_.sigma_scale,分别用于高斯核生成和高斯过程生成。
         * 
         * @param cost 
         * @return double 
         */
        double evolutionary_strategy(double cost);

        /**
         * @brief Get the path mean object获取均值曲线
         * 
         * @return nav_msgs::Path 均值曲线
         */
        nav_msgs::Path get_path_mean();

        /**
         * @brief Get the path all object
         * 
         * @return std::vector< nav_msgs::Path> 所有的曲线 
         */
        std::vector< nav_msgs::Path> get_path_all();

        




        ~guass_plan();
    };

    
    


}



#endif