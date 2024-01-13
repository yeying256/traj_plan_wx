#ifndef INCLUDE_COMMON_H
#define INCLUDE_COMMON_H
#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include <random>
#include <vector>

namespace plan_wx{
    const std::string color_default = "\033[0m";
    const std::string color_red = "\033[31m";
    const std::string color_green = "\032[0m";
    const std::string color_blue = "\034[0m";

    struct Point_robot_state{
        double x;
        double y;
        double yaw;
        double dx;
        double dy;
        double dyaw;
        // s
        double cost_point;
    };
    struct traj_str{
        double cost_all;//记录整条轨迹的代价
        int size;//记录点的个数
        std::vector<Point_robot_state> point_stamp;
    };

    class Tool_wx
    {
    private:
        /* data */
    public:
    /**
     * @brief
     * 该函数接受两个Eigen::VectorXd类型的初始值和结束值以及一个整数型的中间点个数作为输入。
     * 它计算出从初始值到结束值之间的线性插值点，并将结果存储在一个std::vector<Eigen::VectorXd>中返回。
     * 返回的向量包含way_point_num+个元素，其中前一个是初始值，后一个是结束值，中间是way_point_num个插值点。
     * 
     * @param init_value 初始值（Eigen::VectorXd类型）
     * @param end_value 结束值（Eigen::VectorXd类型）
     * @param way_point_num 中间点个数（包含初始点和结束点，int类型）
     * @return 插值点的向量（Eigen::MatrixXd类型）
     */
    static Eigen::MatrixXd linearInterpolation(const Eigen::VectorXd& init_value, 
    const Eigen::VectorXd& end_value, 
    int way_point_num);

    /**
     * @brief 两个double值的线形插值，返回一个列向量储存
     * 
     * @param init_value 初始值double
     * @param end_value 结束值 double
     * @param way_point_num 中间点数量（包含初始点和结束点，int类型）
     * @return Eigen::VectorXd 
     */
    static Eigen::VectorXd linearInterpolation(const double &init_value, 
    const double &end_value, 
    int way_point_num);

    static std::vector<Eigen::MatrixXd> sample_multivariate_normals(Eigen::MatrixXd means,
     Eigen::MatrixXd covariance, 
     int n);

    // MultivariateNormalDistribution(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);

     Tool_wx();
    ~Tool_wx();
    };

    // Multivariate Normal Distribution “多变量正态分布批量采样”
    class Multi_val_dist
    {
    private:
        // 随机变量生成器
        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_;
        Eigen::MatrixXd L_;//下三角矩阵
        Eigen::MatrixXd means_;//平均矩阵

        int n_dimensions_;//数据的维度
        int data_num_;

    public:
        Multi_val_dist(/* args */);
        /**
         * @brief “多变量正态分布批量采样”
         * 
         * @param means Eigen::MatrixXd 每一行代表一个均值向量(一组数据)  
         * @param covariance 共用的协方差矩阵
         */
        Multi_val_dist(Eigen::MatrixXd means,Eigen::MatrixXd covariance);
        /**
         * @brief 根据内部参数生成随机点
         * 
         * @param num 生成样本数量 
         * @return std::vector<Eigen::MatrixXd> 返回num条曲线
         */
        std::vector<Eigen::MatrixXd> sample(int num);

        void init(Eigen::MatrixXd means,Eigen::MatrixXd covariance);

        ~Multi_val_dist();
    };

    
    
    

    
    
}


#endif