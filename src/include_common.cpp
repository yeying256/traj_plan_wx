#include "traj_plan_wx/include_common.h"


namespace plan_wx{
Tool_wx::Tool_wx()
{

}


Eigen::MatrixXd Tool_wx::linearInterpolation(const Eigen::VectorXd& init_value, const Eigen::VectorXd& end_value, int way_point_num) {
    if (init_value.size() != end_value.size()) {
        throw std::invalid_argument("初始值和结束值的大小必须相同。");
    }

    Eigen::MatrixXd result(way_point_num,init_value.size());

    // 计算线性插值的步长
    Eigen::VectorXd step_size = (end_value-init_value)/(way_point_num-1);
    
    // 将初始值赋值进去
    result.row(0) = init_value.transpose();

    // 对每个中间点进行线性插值
    for (int i = 0; i < way_point_num-2; i++) {
        result.row(i+1) = result.row(i) + step_size.transpose();
    }

    // 将初始值和结束值添加到结果向量中
    result.row(way_point_num - 1) = end_value.transpose();

    return result;
}


    Eigen::VectorXd Tool_wx::linearInterpolation(const double &init_value, 
    const double &end_value, 
    int way_point_num)
    {
        Eigen::VectorXd result(way_point_num);
        double step_size = (end_value-init_value)/(way_point_num-1);
        result[0] = init_value;
        for (int i = 0; i < way_point_num-2; i++) 
        {
        result(i+1) = result(i) + step_size;
        }
        result(way_point_num - 1) = end_value;
        return result;
    }



    // 生成n组服从多变量正态分布的随机向量集合，每组共享同一协方差矩阵
    std::vector<Eigen::MatrixXd> Tool_wx::sample_multivariate_normals(Eigen::MatrixXd means,
    Eigen::MatrixXd covariance, 
    int n)
    {
        //自由度
        int n_dimensions = means.cols();

        std::default_random_engine generator;
        generator.seed(std::random_device{}()); // 使用随机设备生成种子
        std::normal_distribution<double> distribution(0.0, 1.0);

        Eigen::MatrixXd L = covariance.llt().matrixL(); // 计算协方差矩阵的Cholesky下三角分解

        std::vector<Eigen::MatrixXd> samples;
        for (int i = 0; i < n; ++i) {
            Eigen::MatrixXd group_samples(n_dimensions, means.rows());
            for (int j = 0; j < means.rows(); ++j) {
                Eigen::VectorXd epsilon(n_dimensions);
                for (int k = 0; k < n_dimensions; ++k)
                    epsilon(k) = distribution(generator);

                group_samples.col(j) = means.row(j) + L * epsilon;
            }
            samples.push_back(group_samples);
        }

        return samples;
    }

    Multi_val_dist::Multi_val_dist(Eigen::MatrixXd means,Eigen::MatrixXd covariance) : distribution_(0,1)
    {
        generator_.seed(std::random_device{}());
        L_ = covariance.llt().matrixL();
        this->means_ = means;
        // 数据的维度
        n_dimensions_ = means.cols();
        // 生成数据的样本大小
        data_num_ = means.rows();
    }

    void Multi_val_dist::init(Eigen::MatrixXd means,Eigen::MatrixXd covariance)
    {        
    generator_.seed(std::random_device{}());
    L_ = covariance.llt().matrixL();
    this->means_ = means;
    // 数据的维度
    n_dimensions_ = means.cols();
    // 生成数据的样本大小
    data_num_ = means.rows();
    }

    Multi_val_dist::Multi_val_dist(/* args */) : distribution_(0,1)
    {
    }
    
    Multi_val_dist::~Multi_val_dist()
    {
    }

    std::vector<Eigen::MatrixXd> Multi_val_dist::sample(int num)
    {
        // num 个样本数量
        std::vector<Eigen::MatrixXd> samples;
        for (int i = 0; i < num; ++i) 
        {//样本数量循环
            Eigen::MatrixXd group_samples(data_num_,n_dimensions_);
            // 行 ：每一个样本点 列：自由度数
            for (int j = 0; j < data_num_; ++j) {
                // 行
                Eigen::VectorXd epsilon(n_dimensions_);
                for (int k = 0; k < n_dimensions_; ++k) //生成两个随机点
                {epsilon(k) = distribution_(generator_);
                }
                group_samples.row(j) = means_.row(j) + (L_ * epsilon).transpose();
            }
            samples.push_back(group_samples);
        }
        return samples;
    }



}