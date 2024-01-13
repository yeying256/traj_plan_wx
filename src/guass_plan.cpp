#include "traj_plan_wx/guass_plan.h"

namespace plan_wx{

    guass_plan::guass_plan(/* args */)
    {
    }
    
    guass_plan::~guass_plan()
    {
    }

    guass_plan::guass_plan(int dof,int global_num_particles,int global_num_waypoints, int global_iterations)
    {
        //赋值给成员变量
        this->guass_base_param_.DOF = dof;
        guass_base_param_.global_num_particles=global_num_particles;
        guass_base_param_.global_num_waypoints=global_num_waypoints;
        guass_base_param_.global_iterations = global_iterations;

        target_pos_.resize(dof);
        start_pos_.resize(dof);

        // 平均路径
        guass_data_.path_mean_.resize(global_num_waypoints,dof);
        this->guass_data_.path_generate_.resize(global_num_particles);

    }

    std::vector<Eigen::VectorXd> guass_plan::global_mpc_planner(Eigen::VectorXd start,
    Eigen::VectorXd end,
    int iterations)
    {
        std::vector<Eigen::VectorXd> path;
        // 检查输入是否符合
        if (start.size() != end.size() || start.size()!=this->guass_base_param_.DOF)
        {
            std::cout<<color_red<<"dof err or start and end size error"<<color_default<<std::endl;
            return path;
        }
        this->start_pos_ = start;

        for (int i = 0; i < iterations; i++)
        {
            // 生成均值轨迹和200个采样轨迹
            get_initial_mean_and_samples(end);

            // 优化处理，选择最合适的曲线

            optimize_path(guass_data_);
            
            // 根据不同的策略缩放sigma scale_value是一个比率double型
            this->evolutionary_strategy(map_.get_min_cost());
            std::cout<<"map_.get_min_cost() = "<<map_.get_min_cost()<<std::endl;
            
        }
        

        return path;
        
    }

    void guass_plan::get_initial_mean_and_samples(Eigen::VectorXd target)
    {

        double distance = (target-this->target_pos_).norm();//求解二范数
        this->target_pos_ = target;
        // 如果目标改变或者还没初始化平均路径
        if (distance >0.01 || this->guass_base_param_.init_meanPath_flag == false)
        {
            // 进行高斯先验分布
            global_gp_prior();
            // 目标改变标志位
            this->guass_base_param_.goal_change_index = true;

            // 每一行是一个点 8*dof
            this->guass_data_.path_mean_ = Tool_wx::linearInterpolation(this->start_pos_,target_pos_,this->guass_base_param_.global_num_waypoints);
            if (guass_base_param_.init_meanPath_flag == false)
            {
                guass_base_param_.init_meanPath_flag = true;
            }
        }else
        {
            // 不重新采样
            ;
        }

        if(this->guass_base_param_.prior_update_flag == true)
        {
            // 生成global_num_particles条轨迹
            guass_data_.normal_batch_samples_ = this->gp_prior_.sample(guass_base_param_.global_num_particles);
        }else{;} //否则就什么也不做
        // 生成随机轨迹
        for (int i = 0; i < this->guass_base_param_.global_num_particles; i++)
        {
            // 缩放系数* 高斯生成的那个下三角矩阵 * 每条轨迹样本点
            this->guass_data_.path_generate_[i] = this->guass_data_.path_mean_ 
            + this->guass_kernelParam_.sigma_scale 
                * guass_data_.L_posterior_ 
                * guass_data_.normal_batch_samples_[i];
        }
    }

    void guass_plan::global_gp_prior()
    {
        Eigen::Vector2d t_support;
        t_support<<0.0,1.0;
        
        // 创建高斯求解协方差矩阵 01 和01 的协方差矩阵
        Eigen::MatrixXd kernel_support = RBF_kernel(t_support,t_support);
        std::cout<<"kernel_support = "<<kernel_support<<std::endl;
        // LLT分解，用于求解线性方程组 2x2
        Eigen::LLT<Eigen::MatrixXd> scale_L(kernel_support + 1e-10 * Eigen::Matrix2d::Identity());
        Eigen::MatrixXd L = scale_L.matrixL();
        std::cout<<"scale_L = "<<L<<std::endl;

        //
        Eigen::VectorXd t_test = Tool_wx::linearInterpolation(0.0,1.0,
        this->guass_base_param_.global_num_waypoints);
        
        Eigen::MatrixXd k_support_test(2,guass_base_param_.global_num_waypoints);

        // 2x8
        k_support_test = RBF_kernel(t_support,t_test);
        std::cout<<"k_support_test = "<<k_support_test<<std::endl;

        // compute the variance at our test points. 8x8
        // Eigen::MatrixXd kernel_test = RBF_kernel(t_test,t_test);

        // 2X8
        Eigen::MatrixXd scale_Lk= scale_L.matrixL().solve(k_support_test);
        std::cout<<"scale_Lk = "<<scale_Lk<<std::endl;

        
        // 8x8 t_test的高斯核
        Eigen::MatrixXd kernel_test = RBF_kernel(t_test,t_test);
        std::cout<<"kernel_test = "<<kernel_test<<std::endl;
        std::cout<<"guass_base_param_.global_num_waypoints = "<<guass_base_param_.global_num_waypoints<<std::endl;
        //  draw samples from the posterior at our test points.
        //  分解矩阵正定矩阵A分解为两个三角矩阵的乘积 posterior：在观测到数据之后对某个参数或模型的概率分布 
        // L_posterior = torch.linalg.cholesky(kernel_test + 1e-6*torch.eye(self.num_test_points) - torch.matmul(scale_Lk.transpose(0, 1), scale_Lk))
        Eigen::LLT<Eigen::MatrixXd> L_posterior(kernel_test 
        + 1e-10* Eigen::MatrixXd::Identity(guass_base_param_.global_num_waypoints,guass_base_param_.global_num_waypoints)
                                                - scale_Lk.transpose()*scale_Lk);  
        // self.L_posterior = L_posterior.to(**self.tensor_args)
        guass_data_.L_posterior_ = L_posterior.matrixL();//下三角
        Eigen::MatrixXd init_action = Eigen::MatrixXd::Zero(guass_base_param_.global_num_waypoints,2);

        // 初始化随机过程
        gp_prior_.init(init_action,Eigen::MatrixXd::Identity(this->guass_base_param_.DOF,this->guass_base_param_.DOF));
        // 是否需要重新进行采样
        guass_base_param_.prior_update_flag = true;
    }

    Eigen::MatrixXd guass_plan::RBF_kernel(Eigen::VectorXd a,Eigen::VectorXd b)
    {
        // 写一个Eigen库的RBF_kernel算法 输入为Eigen::VectorXd a,Eigen::VectorXd b，输出为Eigen::MatrixXd
        // if (a.size() != b.size()) {
        // throw std::invalid_argument("Input vectors must have the same size.");
        // }
        int row = a.size();  // 向量的维度
        int cols = b.size();
        Eigen::MatrixXd kernel(row, cols);  // 初始化结果矩阵
        for (int i = 0; i < row; ++i) //遍历行 
        {
            for (int j = 0; j < cols; ++j) //遍历列
            {
                kernel(i,j) = pow(a(i),2)+pow(b(j),2);//算一个平方
            }
        }
        kernel = kernel - 2.0* a*b.transpose();
        // eigen 库计算每个元素的exp 矩阵数乘呢
        kernel = -0.5 * (1/guass_kernelParam_.l)*kernel;
        kernel = guass_kernelParam_.sigma * (kernel.array()).exp().matrix();
        // 需要先把它转化成向量，然后进行exp操作，再转化回来。因为矩阵直接进行exp操作那就是e的矩阵次幂。
        return kernel;
    }


    void guass_plan::optimize_time_normalized()
    {
        // if();
        // ;
    }

    void guass_plan::optimize_path(struct Guass_data &data)
    {
        //  取出最优轨迹的index ,并且更新代价 
        int index = map_.cost_cal(data.path_generate_);

        //  
        this->guass_data_.path_mean_ = data.path_generate_[index];
        ;
    }

    double guass_plan::evolutionary_strategy(double cost)
    {

        // print("obstacle cost", obs_cost)
        double scale = 1.0;
        if (cost<1000 &&  guass_kernelParam_.sigma>guass_kernelParam_.sigma_min)
        {
            scale = 0.8;
            this->guass_kernelParam_.sigma *=scale; 
        }
        else if (cost > 0 && guass_kernelParam_.sigma<guass_kernelParam_.sigma_max)
        {
            scale = 1.8;
            guass_kernelParam_.sigma *= scale;
        }else
        {
            scale = 1.0;
        }
            this->guass_kernelParam_.sigma_scale *=scale;
            return scale;
        ;
    }

    nav_msgs::Path guass_plan::get_path_mean()
    {
        nav_msgs::Path path;
        guass_data_.path_mean_;
        geometry_msgs::PoseStamped point;
        path.header.frame_id = "/map";
        for (int i = 0; i < guass_data_.path_mean_.rows(); i++)
        {
            point.pose.position.x = guass_data_.path_mean_(i,0);
            point.pose.position.y = guass_data_.path_mean_(i,1);

            path.poses.push_back(point);
        }
        return path;
    }

    
    std::vector<nav_msgs::Path> guass_plan::get_path_all()
    {
        std::vector<nav_msgs::Path> path_all;
        for (int i = 0; i < this->guass_data_.path_generate_.size(); i++)
        {
            nav_msgs::Path path;
            path.header.frame_id = "/map";
            geometry_msgs::PoseStamped point;
            for (size_t j = 0; j < guass_data_.path_generate_[i].rows(); j++)
            {
                point.pose.position.x = guass_data_.path_generate_[i](j,0);
                point.pose.position.y = guass_data_.path_generate_[i](j,1);
                path.poses.push_back(point);
            }
            path_all.push_back(path);
        }
        return path_all;

    }









    

    
}