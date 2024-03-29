#include "traj_plan_wx/map_guass.h"
#include <costmap_2d/layered_costmap.h>


namespace plan_wx{
    MapGuass::MapGuass(ros::NodeHandle& nh): nh_(nh)
    {
        // tf2_ros::Buffer buffer(ros::Duration(10));
        // tf2_ros::TransformListener tf(buffer);
        // cost_map_api_ = new costmap_2d::Costmap2DROS("costmap", buffer);
        buffer_ =new tf2_ros::Buffer(ros::Duration(10));
        tf_ = new tf2_ros::TransformListener(*buffer_);
        cost_map_api_ = new costmap_2d::Costmap2DROS("costmap", *buffer_);
    }
    MapGuass::MapGuass(/* args */)
    {
        buffer_ =new tf2_ros::Buffer(ros::Duration(10));
        tf_ = new tf2_ros::TransformListener(*buffer_);
        cost_map_api_ = new costmap_2d::Costmap2DROS("costmap", *buffer_);
        cost_map_ = cost_map_api_->getCostmap();

        
        // cost_map_->
    }
    
    MapGuass::~MapGuass()
    {
        delete cost_map_api_;
        delete buffer_;
    }

    int MapGuass::pose2dToGridIndex(const geometry_msgs::Pose2D& pose,
                            const nav_msgs::OccupancyGrid& grid)
    {
        // 获取地图元数据
        double resolution = grid.info.resolution; //分辨率 m/格
        int width = grid.info.width;
        int height = grid.info.height;
        geometry_msgs::Point origin = grid.info.origin.position;

        // 将 Pose2D 转换为 OccupancyGrid 的坐标系统
        // 其中，(x, y)坐标定义了地图左下角在全局坐标系中的位置，即地图栅格的起始点或零点。
        // 当需要将真实世界的坐标转换成栅格地图上的索引时，会用到这个原点信息。
        // 通过将实际坐标减去原点坐标并除以分辨率（resolution），可以得到对应的栅格索引。
        double x_in_grid = (pose.x - origin.x) / resolution;
        double y_in_grid = (pose.y - origin.y) / resolution;

        // 计算并确保坐标在网格范围内
        // std::min(int(x_in_grid + 0.5), width - 1)不出最大值
        // std::max(0,xxx)确保不出最小值
        int cell_x = std::max(0, std::min(int(x_in_grid + 0.5), width - 1));
        int cell_y = std::max(0, std::min(int(y_in_grid + 0.5), height - 1));

        // 返回索引（注意：大多数情况下，ROS中的图像/栅格都是行优先存储，所以y轴在前）
        return cell_y * width + cell_x;
    }


    nav_msgs::OccupancyGrid MapGuass::addLaser2GlobalMaps(const sensor_msgs::LaserScan& laser_scan, 
                                                            const geometry_msgs::PoseStamped& robot_pose, 
                                                            nav_msgs::OccupancyGrid& global_map)
    {

        // 创建一个新的全局地图以存储合并后的结果
        nav_msgs::OccupancyGrid merged_map = global_map;

        // 获取地图大小
        int width = global_map.info.width;
        int height = global_map.info.height;


        std::vector<int> map_index_laser = LaserScanToMapIndex(laser_scan,robot_pose,global_map);

        for (int i = 0; i < map_index_laser.size(); i++)
        {
            merged_map.data[map_index_laser[i]] = 100;
        }

        return merged_map;
    }


    std::vector<int> MapGuass::LaserScanToMapIndex(
    const sensor_msgs::LaserScan& laser_scan,
    const geometry_msgs::PoseStamped& robot_pose,
    const nav_msgs::OccupancyGrid& global_map)
    {
        std::vector<int> indices;

        // 将雷达扫描的数据进行储存
        double angle_min = laser_scan.angle_min;
        double angle_increment = laser_scan.angle_increment;
        int map_width = global_map.info.width;
        int map_height = global_map.info.height;
        double map_resolution = global_map.info.resolution;
        double origin_x = global_map.info.origin.position.x;
        double origin_y = global_map.info.origin.position.y;

        // 获取机器人的位置和方向
        tf2::Quaternion q(robot_pose.pose.orientation.x, 
                        robot_pose.pose.orientation.y, 
                        robot_pose.pose.orientation.z, 
                        robot_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 获取机器人的yaw角

        double laser_angle = angle_min;//角度

        for (size_t i = 0; i < laser_scan.ranges.size(); ++i) {
            // 转换雷达数据到全局坐标系下的坐标
            // double laser_angle = angle_min + i * angle_increment;//角度
            double range = laser_scan.ranges[i];//距离
            if (range <= laser_scan.range_max && range >= laser_scan.range_min) 
            {
                double x = range * cos(laser_angle + yaw) + robot_pose.pose.position.x;
                double y = range * sin(laser_angle + yaw) + robot_pose.pose.position.y;

                // 将雷达测量点坐标转换为地图一维索引
                int map_x = static_cast<int>((x - origin_x) / map_resolution);
                int map_y = static_cast<int>((y - origin_y) / map_resolution);

                // 检查索引是否在地图范围内并转换为一维索引
                if (map_x >= 0 && map_x < map_width && map_y >= 0 && map_y < map_height) {
                    int map_index = map_y * map_width + map_x;
                    indices.push_back(map_index);
                }
            }
            // 每次循环结束加角度
            laser_angle+=angle_increment;
        }

        return indices;
    }


    uint16_t MapGuass::cost_cal(const std::vector<Eigen::MatrixXd>& line,
                    std::vector<struct traj_str> & traj_all)
    {
        uint16_t min_cost_index = 0;
        double resulote = this->cost_map_->getResolution();//分辨率，单位：m每格
        if (traj_all.size() != line.size()) //如果不等于
        {
            traj_all.resize(line.size());
        }
        for (int i = 0; i < line.size(); i++)//遍历每一条生成的线
        {
            bool flag_out_ = false;
            struct traj_str traj_only;  //每一条曲线的信息
            traj_only.cost_all = 0.0;
            for (int j = 0; j < line[i].rows() - 1; j++) //遍历每一个点 减去1是因为最后一个点
            {
                double distance = sqrt(
                                        pow(line[i](j+1,0) - line[i](j,0),2)
                                        +pow(line[i](j+1,1) - line[i](j,1),2)
                                        );
                if (distance>10.0)
                {
                    flag_out_ = true;
                    break;
                }
                
                // flag_out_ == true;
                // 求 每两点之间的斜率
                // double kl = (line[i](j+1,1)-line[i](j,1) ) / (line[i](j+1,0)-line[i](j,0));
                // double dy = kl*resulote;    //计算dy
                double x0 = line[i](j,0);   //
                double y0 = line[i](j,1);   //
                
                // 计算yaw值
                // double yaw = atan(kl);
                // tf2::Quaternion q4;
                // q4.setRPY(0,0,yaw);

                double x = line[i](j,0);
                double y = line[i](j,1);
                // 计算加的次数
                int count_x = abs(line[i](j+1,0) - line[i](j,0))/resulote -1;
                int count_y = abs(line[i](j+1,1) - line[i](j,1))/resulote -1;

                int count = 0;
                count = count_x>=count_y ?   count_x: count_y;
                // std::cout<<"count = "<<count<<std::endl;

                    double dx_1 = 0;
                    double dy_1 = 0;
                if (count>0)
                {
                    dx_1 = (line[i](j+1,0) - line[i](j,0))/(double)count;
                    dy_1 = (line[i](j+1,1) - line[i](j,1))/(double)count;
                }
                


                

                for (int k = 0; k < count; k++)
                {
                    Point_robot_state point;
                    point.x = x0 + dx_1*k;    //设置每个点的坐标值
                    // point.y = y0 + kl * (x-x0);
                    point.y = y0 + dy_1*k;

                    
                    // 设置yaw值
                    // point.yaw = yaw;
                    //设置四元数
                    // point.q4 = q4;
                    unsigned int x_map,y_map;   //地图int

                    this->cost_map_->worldToMap(point.x,point.y,x_map,y_map);

                    int x_max_map = cost_map_->getSizeInCellsX();
                    int y_max_map = cost_map_->getSizeInCellsY();
                    x_map = x_map>x_max_map?x_max_map : x_map;
                    y_map = y_map>y_max_map?y_max_map : y_map;

                    // std::cout<<"x_map = "<<x_map<<"cost_map_->getSizeInCellsX =" <<cost_map_->getSizeInCellsX()<<std::endl;
                    // std::cout<<"y_map="<<y_map<<"cost_map_->getSizeInCellsY =" <<cost_map_->getSizeInCellsY()<<std::endl;
                    if (x_map>x_max_map || y_map>y_max_map)
                    {
                        flag_out_ = true;
                        break;
                    }
                    else
                    {
                        point.cost_point = (double)cost_map_->getCost(x_map,y_map);
                        traj_only.cost_all+=point.cost_point;

                    }
                    


                    // if (cost_map_->getCost(x_map,y_map) == 254)
                    // {
                    // std::cout<<"cost_map_->getCost(x_map,y_map) = "<<(int)cost_map_->getCost(x_map,y_map)<<std::endl;
                    // std::cout<<"point.cost_point = "<<point.cost_point<<std::endl;
                    // }


                    point.dx = dx_1;
                    point.dy = dy_1;



                    // 长度权重
                    

                }
                traj_only.cost_all +=distance;

                
            }
            if (flag_out_ == true)
            {
                traj_only.cost_all=10000000;
                break;
            }
            
            // 轨迹长度代价
            // traj_only.cost_all+=traj_only.point_stamp.size()*0.1;
        
            // 赋值
            traj_all[i] = traj_only;
            if (traj_all[i].cost_all< traj_all[min_cost_index].cost_all)
            {
                min_cost_index = i;
            }
        }
        //将最小代价赋值并返回索引
        min_cost_ = traj_all[min_cost_index].cost_all;
        // for (size_t i = 0; i < traj_all.size(); i++)
        // {
        //     std::cout<<"cost_traj"<<i<<" = "<< traj_all[i].cost_all<<std::endl;
        // }
        // std::cout<<"min_cost_ "<<min_cost_index<<"="<<traj_all[min_cost_index].cost_all<<std::endl;
        
        return min_cost_index;
        
    }



    uint16_t MapGuass::cost_cal(const std::vector<Eigen::MatrixXd>& line)
    {
        return cost_cal(line,this->traj_all_);
    }

    double MapGuass::get_min_cost()
    {
        return min_cost_;
    }

    double MapGuass::get_cost(uint16_t index)
    {
        if (traj_all_.size()<=index)
        {
            std::cout<<"get_cost(uint16_t index) error,traj_all_.size()<=index"<<std::endl;
            return 0.0;
        }else{
            return traj_all_[index].cost_all;
        }
    }






}