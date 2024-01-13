#ifndef MAP_GUASS_H
#define MAP_GUASS_H

#include "traj_plan_wx/include_common.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/LaserScan.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/static_layer.h"
#include "costmap_2d/obstacle_layer.h"
#include "costmap_2d/inflation_layer.h"
#include "ros/ros.h"

namespace plan_wx{

    class MapGuass
    {
    private:
        nav_msgs::OccupancyGrid map_;
        ros::NodeHandle nh_;
        costmap_2d::Costmap2DROS* cost_map_api_;

        tf2_ros::Buffer* buffer_;
        tf2_ros::TransformListener* tf_;


        /* data */
    public:
        /**
         * @brief pos2d 到地图的索引值
         * 
         * @param pose 机器人位姿
         * @param grid 地图
         * @return int grid数据的索引值
         */
        std::vector<struct traj_str> traj_all_;
        // 最小代价
        double min_cost_;
        costmap_2d::Costmap2D* cost_map_;


        static int pose2dToGridIndex(const geometry_msgs::Pose2D& pose,
                            const nav_msgs::OccupancyGrid& grid);
        
        MapGuass(/* args */);
        MapGuass(ros::NodeHandle& nh);
        ~MapGuass();

        /**
         * @brief 将雷达数据加入到全局地图中
         * 
         * @param laser_scan 雷达数据
         * @param robot_pose 机器人坐标
         * @param global_map 全局地图
         * @return nav_msgs::OccupancyGrid 合成后的地图 
         */
        nav_msgs::OccupancyGrid addLaser2GlobalMaps(const sensor_msgs::LaserScan& laser_scan, 
        const geometry_msgs::PoseStamped& robot_pose, 
        nav_msgs::OccupancyGrid& global_map);

        /**
         * @brief 将每个雷达数据点转化为全局地图的索引
         * 
         * @param laser_scan 雷达数据信息 
         * @param robot_pose 机器人位姿
         * @param global_map 全局地图
         * @return std::vector<int> 每个数据点的索引
         */
        static std::vector<int> LaserScanToMapIndex(
        const sensor_msgs::LaserScan& laser_scan,
        const geometry_msgs::PoseStamped& robot_pose,
        const nav_msgs::OccupancyGrid& global_map);

        static std::vector<Eigen::Vector2i> map_interLine(int start_x,int start_y,int end_x,int end_y);

        /**
         * @brief 从线段到计算所有代价
         * 
         * @param line 所有的线段在世界坐标系下的表达
         * @param traj_all 传出参数所有的轨迹在代价地图下的表达，以及所有的代价，最外侧的std::vector是多条轨迹。
         * @return uint16_t 返回最小代价的索引
         */
        uint16_t cost_cal(const std::vector<Eigen::MatrixXd>& line,
                    std::vector<struct traj_str> & traj_all);
        /**
         * @brief 从线段到计算所有代价,内部参数进行处理
         * 
         * @param line 所有的线段在世界坐标系下的表达
         * @return uint16_t 返回最小代价的索引
         */
        uint16_t cost_cal(const std::vector<Eigen::MatrixXd>& line);

        /**
         * @brief Get the min cost object返回最小的代价
         * 
         * @return double 
         */
        double get_min_cost();

        /**
         * @brief Get the cost object获取第index轨迹的cost
         * 
         * @param index 索引
         * @return double 返回的cost
         */
        double get_cost(uint16_t index);


    };
    

    
    ;
}

#endif