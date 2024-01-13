#include <ros/ros.h>
#include "traj_plan_wx/guass_plan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"

ros::Subscriber sub_amcl_pose;
Eigen::Vector2d pose_now;


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // 这里你可以访问msg->pose.pose来获取机器人的位置和姿态信息
    ROS_INFO_STREAM("Current robot position: (" << msg->pose.pose.position.x 
                << ", " << msg->pose.pose.position.y 
                << ", " << tf::getYaw(msg->pose.pose.orientation) << ")");
    pose_now(0) = msg->pose.pose.position.x;
    pose_now(1) = msg->pose.pose.position.y;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    //     dof = 2                     #自由度
    // delta_t = 0.05              #每一步时间
    // global_num_waypoints = 8    #中间点的数量 
    // global_iterations = 1       #迭代次数
    // global_n_particles = 200    #生成轨迹的数量

    // plan_wx::guass_plan plan(2,200,8,1);
    // plan_wx::MapGuass map(n);

    // tf2_ros::Buffer buffer(ros::Duration(10));
    // tf2_ros::TransformListener tf(buffer);
    // costmap_2d::Costmap2DROS* cost_map_api_ = new costmap_2d::Costmap2DROS("costmap", buffer);
    // sub_amcl_pose = n.subscribe("/amcl_pose", 10, poseCallback);

    plan_wx::guass_plan plan(2,200,8,1);
    Eigen::Vector2d pose_end = {5.48,-4.59};

    // sleep(5);
    ros::Publisher path_pub_good= n.advertise<nav_msgs::Path>("path_topic_good", 10);

    for (int i = 0; i < 500; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();
        plan.global_mpc_planner(pose_now,pose_end,1);
        auto end = std::chrono::high_resolution_clock::now();

            // 计算并输出运行时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << "The function took " << duration.count() << " milliseconds to run.\n";
        nav_msgs::Path path_mean = plan.get_path_mean();
        path_pub_good.publish(path_mean);

    }
    






    ros::Publisher path_pub_all = n.advertise<nav_msgs::Path>("path_topic_all", 10);
    std::vector<nav_msgs::Path> plan_all = plan.get_path_all();

    ros::Duration time(0.1);

    for(int i = 0; i < plan_all.size(); i++)
    {
        path_pub_all.publish(plan_all[i]);

        time.sleep();
    }


    

    ;

    ros::spin();
    // delete cost_map_api_;
    
    
    return 0;
}
