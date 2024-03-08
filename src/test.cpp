#include <ros/ros.h>
#include "traj_plan_wx/guass_plan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "traj_plan_wx/controller.h"
#include <nav_msgs/Odometry.h>

ros::Subscriber sub_amcl_pose;
Eigen::Vector2d pose_now;
Eigen::Vector2d vel_now;

geometry_msgs::Pose pose_now_geometry_msgs;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // 这里你可以访问msg->pose.pose来获取机器人的位置和姿态信息
    // ROS_INFO_STREAM("Current robot position: (" << msg->pose.pose.position.x 
    //             << ", " << msg->pose.pose.position.y 
    //             << ", " << tf::getYaw(msg->pose.pose.orientation) << ")");
    pose_now(0) = msg->pose.pose.position.x;
    pose_now(1) = msg->pose.pose.position.y;
    pose_now_geometry_msgs = msg->pose.pose;

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)

{

    // 解析Odometry消息内容

    geometry_msgs::Pose pose = odom_msg->pose.pose;

    geometry_msgs::Point position = pose.position;

    geometry_msgs::Quaternion orientation = pose.orientation;


    geometry_msgs::Twist twist = odom_msg->twist.twist;

    geometry_msgs::Vector3 linear_velocity = twist.linear;

    geometry_msgs::Vector3 angular_velocity = twist.angular;
    vel_now(0) = twist.linear.x;
    vel_now(1) = twist.linear.y;



    // ROS_INFO_STREAM("Received Odom data:");

    // ROS_INFO_STREAM("Position: (" << position.x << ", " << position.y << ", " << position.z << ")");

    // // 注意：将四元数转换为欧拉角或航向角通常更为直观，这里仅作演示未做转换

    // ROS_INFO_STREAM("Orientation: (" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ")");

    // ROS_INFO_STREAM("Linear Velocity: (" << linear_velocity.x << ", " << linear_velocity.y << ", " << linear_velocity.z << ")");

    // ROS_INFO_STREAM("Angular Velocity: (" << angular_velocity.x << ", " << angular_velocity.y << ", " << angular_velocity.z << ")");


    // 这里可以进一步处理odom数据，例如记录、绘图、控制等

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::AsyncSpinner s(3);
    s.start();

    pose_now_geometry_msgs.orientation.w = 1;
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
    sub_amcl_pose = n.subscribe("/amcl_pose", 10, poseCallback);


  // 创建一个publisher，topic名为/cmd_vel，消息类型为geometry_msgs/Twist

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);

    //监听当前机器人位姿
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 获取目标坐标系相对于源坐标系的变换
    geometry_msgs::TransformStamped transformStamped;

    plan_wx::guass_plan plan(2,300,12,1);

    
    Eigen::Vector2d pose_end = {5.35,-3.4};
    
    geometry_msgs::Pose pos_d;
    pos_d.position.x = pose_end(0);
    pos_d.position.y = pose_end(1);
    pos_d.orientation.w = 1;
    pos_d.orientation.x = 0;
    pos_d.orientation.y = 0;
    pos_d.orientation.z = 0;


    

    // sleep(5);
    ros::Publisher path_pub_good= n.advertise<nav_msgs::Path>("path_topic_good", 10);

    plan_wx::controller cmd;

    geometry_msgs::Twist cmd_vel;

    ros::Publisher path_pub_all = n.advertise<nav_msgs::Path>("path_topic_all", 10);
    ros::Publisher path_pub_cubic = n.advertise<nav_msgs::Path>("path_cubic", 10);
    
    for (int i = 0; i < 50000; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();
        plan.global_mpc_planner(pose_now,pose_end,1);
        auto end = std::chrono::high_resolution_clock::now();

            // 计算并输出运行时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // std::cout << "The function took " << duration.count() << " milliseconds to run.\n";
        nav_msgs::Path path_mean = plan.get_path_mean();
        path_pub_good.publish(path_mean);

        
        transformStamped = tf_buffer.lookupTransform(
            "map",  // 源坐标系
            "base_footprint",  // 目标坐标系
            ros::Time(0), // 查询当前时间或指定时间戳的变换
            ros::Duration(0.001)); // 超时等待时间，这里设置为1秒

            // transformStamped.transform
            transformStamped.transform.rotation;
            geometry_msgs::Pose pose_now_tf;
            pose_now_tf.orientation = transformStamped.transform.rotation;
            pose_now_tf.position.x = transformStamped.transform.translation.x;
            pose_now_tf.position.y = transformStamped.transform.translation.y;

        cmd_vel = cmd.line2cmd_vel(path_mean,pos_d,pose_now_tf,vel_now,plan.map_.get_min_cost());
        path_pub_cubic.publish(cmd.path_cubic_);

        // 获取控制优化后的新点
        Eigen::MatrixXd new_point_cubic = cmd.get_new_point();
        // std::cout<<"new_point_cubic = "<<new_point_cubic<<std::endl;

        plan.updata_opt_path(new_point_cubic);
        
        // 发布cmd_vel
        cmd_vel_pub.publish(cmd_vel);

        if (!ros::ok() )
        {
            break;
        }
        

    }
    
    





    std::vector<nav_msgs::Path> plan_all = plan.get_path_all();

    ros::Duration time(0.1);

    for(int i = 0; i < plan_all.size(); i++)
    {
        path_pub_all.publish(plan_all[i]);

        time.sleep();
    }


    

    ;

    ros::spin();

    s.stop();
    // delete cost_map_api_;
    
    
    return 0;
}
