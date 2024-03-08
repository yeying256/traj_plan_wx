#include <ros/ros.h>
#include "traj_plan_wx/guass_plan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"
#include "traj_plan_wx/controller.h"
#include <nav_msgs/Odometry.h>
#include "traj_plan_wx/plan.h"

#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner s(5);
    s.start();
    plan_wx::plan plan(nh);


    plan.run();
    
    ros::spin();
    s.stop();
    ros::shutdown();
    return 0;
}
