#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>


void planWithSimpleSetup(ros::Publisher &path_pub)
{
    // 构造Reeds-Shepp状态空间
    auto space(std::make_shared<ompl::base::ReedsSheppStateSpace>(2.0));

    // 设置状态空间的边界
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(3);
    space->setBounds(bounds);

    // 设置起始和目标状态
    auto start = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
    auto goal = space->allocState()->as<ompl::base::SE2StateSpace::StateType>();
 
    start->setXY(3.0, -3.0);
    start->setYaw(-M_PI_2);
    goal->setXY(2.9, -2.0);
    goal->setYaw(-M_PI_2);

    // 计算Reeds-Shepp路径
    auto rs_path = space->reedsShepp(start, goal);

    // 转换OMPL路径为ROS Path消息
    nav_msgs::Path navPath;
    navPath.header.stamp = ros::Time::now();
    navPath.header.frame_id = "map";
    // 采样Reeds-Shepp路径上的状态
    for (double t = 0.0; t <= 1.0; t += 0.1) {
        auto state = space->allocState()->as<ompl::base::ReedsSheppStateSpace::StateType>();
        space->interpolate(start, goal, t, state);

        // 创建PoseStamped消息
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = "map"; // 或者你的特定参考框架
        poseStamped.pose.position.x = state->getX();
        poseStamped.pose.position.y = state->getY();
        tf2::Quaternion q(tf2::Vector3(0, 0, 1), state->getYaw());
        poseStamped.pose.orientation = tf2::toMsg(q);

        // 将PoseStamped添加到Path消息中
        navPath.poses.push_back(poseStamped);
    }
    // 发布路径
    path_pub.publish(navPath);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reeds_shepp_path_publisher");
    ros::NodeHandle nh;

    // 创建一个Publisher，用于发布path话题
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);

    // 进行路径规划并发布
    planWithSimpleSetup(path_pub);

    ros::spin();
    return 0;
}
