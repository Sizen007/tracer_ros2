/*
 * @Author: Shizeng Zhang 
 * @Date: 2022-01-13 10:35:24 
 * @Last Modified by: Shizeng Zhang
 * @Last Modified time: 2022-01-14 11:54:28
 */
#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "tracer_msgs/msg/tracer_light_cmd.hpp"
#include "tracer_msgs/msg/tracer_status.hpp"
#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

// using std::placeholders::_1;

namespace westonrobot{

class TracerROSMessenger: protected TracerRobot
{
public:
    explicit TracerROSMessenger(std::shared_ptr<rclcpp::Node> ros2_node);
    TracerROSMessenger(TracerRobot *Tracer, std::shared_ptr<rclcpp::Node> ros2_node);

    std::string odom_frame_;
    std::string base_frame_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();
    void PublishUartStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);
    void DetachRobot();

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

private:
    TracerRobot *tracer_;
    std::shared_ptr<rclcpp::Node> ros2_node_;
    // std::shared_ptr<rclcpp::Node> ros2_node_;

    std::mutex twist_mutex_;
    geometry_msgs::msg::Twist current_twist_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<tracer_msgs::msg::TracerStatus>::SharedPtr status_publisher_;
    // not use status_uart_publisher_
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
    rclcpp::Subscription<tracer_msgs::msg::TracerLightCmd>::SharedPtr light_cmd_subscriber_;
    // tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    rclcpp::Time last_time_;
    rclcpp::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void LightCmdCallback(const tracer_msgs::msg::TracerLightCmd::SharedPtr msg);
    void PublishOdometryToROS(double linear, double angular, double dt);

};// namespace wescore
}

#endif /* TRACER_MESSENGER_HPP */