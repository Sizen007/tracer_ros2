/*
 * @Author: Shizeng Zhang 
 * @Date: 2022-01-13 10:19:19 
 * @Last Modified by: Shizeng Zhang
 * @Last Modified time: 2022-01-13 10:34:45
 */

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tracer_base/tracer_messenger.hpp"

#include <ugv_sdk/mobile_robot/tracer_robot.hpp>

using namespace westonrobot;

std::shared_ptr<TracerRobot> robot;

int main(int argc, char **argv){
    // setup ROS2 node
    rclcpp::init(argc, argv);   
    std::shared_ptr<rclcpp::Node> node;
    node = std::make_shared<rclcpp::Node>("tracer_base");
    
    RCLCPP_INFO(node->get_logger(), "tracer_base node start...");

    robot = std::make_shared<TracerRobot>(); 
    TracerROSMessenger messenger(robot.get(), node);

    // fetch parameters before connecting to robot
    std::string port_name;
    rclcpp::Parameter parameter;

    port_name = "can0";
    messenger.odom_frame_ = "odom";
    messenger.base_frame_ = "base_link";

    // connect to robot and setup ROS subscription
    if(port_name.find("can") != port_name.npos){
        try{
            robot->Connect(port_name);
        }catch(std::exception error){
            RCLCPP_ERROR(node->get_logger(), "please bringup up can or make sure can port exist");
            rclcpp::shutdown();
        }
    }else{
        RCLCPP_INFO(node->get_logger(), "Using UART to talk with the robot");
    }

    messenger.SetupSubscription();

    rclcpp::Rate rate(std::chrono::milliseconds(20));

    while (rclcpp::ok()) {
        
        rclcpp::spin_some(node);
        messenger.PublishStateToROS();
        // robot->SetMotionCommand(0.1,0.38);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}







