/*
 * @Author: Shizeng Zhang 
 * @Date: 2022-01-13 11:41:20 
 * @Last Modified by: Shizeng Zhang
 * @Last Modified time: 2022-01-13 14:51:19
 */

#include "tracer_base/tracer_messenger.hpp"

namespace westonrobot
{ 
TracerROSMessenger::TracerROSMessenger(std::shared_ptr<rclcpp::Node> ros2_node) {ros2_node_ = ros2_node;}
TracerROSMessenger::TracerROSMessenger(TracerRobot *tracer, std::shared_ptr<rclcpp::Node> ros2_node) : tracer_(tracer) 
{
    ros2_node_ = ros2_node;
}

void TracerROSMessenger::DetachRobot(){}

void TracerROSMessenger::SetupSubscription()
{
    // odometry publisher
    // TODO check qos deepth data for every publisher and subscriber
    odom_publisher_ = ros2_node_->create_publisher<nav_msgs::msg::Odometry> ("odom", 50); // QoS(KeepLast(50 history deepth))
    status_publisher_ = ros2_node_->create_publisher<tracer_msgs::msg::TracerStatus> ("tracer_status", 10);
    
    // cmd subscriber
    motion_cmd_subscriber_ = ros2_node_->create_subscription<geometry_msgs::msg::Twist> ("cmd_vel", 5, std::bind(&TracerROSMessenger::TwistCmdCallback, this, std::placeholders::_1)); //不启用平滑包则订阅“cmd_vel”
    light_cmd_subscriber_ = ros2_node_->create_subscription<tracer_msgs::msg::TracerLightCmd> ("tracer_light_control", 5, std::bind(&TracerROSMessenger::LightCmdCallback, this, std::placeholders::_1));
    
}

void TracerROSMessenger::TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    RCLCPP_INFO(ros2_node_->get_logger(), "get cmd %lf %lf", msg->linear.x, msg->linear.z);
    
    if (!simulated_robot_)
    {
        tracer_->SetMotionCommand(msg->linear.x, msg->angular.z);
    }
    else
    {
        std::lock_guard<std::mutex> guard(twist_mutex_);
        current_twist_ = *msg.get();
    }

}

void TracerROSMessenger::GetCurrentMotionCmdForSim(double &linear, double &angular)
{
    std::lock_guard<std::mutex> guard(twist_mutex_);
    linear = current_twist_.linear.x;
    angular = current_twist_.angular.z;
}


void TracerROSMessenger::LightCmdCallback(const tracer_msgs::msg::TracerLightCmd::SharedPtr msg)
{
    if (!simulated_robot_)
    {
        if (msg->enable_cmd_light_control)
        {
            LightCommandMessage cmd;

            switch (msg->front_mode)
            {
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_OFF:
            {
                cmd.front_light.mode = CONST_OFF;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_ON:
            {
                cmd.front_light.mode = CONST_ON;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_BREATH:
            {
                cmd.front_light.mode = BREATH;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CUSTOM:
            {
                cmd.front_light.mode = CUSTOM;
                cmd.front_light.custom_value = msg->front_custom_value;
                break;
            }
            }

            switch (msg->rear_mode)
            {
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_OFF:
            {
                cmd.rear_light.mode = CONST_OFF;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CONST_ON:
            {
                cmd.rear_light.mode = CONST_ON;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_BREATH:
            {
                cmd.rear_light.mode = BREATH;
                break;
            }
            case tracer_msgs::msg::TracerLightCmd::LIGHT_CUSTOM:
            {
                cmd.rear_light.mode = CUSTOM;
                cmd.rear_light.custom_value = msg->rear_custom_value;
                break;
            }
            }

            tracer_->SetLightCommand(cmd.front_light.mode,cmd.front_light.custom_value);
        }
        else
        {
        }
    }
    else
    {
        std::cout << "simulated robot received light control cmd" << std::endl;
    }   
}

void TracerROSMessenger::PublishStateToROS()
{
    current_time_ = rclcpp::Clock().now();
    double dt = (current_time_ - last_time_).seconds();

    static bool init_run = true;
    if(init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    auto robot_state = tracer_->GetRobotState();
    auto actuator_state = tracer_->GetActuatorState();

    // publish tracer state message
    tracer_msgs::msg::TracerStatus status_msg;

    // status_msg.header.stamp = current_time_;
    status_msg.linear_velocity = robot_state.motion_state.linear_velocity;
    status_msg.angular_velocity = robot_state.motion_state.angular_velocity;

    status_msg.base_state = robot_state.system_state.vehicle_state;
    status_msg.control_mode = robot_state.system_state.control_mode;
    status_msg.fault_code = robot_state.system_state.error_code;
    status_msg.battery_voltage = robot_state.system_state.battery_voltage;

    for (int i = 0; i < 2; ++i)
    {
        //status_msg.motor_states[i]. = state.motor_states[i].current;
        status_msg.motor_states[i].rpm = actuator_state.actuator_hs_state[i].rpm;
        //status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
    }

    status_msg.light_control_enabled = robot_state.light_state.enable_cmd_ctrl;
    status_msg.front_light_state.mode = robot_state.light_state.front_light.mode;
    status_msg.front_light_state.custom_value = robot_state.light_state.front_light.custom_value;
    status_publisher_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(status_msg.linear_velocity, status_msg.angular_velocity, dt);

    // record time for next integration
    last_time_ = current_time_;
}

void TracerROSMessenger::PublishSimStateToROS(double linear, double angular)
{
    current_time_ = ros2_node_->now();
    double dt = 1.0 / sim_control_rate_;

    // publish tracer state message
    tracer_msgs::msg::TracerStatus status_msg;

    // status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = linear;
    status_msg.angular_velocity = angular;

    status_msg.base_state = 0x00;
    status_msg.control_mode = 0x01;
    status_msg.fault_code = 0x00;
    status_msg.battery_voltage = 29.5;

    // for (int i = 0; i < 4; ++i)
    // {
    //     status_msg.motor_states[i].current = state.motor_states[i].current;
    //     status_msg.motor_states[i].rpm = state.motor_states[i].rpm;
    //     status_msg.motor_states[i].temperature = state.motor_states[i].temperature;
    // }

    status_msg.light_control_enabled = false;
    // status_msg.front_light_state.mode = state.front_light_state.mode;
    // status_msg.front_light_state.custom_value = state.front_light_state.custom_value;
    // status_msg.rear_light_state.mode = state.rear_light_state.mode;
    // status_msg.rear_light_state.custom_value = state.front_light_state.custom_value;

    status_publisher_->publish(status_msg);
    // publish odometry and tf
    PublishOdometryToROS(linear, angular, dt); 
}

geometry_msgs::msg::Quaternion TracerROSMessenger::createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

void TracerROSMessenger::PublishOdometryToROS(double linear, double angular, double dt)
{
    // perform numerical integration to get an estimation of pose
    linear_speed_ = linear;
    angular_speed_ = angular;

    current_time_ = rclcpp::Clock().now();

    double d_x = linear_speed_ * std::cos(theta_) * dt;
    double d_y = linear_speed_ * std::sin(theta_) * dt;
    double d_theta = angular_speed_ * dt;

    position_x_ += d_x;
    position_y_ += d_y;
    theta_ += d_theta;

    // geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    // tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(odom_quat);
    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(odom_quat);

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_speed_;

    odom_publisher_->publish(odom_msg);
}

} // namespace westonrobot




