#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cstdint>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "steered_drive_msg/msg/steered_drive.hpp"

namespace controller{

class RcController : public rclcpp::Node {
public:
    explicit RcController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void callback_can(const socketcan_interface_msg::msg::SocketcanIF::ConstSharedPtr msg);
    void callback_timer();
    bool is_silent(const rclcpp::Time& now) const;
    void publish_status(bool alive, bool link, bool autonomous, bool sticks, double quality);
    void publish_autonomous();
    void publish_cmd_vel(double velocity, double steering_angle);

    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr subscription_can;
    rclcpp::Publisher<steered_drive_msg::msg::SteeredDrive>::SharedPtr publisher_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_autonomous;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_restart;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_status;
    rclcpp::TimerBase::SharedPtr timer;

    const double linear_max_vel;
    const double steering_max_angle;
    const rclcpp::Duration timeout;

    rclcpp::Clock steady_clock{RCL_STEADY_TIME};
    rclcpp::Time last_rx;
    bool receiving = false;
    bool silent_logged = false;
    bool is_autonomous = false;
    bool link_up = false;
    uint8_t restart_counter = 0;
};

}
