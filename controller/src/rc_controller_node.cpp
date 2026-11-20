#include "controller/rc_controller_node.hpp"

#include <array>
#include <chrono>

#include "utilities/data_utils.hpp"
#include "utilities/utils.hpp"

namespace controller{

RcController::RcController(const rclcpp::NodeOptions& options)
: rclcpp::Node("controller_node", options),
linear_max_vel(get_parameter("linear_max_vel").as_double()),
steering_max_angle(utils::dtor(get_parameter("steering_max.pos").as_double())),
timeout(std::chrono::milliseconds(get_parameter("timeout_ms").as_int())),
last_rx(0, 0, RCL_STEADY_TIME)
{
    const rclcpp::QoS qos(10);
    subscription_can = create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_014", qos, std::bind(&RcController::callback_can, this, std::placeholders::_1));
    publisher_vel = create_publisher<steered_drive_msg::msg::SteeredDrive>("cmd_vel", qos);
    publisher_autonomous = create_publisher<std_msgs::msg::Bool>("autonomous", qos);
    publisher_restart = create_publisher<std_msgs::msg::Empty>("restart", qos);
    publisher_status = create_publisher<std_msgs::msg::Float64MultiArray>("rc_status", qos);
    timer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&RcController::callback_timer, this));

    RCLCPP_INFO(get_logger(), "RcController Node has been started. max vel: %.2f m/s, steering angle: %.1f deg, timeout: %ld ms",
        linear_max_vel, utils::rtod(steering_max_angle), static_cast<long>(get_parameter("timeout_ms").as_int()));
}

bool RcController::is_silent(const rclcpp::Time& now) const {
    return !receiving || now - last_rx > timeout;
}

void RcController::callback_can(const socketcan_interface_msg::msg::SocketcanIF::ConstSharedPtr msg){
    if(msg->candlc != 8){
        return;
    }
    const rclcpp::Time now = steady_clock.now();
    const bool first_frame = is_silent(now);
    last_rx = now;
    receiving = true;
    silent_logged = false;

    std::array<uint8_t, 8> data = msg->candata;
    const int16_t throttle = bytes_to_int16(&data[0]);
    const int16_t steering = bytes_to_int16(&data[2]);
    const uint8_t flags = data[4];
    const bool link = flags & 0x01;
    const bool autonomous = flags & 0x02;
    const bool sticks = flags & 0x04;

    if(first_frame){
        RCLCPP_INFO(get_logger(), "ECUからのフレームを受信");
        restart_counter = data[5];
    }else if(data[5] != restart_counter){
        restart_counter = data[5];
        publisher_restart->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(get_logger(), "再稼働");
    }
    if(first_frame || autonomous != is_autonomous){
        if(autonomous && !is_autonomous){
            publish_cmd_vel(0.0, 0.0);
        }
        is_autonomous = autonomous;
        publish_autonomous();
    }

    if(!is_autonomous){
        publish_cmd_vel(linear_max_vel * throttle / 1000.0, steering_max_angle * steering / 1000.0);
    }

    publish_status(true, link, autonomous, sticks, data[6]);
    if(link != link_up){
        link_up = link;
        if(link){
            RCLCPP_INFO(get_logger(), "RC通信 接続");
        }else{
            RCLCPP_WARN(get_logger(), "RC通信 切断");
        }
    }
}

void RcController::callback_timer(){
    if(!is_silent(steady_clock.now())){
        return;
    }
    receiving = false;
    if(!silent_logged){
        silent_logged = true;
        RCLCPP_WARN(get_logger(), "ECUからのフレームが途絶えました");
    }
    if(!is_autonomous){
        publish_cmd_vel(0.0, 0.0);
    }
    publish_status(false, false, is_autonomous, false, 0.0);
}

void RcController::publish_status(bool alive, bool link, bool autonomous, bool sticks, double quality){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {alive ? 1.0 : 0.0, link ? 1.0 : 0.0, autonomous ? 1.0 : 0.0, sticks ? 1.0 : 0.0, quality};
    publisher_status->publish(msg);
}

void RcController::publish_autonomous(){
    std_msgs::msg::Bool msg;
    msg.data = is_autonomous;
    publisher_autonomous->publish(msg);
    RCLCPP_INFO(get_logger(), "自動フラグ : %d", is_autonomous);
}

void RcController::publish_cmd_vel(double velocity, double steering_angle){
    steered_drive_msg::msg::SteeredDrive msg;
    msg.velocity = velocity;
    msg.steering_angle = steering_angle;
    publisher_vel->publish(msg);
}

}
