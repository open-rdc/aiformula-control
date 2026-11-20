#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

#include "chassis_driver/chassis_driver_node.hpp"
#include "controller/controller_node.hpp"
#include "controller/rc_controller_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);

    exec.add_node(chassis_driver_node);
    rclcpp::Node::SharedPtr operator_node;
    const auto teleop = chassis_driver_node->get_parameter_or("teleop", std::string("rc"));
    if(teleop == "joy"){
        operator_node = std::make_shared<controller::Controller>(nodes_option);
    }else if(teleop == "rc"){
        operator_node = std::make_shared<controller::RcController>(nodes_option);
    }else{
        throw std::invalid_argument("teleop must be \"rc\" or \"joy\": " + teleop);
    }
    exec.add_node(operator_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
