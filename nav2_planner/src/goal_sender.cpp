#include <iostream>
#include <unistd.h>
#include <tf2/utils.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rclcpp/rclcpp.hpp>

bool flag = true;
void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    flag = false;
    std::cout << msg->poses[0].pose.position.x;
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    sleep(8);

    auto node = std::make_shared<rclcpp::Node>("goal_pub");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    geometry_msgs::msg::PoseStamped goal;
    node->declare_parameter<double>("x");
    node->declare_parameter<double>("y");
    node->declare_parameter<double>("heading");
    node->get_parameter<double>("x", goal.pose.position.x);
    node->get_parameter<double>("y", goal.pose.position.y);
    double heading;
    node->get_parameter<double>("y", heading);
    tf2::Quaternion q_tmp;
    q_tmp.setRPY(0, 0, heading);
    goal.pose.orientation = tf2::toMsg(q_tmp);
    goal.header.frame_id = "map";

    auto path_node = std::make_shared<rclcpp::Node>("path_sub");
    auto subscription = path_node->create_subscription<nav_msgs::msg::Path>("unsmoothed_plan", 10, topic_callback);
    while (rclcpp::ok()) {
        rclcpp::spin_some(path_node);
        if (flag == false) {
            break;
        }
        publisher->publish(goal);
        rclcpp::spin_some(node);
        sleep(1);
    }
    RCLCPP_INFO(node->get_logger(), "goal has sent %lf, %lf", goal.pose.position.x, goal.pose.position.x);
    rclcpp::shutdown();

    return 0;
}