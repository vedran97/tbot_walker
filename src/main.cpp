/**
 * @file main.cpp
 * @author Vedant Ranade
 * @brief ROS2 tbot3 walker node's main function file
 * @version 0.1
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "walker.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walker::Walker>());
  rclcpp::shutdown();
  return 0;
}
