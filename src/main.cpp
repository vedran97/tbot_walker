#include "walker.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walker::Walker>());
  rclcpp::shutdown();
  return 0;
}
