/**
 * @file walker.cpp
 * @author Vedant Ranade
 * @brief ROS2 tbot3 walker node
 * @version 0.1
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
enum class eMotionState { START, FORWARD, TURN, DO_NOTHING };
class Walker : public rclcpp::Node {
 public:
  Walker() : Node("walker") {
    commandVelPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Walker::timerCallback, this));
    motionState_ = eMotionState::DO_NOTHING;
  }

 private:
  void timerCallback() { ; }
  void updateMotionState() {
    switch (motionState_) {
      case eMotionState::START:
        motionState_ = eMotionState::FORWARD;
        break;
      case eMotionState::FORWARD:
        if (obstacleDetected()) {
          // todo publish stop motion
          // set state to turn
          motionState_ = eMotionState::TURN;
        } else {
          // todo publish forward motion
        }
        break;
      case eMotionState::TURN:
        if (!obstacleDetected()) {
          // todo publish stop motion
          motionState_ = eMotionState::FORWARD;
        } else {
          // todo publish turn motion
        }
        break;
      case eMotionState::DO_NOTHING:
        break;
    }
  }
  bool obstacleDetected() {
    auto obstacleDetected = false;
    return obstacleDetected;
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  eMotionState motionState_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}
