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
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
enum class eMotionState { START, FORWARD, TURN, DO_NOTHING };
class Walker : public rclcpp::Node {
 public:
  Walker() : Node("walker") {
    commandVelPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&Walker::timerCallback, this));
    motionState_ = eMotionState::DO_NOTHING;
  }

 private:
  void timerCallback() {
    RCLCPP_INFO_STREAM(this->get_logger(),"Timer callback called");
    updateMotionState();
  }
  void updateMotionState() {
    switch (motionState_) {
      case eMotionState::START:
        motionState_ = eMotionState::FORWARD;
        break;
      case eMotionState::FORWARD:
        if (obstacleDetected()) {
          stop();
          // set state to turn
          motionState_ = eMotionState::TURN;
        } else {
          // move forward
          forward();
        }
        break;
      case eMotionState::TURN:
        if (!obstacleDetected()) {
          stop();
          motionState_ = eMotionState::FORWARD;
        } else {
          turn();
        }
        break;
      case eMotionState::DO_NOTHING:
        stop();
        break;
    }
  }
  // @brief: Publishes on cmd_vel topic to move the robot forward in x direction
  inline void forward() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.2;
    commandVelPublisher_->publish(msg);
  }
  // @brief: Publishes on cmd_vel topic to turn the robot in z direction
  inline void turn() {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.2;
    commandVelPublisher_->publish(msg);
  }
  // @brief: Publishes on cmd_vel topic to stop the robot
  inline void stop() {
    auto msg = geometry_msgs::msg::Twist();
    commandVelPublisher_->publish(msg);
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
