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
#include "walker.hpp"
// @brief walker namespace to hold types and classes for the walker_node
namespace walker {
  // @brief: Timer callback function, checks for obstacle and updates motion
  void Walker::timerCallback() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Motion state updater callback");
    if (lastReceivedImg_.header.stamp.nanosec > 0 &&
        motionState_ == eMotionState::DO_NOTHING) {
      motionState_ = eMotionState::START;
    }
    updateMotionState();
  }
  // @brief: Updates motion state of the robot
  void Walker::updateMotionState() {
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
  inline void Walker::forward() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Moving forward");
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.2;
    commandVelPublisher_->publish(msg);
  }
  // @brief: Publishes on cmd_vel topic to turn the robot in z direction
  inline void Walker::turn() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Turning");
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = 0.2;
    commandVelPublisher_->publish(msg);
  }
  // @brief: Publishes on cmd_vel topic to stop the robot
  inline void Walker::stop() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Stopping");
    auto msg = geometry_msgs::msg::Twist();
    commandVelPublisher_->publish(msg);
  }
  // @brief: Checks if obstacle is detected or not
  // @return: true if obstacle is detected, false otherwise
  bool Walker::obstacleDetected() {
    auto obstacleDetected = false;
    std::unique_lock<std::mutex> lock(dataMutex_);
    const auto& maxHeight = lastReceivedImg_.height;
    const auto& maxWidth = lastReceivedImg_.width;
    const auto& step = lastReceivedImg_.step;
    const auto data = reinterpret_cast<float*>(lastReceivedImg_.data.data());
    RCLCPP_INFO_STREAM(this->get_logger(), "Depth Image received: maxHeight: "
                                               << maxHeight
                                               << " maxWidth: " << maxWidth
                                               << " step: " << step);
    // topPixel is the pixel number from top of the image that should be checked
    // for existence of an image
    constexpr const size_t topPixel = 50;
    // closest tolerated depth
    constexpr const float closestToleratedDepthMeters = 0.5;
    assert(maxHeight > topPixel);
    for (size_t height = 0; height < topPixel; height++) {
      for (size_t width = 0; width < maxWidth; width++) {
        const auto idx = height * maxWidth + width;
        assert((width) <= step / sizeof(float));
        const auto& pixel = data[idx];
        if (pixel < closestToleratedDepthMeters) {
          obstacleDetected = true;
          break;
        }
      }
    }
    return obstacleDetected;
  }
};  // namespace walker
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<walker::Walker>());
  rclcpp::shutdown();
  return 0;
}
