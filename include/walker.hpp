#ifndef WALKER_808x_HPP
#define WALKER_808x_HPP
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace walker {
enum class eMotionState { START, FORWARD, TURN, DO_NOTHING };
using image = sensor_msgs::msg::Image;
class Walker : public rclcpp::Node {
 public:
  Walker() : Node("walker") {
    commandVelPublisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // timer callback function is called every 100ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&Walker::timerCallback, this));
    // set state to DO_NOTHING
    motionState_ = eMotionState::DO_NOTHING;
    // subscribe to depth image topic
    depthImgSubscriber_ = this->create_subscription<image>(
        "depth_cam/tbot_waffle_cam/depth", 10, [this](const image& msg) {
          std::unique_lock<std::mutex> lock(dataMutex_);
          lastReceivedImg_ = msg;
        });
  }

 private:
  // @brief: Timer callback function, checks for obstacle and updates motion
  void timerCallback();
  // @brief: Updates motion state of the robot
  void updateMotionState();
  // @brief: Publishes on cmd_vel topic to move the robot forward in x direction
  inline void forward();
  // @brief: Publishes on cmd_vel topic to turn the robot in z direction
  inline void turn();
  // @brief: Publishes on cmd_vel topic to stop the robot
  inline void stop();
  // @brief: Checks if obstacle is detected or not
  // @return: true if obstacle is detected, false otherwise
  bool obstacleDetected();
  std::mutex dataMutex_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandVelPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  eMotionState motionState_;
  image lastReceivedImg_;
  rclcpp::Subscription<image>::SharedPtr depthImgSubscriber_;
};
};
#endif