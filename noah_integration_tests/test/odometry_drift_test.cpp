#include <chrono>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

class OdometryDriftCalculator : public rclcpp::Node {
public:
  OdometryDriftCalculator(int capture_miliseconds)
      : Node("odometry_drift_test") {
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/noah/odom", 10,
        std::bind(&OdometryDriftCalculator::odometry_callback, this,
                  std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(capture_miliseconds),
        std::bind(&OdometryDriftCalculator::timer_callback, this));
  }

  double get_accumulated_drift() const {
    std::lock_guard<std::mutex> accum_lock(accumulated_odometry_drift_mutex_);
    return accumulated_odometry_drift_;
  }

private:
  void odometry_callback(
      const nav_msgs::msg::Odometry::SharedPtr current_odometry_msg) {
    if (!previous_odometry_msg_) {
      // On reception of the first odom message, reset drift to 0.
      // This is to fail the test if odometry is never published.
      RCLCPP_INFO(get_logger(), "First odom message received!.");
      accumulated_odometry_drift_ = 0.0;
    } else {
      update_odometry_drift(current_odometry_msg);
    }

    previous_odometry_msg_ =
        std::make_shared<nav_msgs::msg::Odometry>(*current_odometry_msg);
  }

  void timer_callback() {
    timer_->cancel();
    rclcpp::shutdown(nullptr, "Stopping the testing node.");
  }

  void update_odometry_drift(
      const nav_msgs::msg::Odometry::SharedPtr current_odometry_msg) {
    std::lock_guard<std::mutex> accum_lock(accumulated_odometry_drift_mutex_);
    const geometry_msgs::msg::Point current{
        current_odometry_msg->pose.pose.position};
    const geometry_msgs::msg::Point previous{
        previous_odometry_msg_->pose.pose.position};

    const double current_drift = accumulated_odometry_drift_;
    const double new_drift = std::abs(current.x - previous.x) +
                             std::abs(current.y - previous.y) +
                             std::abs(current.z - previous.z);
    accumulated_odometry_drift_ = current_drift + new_drift;
    RCLCPP_INFO(get_logger(), "Currently accumulated drift: " +
                                  std::to_string(accumulated_odometry_drift_));
  }

  mutable std::mutex accumulated_odometry_drift_mutex_{};
  double accumulated_odometry_drift_{std::numeric_limits<double>::infinity()};
  nav_msgs::msg::Odometry::SharedPtr previous_odometry_msg_{nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

#define CAPTURE_MILLISECONDS 1000
#define MAX_ACCEPTED_DRIFT 1e-3

class OdometryDriftTest : public ::testing::Test {
public:
  OdometryDriftTest() {
    auto node = std::make_shared<OdometryDriftCalculator>(CAPTURE_MILLISECONDS);
    rclcpp::spin(node);
    accumulated_drift_ = node->get_accumulated_drift();
  }

  double accumulated_drift_{0.0};
};

TEST_F(OdometryDriftTest, IsDriftAccetable) {
  EXPECT_LE(accumulated_drift_, MAX_ACCEPTED_DRIFT)
      << "Accumulated drift too high.";
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
