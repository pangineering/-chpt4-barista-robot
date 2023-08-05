#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_storage.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h" // Include this header for the tf2_ros::Buffer
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/transform_stamped.h>

class MyPublisher : public rclcpp::Node {
public:
  MyPublisher() : Node("robot_chase_node") {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // Create a buffer and listener for TF2 transforms
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    auto timer_callback = [this]() -> void { this->chase(); };
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    // Publish message at a fixed rate (e.g., 1 Hz)
    // timer_ =
    //    this->create_wall_timer(std::chrono::milliseconds(1000),
    //     std::bind(&MyPublisher::publishMessage, this));
  }

private:
  void chase() {
    try {
      // Wait for the transform to be available between rick/base_link and
      // morty/base_link
      if (tf_buffer_->canTransform("rick/base_link", "morty/base_link",
                                   tf2::TimePoint(),
                                   tf2::durationFromSec(1.0))) {
        // Get the transform between rick/base_link and morty/base_link
        geometry_msgs::msg::TransformStamped transformStamped =
            tf_buffer_->lookupTransform("rick/base_link", "morty/base_link",
                                        tf2::TimePoint());

        // Calculate the distance and angular error between the reference frames
        double error_distance =
            std::sqrt(std::pow(transformStamped.transform.translation.x, 2) +
                      std::pow(transformStamped.transform.translation.y, 2));
        double error_yaw = std::atan2(transformStamped.transform.translation.y,
                                      transformStamped.transform.translation.x);

        // Define angular and linear velocity
        double kp_yaw = 1.0;      // You can adjust this value as needed
        double kp_distance = 0.5; // You can adjust this value as needed
        double angular_velocity = kp_yaw * error_yaw;
        double linear_velocity = kp_distance * error_distance;

        // Create Twist message and publish it
        geometry_msgs::msg::Twist msg;
        msg.angular.z = angular_velocity;
        msg.linear.x = linear_velocity;
        publisher_->publish(msg);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
  }
  void getLatestTransform() {
    try {
      // Get the latest transform from "rick/base_link" to "morty/base_link"
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("morty/base_link", "rick/base_link",
                                      tf2::TimePointZero);

      // Print the transform information
      RCLCPP_INFO(this->get_logger(), "Got latest transform:");
      RCLCPP_INFO(this->get_logger(), "Translation: x = %f, y = %f, z = %f",
                  transform_stamped.transform.translation.x,
                  transform_stamped.transform.translation.y,
                  transform_stamped.transform.translation.z);
      RCLCPP_INFO(this->get_logger(),
                  "Rotation: x = %f, y = %f, z = %f, w = %f",
                  transform_stamped.transform.rotation.x,
                  transform_stamped.transform.rotation.y,
                  transform_stamped.transform.rotation.z,
                  transform_stamped.transform.rotation.w);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Error while getting the transform: %s",
                   ex.what());
    }
  }
  void publishMessage() {
    try {
      // Get the latest transform between "rick/base_link" and "morty/base_link"

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get the transform: %s",
                   ex.what());
    }
    // Create and populate the Twist message
    auto msg = std::make_unique<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.1;  // Set the linear velocity in the x-direction
    msg->angular.z = 0.5; // Set the angular velocity in the z-direction

    // Publish the message
    publisher_->publish(std::move(msg));
  };

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}
