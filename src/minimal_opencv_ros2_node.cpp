#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
  // Load your image from file (replace with your actual path)
  std::string image_path = "/home/om/ardu_ws/src/opencv/src/p1.jpeg";
  cv::Mat my_image = cv::imread(image_path);

  // Convert to HSV
  cv::Mat hsv_image;
  cv::cvtColor(my_image, hsv_image, cv::COLOR_BGR2HSV);

  // Yellow range in HSV
  cv::Scalar lower_yellow(20, 100, 100);
  cv::Scalar upper_yellow(30, 255, 255);
  cv::Mat yellow_mask;
  cv::inRange(hsv_image, lower_yellow, upper_yellow, yellow_mask);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Draw bounding boxes around yellow regions
  for (const auto& contour : contours) {
    if (cv::contourArea(contour) < 100) continue; // Ignore noise
    cv::Rect box = cv::boundingRect(contour);
    cv::rectangle(my_image, box, cv::Scalar(255, 0, 0), 2); // Blue box
  }

  // Convert and publish
  std_msgs::msg::Header header;
  header.stamp = this->now();
  auto msg = cv_bridge::CvImage(header, "bgr8", my_image).toImageMsg();

  publisher_->publish(*msg);
  RCLCPP_INFO(this->get_logger(), "Image %ld published with yellow detection", count_);
  count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}