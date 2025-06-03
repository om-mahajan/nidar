#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    center_pub_ = this->create_publisher<geometry_msgs::msg::Point>("yellow_center", 10);
    std::string video_path = "/home/om/ardu_ws/src/opencv/src/greenplantsyellow.mp4";
    cap_.open(video_path);

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file at %s", video_path.c_str());
      return;
    }

    double fps = cap_.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 25.0;
    frame_delay_ = static_cast<int>(1000.0 / fps);

    // Start video processing in a new thread
    video_thread_ = std::thread(&MinimalImagePublisher::process_video, this);
  }

  ~MinimalImagePublisher() {
    if (video_thread_.joinable()) {
      video_thread_.join();
    }
  }

private:
  void process_video() {
    while (rclcpp::ok()) {
      cv::Mat frame;
      cap_ >> frame;
                                                                                                                                                        
      if (frame.empty()) {
        RCLCPP_INFO(this->get_logger(), "End of video reached.");
        break;
      }

      // Convert to HSV
      cv::Mat hsv_image;
      cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

      // Yellow range in HSV
      cv::Scalar lower_yellow(15,  70,  70);
      cv::Scalar upper_yellow(35, 255, 255);
      
      /* cv::Scalar lower_yellow(20, 100, 100); slightly narrower range
      cv::Scalar upper_yellow(30, 255, 255); */
      
      cv::Mat yellow_mask;
      cv::inRange(hsv_image, lower_yellow, upper_yellow, yellow_mask);

      // Find contours
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const auto &contour : contours) {
        if (cv::contourArea(contour) < 100) continue;
        cv::Rect box = cv::boundingRect(contour);
        cv::rectangle(frame, box, cv::Scalar(255, 0, 0), 2);
        int cx = box.x + box.width / 2;
        int cy = box.y + box.height / 2;
        cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
        RCLCPP_INFO(this->get_logger(), "Yellow box center: (%d, %d)", cx, cy);

        geometry_msgs::msg::Point center_msg;
        center_msg.x = static_cast<double>(cx);
        center_msg.y = static_cast<double>(cy);
        center_msg.z = 0.0;
        center_pub_->publish(center_msg);
      }

      // Convert and publish
      std_msgs::msg::Header header;
      header.stamp = this->now();
      auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      publisher_->publish(*msg);

      count_++;
      std::this_thread::sleep_for(std::chrono::milliseconds(frame_delay_));
    }

    rclcpp::shutdown(); // Stop node once video ends
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr center_pub_;
  size_t count_;
  cv::VideoCapture cap_;
  int frame_delay_;
  std::thread video_thread_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
