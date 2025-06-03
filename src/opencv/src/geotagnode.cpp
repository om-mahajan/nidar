#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>
#include <cmath>

constexpr double EARTH_RADIUS_METERS = 6378137.0;

struct GeotaggedPoint {
  double latitude;
  double longitude;
  double altitude;
  double image_x;
  double image_y;
};

class GeotagNode : public rclcpp::Node {
public:
  GeotagNode() : Node("geotag_node") {
    yellow_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "yellow_box_center", 10,
        std::bind(&GeotagNode::yellow_callback, this, std::placeholders::_1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps", 10,
        [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          gps_latitude_ = msg->latitude;
          gps_longitude_ = msg->longitude;
        });

    alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/altitude", 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
          gps_altitude_ = msg->data;
        });

    // Simulated fallback values
    gps_latitude_ = 37.7749;
    gps_longitude_ = -122.4194;
    gps_altitude_ = 15.0;

    // Nadir camera by default (camera Z points down)
    rotation_matrix_ = {
        1.0, 0.0,  0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0
    };

    cv::namedWindow("Geotagged Map", cv::WINDOW_AUTOSIZE);
    map_image_ = cv::Mat::zeros(500, 500, CV_8UC3);
  }

private:
  void yellow_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Camera intrinsics (replace with real values if known)
    double fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0;

    // Image point to normalized camera coordinates
    double x_img = msg->x;
    double y_img = msg->y;
    double x_cam = (x_img - cx) / fx;
    double y_cam = (y_img - cy) / fy;
    double z_cam = 1.0;

    // Apply camera rotation matrix (R * [x_cam, y_cam, z_cam])
    double x_rot = rotation_matrix_[0] * x_cam + rotation_matrix_[1] * y_cam + rotation_matrix_[2] * z_cam;
    double y_rot = rotation_matrix_[3] * x_cam + rotation_matrix_[4] * y_cam + rotation_matrix_[5] * z_cam;
    double z_rot = rotation_matrix_[6] * x_cam + rotation_matrix_[7] * y_cam + rotation_matrix_[8] * z_cam;

    // Assume flat ground at Z = 0, solve for scale s in s * z_rot = altitude
    if (std::abs(z_rot) < 1e-6) {
      RCLCPP_WARN(this->get_logger(), "z_rot too small, skipping point");
      return;
    }
    double scale = gps_altitude_ / z_rot;

    double X_ground = scale * x_rot;
    double Y_ground = scale * y_rot;

    // Convert ground displacement to lat/lon
    double delta_lat = (Y_ground / EARTH_RADIUS_METERS) * (180.0 / M_PI);
    double delta_lon = (X_ground / (EARTH_RADIUS_METERS * std::cos(gps_latitude_ * M_PI / 180.0))) * (180.0 / M_PI);

    GeotaggedPoint point;
    point.latitude = gps_latitude_ + delta_lat;
    point.longitude = gps_longitude_ + delta_lon;
    point.altitude = gps_altitude_;
    point.image_x = x_img;
    point.image_y = y_img;

    geotagged_points_.push_back(point);

    // Display on terminal
    RCLCPP_INFO(this->get_logger(),
                "Yellow zone at pixel (%.1f, %.1f) => GPS (Lat: %.6f, Lon: %.6f, Alt: %.2f)",
                x_img, y_img, point.latitude, point.longitude, point.altitude);

    update_map();
  }

  void update_map() {
    map_image_ = cv::Mat::zeros(500, 500, CV_8UC3);
    for (const auto &pt : geotagged_points_) {
      int x = static_cast<int>(pt.image_x / 5.0);  // scaled for visualization
      int y = static_cast<int>(pt.image_y / 5.0);
      cv::circle(map_image_, cv::Point(x, y), 4, cv::Scalar(0, 255, 255), -1);
    }
    cv::imshow("Geotagged Map", map_image_);
    cv::waitKey(1);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr yellow_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;

  double gps_latitude_;
  double gps_longitude_;
  double gps_altitude_;

  std::vector<GeotaggedPoint> geotagged_points_;
  std::vector<double> rotation_matrix_; // 3x3 row-major
  cv::Mat map_image_;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeotagNode>());
  rclcpp::shutdown();
  return 0;
}
