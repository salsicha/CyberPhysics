#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

extern camodocal::CameraPtr m_camera;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match_img;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_match_points;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern int ROW;
extern int COL;
extern std::string VINS_RESULT_PATH;
extern int DEBUG_IMAGE;
extern int FAST_RELOCALIZATION;



// rclcpp::Time(double) would treat the value as integer nanoseconds. Split the
// seconds instead, so a header stamp converted to double and back compares
// equal (relocalization matches frames by exact stamp).
inline rclcpp::Time toRosTime(double seconds)
{
    double sec = std::floor(seconds);
    int64_t nsec = std::llround((seconds - sec) * 1e9);
    if (nsec >= 1000000000)
    {
        sec += 1.0;
        nsec -= 1000000000;
    }
    return rclcpp::Time(static_cast<int32_t>(sec), static_cast<uint32_t>(nsec));
}
