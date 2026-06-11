#ifndef AS2_PLATFORM_BLUEOS__AS2_PLATFORM_BLUEOS_HPP_
#define AS2_PLATFORM_BLUEOS__AS2_PLATFORM_BLUEOS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <as2_core/aerial_platform.hpp>
#include <as2_msgs/msg/control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace blueos_platform
{

class BlueOSPlatform : public as2::AerialPlatform
{
public:
  explicit BlueOSPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & mode) override;
  bool ownTakeoff() override;
  bool ownLand() override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

private:
  bool setFlightMode(const std::string & mode);
  bool waitForService(const rclcpp::ClientBase::SharedPtr & client);

  std::string mavros_ns_;
  std::string guided_mode_;
  std::string manual_mode_;
  std::string stop_mode_;
  double takeoff_altitude_;
  double vehicle_yaw_{0.0};
  std::chrono::milliseconds service_timeout_;

  as2_msgs::msg::ControlMode requested_control_mode_;
  rclcpp::Node::SharedPtr service_node_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr as2_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr as2_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr as2_gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr as2_battery_pub_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
};

}  // namespace blueos_platform

#endif  // AS2_PLATFORM_BLUEOS__AS2_PLATFORM_BLUEOS_HPP_
