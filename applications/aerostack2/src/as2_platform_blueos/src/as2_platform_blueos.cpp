#include "as2_platform_blueos/as2_platform_blueos.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <as2_core/names/topics.hpp>

namespace
{
constexpr uint16_t MAV_CMD_COMPONENT_ARM_DISARM = 400;
constexpr float MAV_FORCE_DISARM_MAGIC = 21196.0F;
}

namespace blueos_platform
{

BlueOSPlatform::BlueOSPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  mavros_ns_ = this->declare_parameter<std::string>("mavros_namespace", "/mavros");
  guided_mode_ = this->declare_parameter<std::string>("guided_mode", "GUIDED");
  manual_mode_ = this->declare_parameter<std::string>("manual_mode", "LOITER");
  stop_mode_ = this->declare_parameter<std::string>("stop_mode", "BRAKE");
  takeoff_altitude_ = this->declare_parameter<double>("takeoff_altitude", 2.0);
  service_timeout_ = std::chrono::milliseconds(
    this->declare_parameter<int>("service_timeout_ms", 3000));

  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    mavros_ns_ + "/setpoint_velocity/cmd_vel", rclcpp::QoS(10));
  position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    mavros_ns_ + "/setpoint_position/local", rclcpp::QoS(10));

  rclcpp::NodeOptions service_node_options;
  service_node_options.use_global_arguments(false);
  service_node_options.enable_rosout(false);
  service_node_options.start_parameter_services(false);
  service_node_options.start_parameter_event_publisher(false);
  service_node_ = std::make_shared<rclcpp::Node>(
    "blueos_mavros_service_client", service_node_options);
  arm_client_ = service_node_->create_client<mavros_msgs::srv::CommandBool>(
    mavros_ns_ + "/cmd/arming");
  mode_client_ = service_node_->create_client<mavros_msgs::srv::SetMode>(
    mavros_ns_ + "/set_mode");
  takeoff_client_ = service_node_->create_client<mavros_msgs::srv::CommandTOL>(
    mavros_ns_ + "/cmd/takeoff");
  land_client_ = service_node_->create_client<mavros_msgs::srv::CommandTOL>(
    mavros_ns_ + "/cmd/land");
  command_client_ = service_node_->create_client<mavros_msgs::srv::CommandLong>(
    mavros_ns_ + "/cmd/command");

  configureSensors();
}

void BlueOSPlatform::configureSensors()
{
  as2_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    as2_names::topics::sensor_measurements::odom, rclcpp::SensorDataQoS());
  as2_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    as2_names::topics::sensor_measurements::imu, rclcpp::SensorDataQoS());
  as2_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
    as2_names::topics::sensor_measurements::gps, rclcpp::SensorDataQoS());
  as2_battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    as2_names::topics::sensor_measurements::battery, rclcpp::SensorDataQoS());

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    mavros_ns_ + "/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) {
      platform_info_msg_.connected = msg->connected;
      platform_info_msg_.armed = msg->armed;
      platform_info_msg_.offboard = msg->guided || msg->mode == guided_mode_;
    });
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    mavros_ns_ + "/local_position/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const auto & q = msg->pose.pose.orientation;
      vehicle_yaw_ = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      as2_odom_pub_->publish(*msg);
    });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    mavros_ns_ + "/imu/data", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      as2_imu_pub_->publish(*msg);
    });
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    mavros_ns_ + "/global_position/global", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      as2_gps_pub_->publish(*msg);
    });
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    mavros_ns_ + "/battery", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      as2_battery_pub_->publish(*msg);
    });
}

bool BlueOSPlatform::waitForService(const rclcpp::ClientBase::SharedPtr & client)
{
  if (client->service_is_ready()) {
    return true;
  }
  if (!client->wait_for_service(service_timeout_)) {
    RCLCPP_ERROR(this->get_logger(), "MAVROS service %s is unavailable", client->get_service_name());
    return false;
  }
  return true;
}

bool BlueOSPlatform::setFlightMode(const std::string & mode)
{
  if (!waitForService(mode_client_)) {
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  auto future = mode_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, future, service_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Timed out setting ArduPilot mode to %s", mode.c_str());
    return false;
  }
  return future.get()->mode_sent;
}

bool BlueOSPlatform::ownSetArmingState(bool state)
{
  if (!waitForService(arm_client_)) {
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = state;
  auto future = arm_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, future, service_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  return future.get()->success;
}

bool BlueOSPlatform::ownSetOffboardControl(bool offboard)
{
  return setFlightMode(offboard ? guided_mode_ : manual_mode_);
}

bool BlueOSPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & mode)
{
  const bool hover = mode.control_mode == as2_msgs::msg::ControlMode::HOVER;
  const bool speed =
    mode.control_mode == as2_msgs::msg::ControlMode::SPEED &&
    (mode.reference_frame == as2_msgs::msg::ControlMode::BODY_FLU_FRAME ||
    mode.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME);
  const bool position =
    mode.control_mode == as2_msgs::msg::ControlMode::POSITION &&
    mode.reference_frame == as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  const bool supported = hover || speed || position;
  if (!supported) {
    RCLCPP_ERROR(
      this->get_logger(), "Unsupported BlueOS control mode/frame: %d/%d",
      mode.control_mode, mode.reference_frame);
    return false;
  }
  requested_control_mode_ = mode;
  return true;
}

bool BlueOSPlatform::ownSendCommand()
{
  switch (requested_control_mode_.control_mode) {
    case as2_msgs::msg::ControlMode::HOVER: {
        geometry_msgs::msg::TwistStamped hover;
        hover.header.stamp = this->now();
        hover.header.frame_id = command_twist_msg_.header.frame_id;
        velocity_pub_->publish(hover);
        return true;
    }
    case as2_msgs::msg::ControlMode::SPEED: {
        auto command = command_twist_msg_;
        command.header.stamp = this->now();
        if (requested_control_mode_.reference_frame ==
          as2_msgs::msg::ControlMode::BODY_FLU_FRAME)
        {
          const double body_x = command.twist.linear.x;
          const double body_y = command.twist.linear.y;
          command.twist.linear.x =
            std::cos(vehicle_yaw_) * body_x - std::sin(vehicle_yaw_) * body_y;
          command.twist.linear.y =
            std::sin(vehicle_yaw_) * body_x + std::cos(vehicle_yaw_) * body_y;
        }
        command.header.frame_id = "map";
        velocity_pub_->publish(command);
        return true;
      }
    case as2_msgs::msg::ControlMode::POSITION: {
        command_pose_msg_.header.stamp = this->now();
        position_pub_->publish(command_pose_msg_);
        return true;
      }
    default:
      return false;
  }
}

bool BlueOSPlatform::ownTakeoff()
{
  if (!waitForService(takeoff_client_)) {
    return false;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  request->altitude = takeoff_altitude_;
  auto future = takeoff_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, future, service_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  return future.get()->success;
}

bool BlueOSPlatform::ownLand()
{
  if (!waitForService(land_client_)) {
    return setFlightMode("LAND");
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  auto future = land_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, future, service_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  return future.get()->success;
}

void BlueOSPlatform::ownKillSwitch()
{
  if (!waitForService(command_client_)) {
    RCLCPP_ERROR(this->get_logger(), "Kill switch failed: could not send force disarm command");
    return;
  }
  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = MAV_CMD_COMPONENT_ARM_DISARM;
  request->param1 = 0.0F;
  request->param2 = MAV_FORCE_DISARM_MAGIC;
  auto future = command_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(service_node_, future, service_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Timed out sending force disarm command");
    return;
  }
  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Force disarm command rejected (result %d)", response->result);
  }
}

void BlueOSPlatform::ownStopPlatform()
{
  if (!setFlightMode(stop_mode_)) {
    geometry_msgs::msg::TwistStamped hover;
    hover.header.stamp = this->now();
    velocity_pub_->publish(hover);
  }
}

}  // namespace blueos_platform
