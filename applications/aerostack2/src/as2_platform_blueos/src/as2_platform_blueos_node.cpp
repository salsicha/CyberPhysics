#include <memory>

#include <as2_core/core_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_platform_blueos/as2_platform_blueos.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<blueos_platform::BlueOSPlatform>();
  as2::spinLoop(node);
  rclcpp::shutdown();
  return 0;
}
