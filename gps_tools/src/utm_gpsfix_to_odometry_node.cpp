#include "gps_tools/utm_gpsfix_to_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

/*!
 * \brief Main.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto gpsd_client_node = std::make_shared<gps_tools::UTMGPSFixToOdometryComponent>(options);

  rclcpp::spin(gpsd_client_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
