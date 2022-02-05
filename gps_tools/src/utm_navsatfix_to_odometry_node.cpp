#include "gps_tools/utm_navsatfix_to_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

/*!
 * \brief Main.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto gpsd_client_node = std::make_shared<gps_tools::UTMNavSatFixToOdometryComponent>(options);

  rclcpp::spin(gpsd_client_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
