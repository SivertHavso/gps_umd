#include "gps_tools/utm_odometry_to_navsatfix_component.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

/*!
 * \brief Main.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto gpsd_client_node = std::make_shared<gps_tools::UtmOdometryToNavSatFixComponent>(options);

  rclcpp::spin(gpsd_client_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
