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
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  options.enable_topic_statistics(false);
  options.use_intra_process_comms(true);

  auto gpsd_client_node = std::make_shared<gps_tools::UTMGPSFixToOdometryComponent>(options);

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  // for (auto & weak_group : gpsd_client_node->get_callback_groups()) {
  //   std::shared_ptr<rclcpp::CallbackGroup> group(weak_group);
  //   if (!group->automatically_add_to_executor_with_node()) {
  //     executor.add_callback_group(group, gpsd_client_node->get_node_base_interface());
  //   }
  // }

  executor.add_node(gpsd_client_node);

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
