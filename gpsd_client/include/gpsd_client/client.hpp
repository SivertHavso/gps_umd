#ifndef GPSD_CLIENT__CLIENT_HPP_
#define GPSD_CLIENT__CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libgpsmm.h>

#include <string>

namespace gpsd_client
{
class GPSDClientComponent : public rclcpp::Node
{
public:
  explicit GPSDClientComponent(const rclcpp::NodeOptions & options);

private:
  void process_data(struct gps_data_t * p);

  void process_data_gps(struct gps_data_t * p);

  void process_data_navsat(struct gps_data_t * p);

  // Poll thread (based on Velodyne LIDAR ROS driver)
  void poll_thread();

  void poll();

  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;

  gpsmm * gps_;

  bool use_gps_time_;
  bool check_fix_by_variance_;
  std::string frame_id_;
  unsigned int gps_timeout_us_;

  // We use this future/promise pair to notify threads that we are shutting down
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  // The thread that deals with data
  std::thread poll_thread_;
};
}  // namespace gpsd_client

#endif  // GPSD_CLIENT__CLIENT_HPP_
