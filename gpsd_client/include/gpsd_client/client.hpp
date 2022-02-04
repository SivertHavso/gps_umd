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
  bool start();

  void step();

  void stop();

private:
  void process_data(struct gps_data_t * p);

  void process_data_gps(struct gps_data_t * p);

  void process_data_navsat(struct gps_data_t * p);

  rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;

  gpsmm * gps_;

  bool use_gps_time_;
  bool check_fix_by_variance_;
  std::string frame_id_;
};
}  // namespace gpsd_client

#endif  // GPSD_CLIENT__CLIENT_HPP_
