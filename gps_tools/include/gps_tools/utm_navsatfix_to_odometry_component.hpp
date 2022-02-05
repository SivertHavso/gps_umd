/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */
#ifndef GPS_TOOLS__UTM_NAVSATFIX_TO_ODOMETRY_COMPONENT_HPP_
#define GPS_TOOLS__UTM_NAVSATFIX_TO_ODOMETRY_COMPONENT_HPP_


#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace gps_tools
{
class UtmOdometryComponent : public rclcpp::Node
{
public:
  explicit UtmOdometryComponent(const rclcpp::NodeOptions & options);

private:
  void navsatfix_callback(sensor_msgs::msg::NavSatFix::UniquePtr fix);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string frame_id_, child_frame_id_;
  double rot_cov_;
  bool append_zone_;
};
}  // namespace gps_tools

#endif  // GPS_TOOLS__UTM_NAVSATFIX_TO_ODOMETRY_COMPONENT_HPP_
