/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */
#ifndef GPS_TOOLS__UTM_GPSFIX_TO_ODOMETRY_HPP_
#define GPS_TOOLS__UTM_GPSFIX_TO_ODOMETRY_HPP_


#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

#include "gps_tools/visibility_control.h"

namespace gps_tools
{
class UTMGPSFixToOdometryComponent : public rclcpp::Node
{
public:
  GPS_TOOLS_PUBLIC
  explicit UTMGPSFixToOdometryComponent(const rclcpp::NodeOptions & options);

private:
  void gpsfix_callback(gps_msgs::msg::GPSFix::UniquePtr fix);

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string frame_id_, child_frame_id_;
  double rot_cov_;
  bool append_zone_;
};
}  // namespace gps_tools

#endif  // GPS_TOOLS__UTM_GPSFIX_TO_ODOMETRY_HPP_
