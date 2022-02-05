/*
 * Translates nav_msgs/Odometry in UTM coordinates back into sensor_msgs/NavSat{Fix,Status}
 * Useful for visualizing UTM data on a map or comparing with raw GPS data
 * Added by Dheera Venkatraman (dheera@dheera.net)
 */

#ifndef GPS_TOOLS__UTM_ODOMETRY_TO_NAVSATFIX_HPP_
#define GPS_TOOLS__UTM_ODOMETRY_TO_NAVSATFIX_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

#include "gps_tools/visibility_control.h"

namespace gps_tools
{
class UTMOdometryToNavSatFixComponent : public rclcpp::Node
{
public:
  GPS_TOOLS_PUBLIC
  explicit UTMOdometryToNavSatFixComponent(const rclcpp::NodeOptions & options);

private:
  void odom_callback(nav_msgs::msg::Odometry::UniquePtr odom);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

  std::string frame_id_;
  std::string zone_;
};
}  // namespace gps_tools

#endif  // GPS_TOOLS__UTM_ODOMETRY_TO_NAVSATFIX_HPP_
