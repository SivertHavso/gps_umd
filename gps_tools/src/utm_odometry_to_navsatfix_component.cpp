/*
 * Translates nav_msgs/Odometry in UTM coordinates back into sensor_msgs/NavSat{Fix,Status}
 * Useful for visualizing UTM data on a map or comparing with raw GPS data
 * Added by Dheera Venkatraman (dheera@dheera.net)
 */
#include "gps_tools/utm_odometry_to_navsatfix_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace gps_tools
{
UtmOdometryToNavSatFixComponent::UtmOdometryToNavSatFixComponent(
  const rclcpp::NodeOptions & options)
: Node("utm_odometry_to_navsatfix_node", options)
{
  get_parameter_or("frame_id", frame_id_, std::string(""));
  get_parameter_or("zone", zone_, std::string(""));

  fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    10,
    std::bind(&UtmOdometryToNavSatFixComponent::odom_callback, std::placeholders::_1)
  );
}

void UtmOdometryToNavSatFixComponent::odom_callback(nav_msgs::msg::Odometry::UniquePtr odom)
{
  if (odom->header.stamp.sec == 0 && odom->header.stamp.nanosec == 0) {
    return;
  }

  if (!fix_pub_) {
    return;
  }

  double northing, easting, latitude, longitude;
  std::string zone;
  sensor_msgs::msg::NavSatFix fix;

  northing = odom->pose.pose.position.y;
  easting = odom->pose.pose.position.x;

  if (zone_.length() > 0) {
    // utm zone was supplied as a ROS parameter
    zone = zone_;
    fix.header.frame_id = odom->header.frame_id;
  } else {
    // look for the utm zone in the frame_id
    std::size_t pos = odom->header.frame_id.find("/utm_");
    if (pos == std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "UTM zone not found in frame_id");
      return;
    }
    zone = odom->header.frame_id.substr(pos + 5, 3);
    fix.header.frame_id = odom->header.frame_id.substr(0, pos);
  }

  RCLCPP_INFO(this->get_logger(), "zone: %s", zone.c_str());

  fix.header.stamp = odom->header.stamp;

  UTMtoLL(northing, easting, zone, latitude, longitude);

  fix.latitude = latitude;
  fix.longitude = longitude;
  fix.altitude = odom->pose.pose.position.z;

  fix.position_covariance[0] = odom->pose.covariance[0];
  fix.position_covariance[1] = odom->pose.covariance[1];
  fix.position_covariance[2] = odom->pose.covariance[2];
  fix.position_covariance[3] = odom->pose.covariance[6];
  fix.position_covariance[4] = odom->pose.covariance[7];
  fix.position_covariance[5] = odom->pose.covariance[8];
  fix.position_covariance[6] = odom->pose.covariance[12];
  fix.position_covariance[7] = odom->pose.covariance[13];
  fix.position_covariance[8] = odom->pose.covariance[14];

  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  fix_pub_->publish(fix);
};
}  // namespace gps_tools

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UtmOdometryToNavSatFixComponent)
