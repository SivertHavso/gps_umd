/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include "gps_tools/utm_navsatfix_to_odometry_component.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

namespace gps_tools
{

UtmOdometryComponent::UtmOdometryComponent(const rclcpp::NodeOptions & options)
: Node("utm_odometry_node", options),
  frame_id_(""),
  child_frame_id_(""),
  rot_cov_(99999.0),
  append_zone_(false)
{
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  get_parameter_or("rot_covariance", rot_cov_, rot_cov_);
  get_parameter_or("child_frame_id", child_frame_id_, child_frame_id_);
  get_parameter_or("frame_id", frame_id_, frame_id_);
  get_parameter_or("append_zone", append_zone_, append_zone_);

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix",
    10,
    std::bind(&UtmOdometryComponent::navsatfix_callback, std::placeholders::_1)
  );
}

void UtmOdometryComponent::navsatfix_callback(sensor_msgs::msg::NavSatFix::UniquePtr fix)
{
  if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_DEBUG(this->get_logger(), "No fix.");
    return;
  }

  if (fix->header.stamp.nanosec == 0 && fix->header.stamp.sec == 0) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id_.empty()) {
      if (append_zone_) {
        odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = fix->header.frame_id;
      }
    } else {
      if (append_zone_) {
        odom.header.frame_id = frame_id_ + "/utm_" + zone;
      } else {
        odom.header.frame_id = frame_id_;
      }
    }

    odom.child_frame_id = child_frame_id_;

    odom.pose.pose.position.x = easting;
    odom.pose.pose.position.y = northing;
    odom.pose.pose.position.z = fix->altitude;

    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    std::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov_, 0, 0,
      0, 0, 0, 0, rot_cov_, 0,
      0, 0, 0, 0, 0, rot_cov_
    }};

    odom.pose.covariance = covariance;

    odom_pub_->publish(odom);
  }
}
}  // namespace gps_tools

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UtmOdometryComponent)
