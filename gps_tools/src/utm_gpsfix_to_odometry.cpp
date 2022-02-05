/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include "gps_tools/utm_gpsfix_to_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_msgs/msg/gps_status.hpp>
#include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <utility>

namespace gps_tools
{

#define DEG_TO_RAD_FACTOR 0.0174532925

UTMGPSFixToOdometryComponent::UTMGPSFixToOdometryComponent(const rclcpp::NodeOptions & options)
: Node("utm_odometry_node", options),
  frame_id_(""),
  child_frame_id_(""),
  append_zone_(false)
{
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  get_parameter_or("child_frame_id", child_frame_id_, child_frame_id_);
  get_parameter_or("frame_id", frame_id_, frame_id_);
  get_parameter_or("append_zone", append_zone_, append_zone_);

  fix_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
    "fix",
    rclcpp::SensorDataQoS(rclcpp::KeepLast(5)),
    std::bind(&UTMGPSFixToOdometryComponent::gpsfix_callback, this, std::placeholders::_1)
  );
}

void UTMGPSFixToOdometryComponent::gpsfix_callback(gps_msgs::msg::GPSFix::UniquePtr fix)
{
  if (fix->status.status == gps_msgs::msg::GPSStatus::STATUS_NO_FIX) {
    RCLCPP_DEBUG(this->get_logger(), "No fix.");
    return;
  }

  if (fix->header.stamp.nanosec == 0 && fix->header.stamp.sec == 0) {
    RCLCPP_WARN(get_logger(), "No timestamp in GPSFix received.");
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  nav_msgs::msg::Odometry::UniquePtr odom;
  odom->header.stamp = fix->header.stamp;

  if (frame_id_.empty()) {
    if (append_zone_) {
      odom->header.frame_id = fix->header.frame_id + "/utm_" + zone;
    } else {
      odom->header.frame_id = fix->header.frame_id;
    }
  } else {
    if (append_zone_) {
      odom->header.frame_id = frame_id_ + "/utm_" + zone;
    } else {
      odom->header.frame_id = frame_id_;
    }
  }

  odom->child_frame_id = child_frame_id_;

  // Pose

  odom->pose.pose.position.x = easting;
  odom->pose.pose.position.y = northing;
  odom->pose.pose.position.z = fix->altitude;

  double yaw = fix->track * DEG_TO_RAD_FACTOR;

  // We have to flip roll and pitch and flip the yaw to
  // get the orientation in ENU (REP103)
  tf2::Quaternion q;
  q.setEuler(
    -yaw,
    (fix->roll * DEG_TO_RAD_FACTOR),
    (fix->pitch * DEG_TO_RAD_FACTOR));

  odom->pose.pose.orientation.x = q.x();
  odom->pose.pose.orientation.y = q.y();
  odom->pose.pose.orientation.z = q.z();
  odom->pose.pose.orientation.w = q.w();

  auto yaw_error = fix->err_track * DEG_TO_RAD_FACTOR;

  // Assumes position_covariance uses ENU (REP103)
  // XYZRPY 6x6 coveriance matrix
  std::array<double, 36> pose_covariance = {{
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
    0, 0, 0, fix->err_roll, 0, 0,
    0, 0, 0, 0, fix->err_pitch, 0,
    0, 0, 0, 0, 0, yaw_error
  }};

  odom->pose.covariance = pose_covariance;

  // Twist

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  odom->twist.twist.linear.x = fix->speed * sin_yaw;
  odom->twist.twist.linear.y = fix->speed * cos_yaw;
  odom->twist.twist.linear.z = fix->climb;

  if (fix->err_speed > 0.0) {
    // Propagate errors of speed and track - TODO(SivertHavso): confirm algorithm
    odom->twist.covariance[0] = sin_yaw * fix->err_speed + fix->speed * cos_yaw * fix->err_track;
    odom->twist.covariance[4] = cos_yaw * fix->err_speed + fix->speed * sin_yaw * fix->err_track;
  }

  odom->twist.covariance[8] = fix->err_climb;

  odom_pub_->publish(std::move(odom));
}
}  // namespace gps_tools

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UTMGPSFixToOdometryComponent)
