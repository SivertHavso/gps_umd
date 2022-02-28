/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include "gps_tools/utm_gpsfix_to_odometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_msgs/msg/gps_status.hpp>
// #include <gps_tools/conversions.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <utility>
#include <cmath>

#define S_TO_MS(time) time * 1000u
#define TAU M_PI*2
namespace gps_tools
{

#define DEG_TO_RAD_FACTOR 0.0174532925

UTMGPSFixToOdometryComponent::UTMGPSFixToOdometryComponent(const rclcpp::NodeOptions & options)
: Node("utm_gpsfix_to_odometry_node", options),
  frame_id_("odom"),
  child_frame_id_("gps"),
  append_zone_(false),
  estimate_speed_error_(false),
  estimate_track_error_(false),
  previous_utm{}
{
  // callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  // rclcpp::PublisherOptions pub_options;
  // pub_options.callback_group = callback_group_;

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);// , pub_options);

  declare_parameter("child_frame_id", child_frame_id_);
  declare_parameter("frame_id", frame_id_);
  declare_parameter("append_zone", append_zone_);
  declare_parameter("estimate_speed_error", estimate_speed_error_);
  declare_parameter("estimate_track_error", estimate_track_error_);


  get_parameter("child_frame_id",child_frame_id_);
  get_parameter("frame_id",frame_id_);
  get_parameter("append_zone",append_zone_);
  get_parameter("estimate_speed_error",estimate_speed_error_);
  get_parameter("estimate_track_error",estimate_track_error_);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  fix_sub_ = create_subscription<gps_msgs::msg::GPSFix>(
    "fix",
    rclcpp::SensorDataQoS(rclcpp::KeepLast(5)),
    // [this](const gps_msgs::msg::GPSFix::ConstSharedPtr msg) {gpsfix_callback(msg);}
    std::bind(&UTMGPSFixToOdometryComponent::gpsfix_callback, this, std::placeholders::_1)
    // sub_options
  );

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

inline utm_t estimate_speed_err(const utm_t current, const utm_t previous)
{
  utm_t speed;
  // Speed is change in position over time.
  // Assuming the time has a negilent error, the error of the speed is simply:
  speed.easting_err = current.easting_err + previous.easting_err;
  speed.northing_err = current.northing_err + previous.northing_err;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("jaja"), "Estimating speed error " << speed.easting_err << "   " << speed.northing_err);

  return speed;
}


void UTMGPSFixToOdometryComponent::gpsfix_callback(const gps_msgs::msg::GPSFix::ConstSharedPtr fix)
{
  if (fix->status.status == gps_msgs::msg::GPSStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), S_TO_MS(5), "Got a message with no fix. Not publishing odometry (Message throttled 5s).");
    return;
  }

  if (fix->header.stamp.nanosec == 0 && fix->header.stamp.sec == 0) {
    RCLCPP_WARN(get_logger(), "No timestamp in GPSFix received.");
    return;
  }

  utm_t utm;
  std::string zone;

  double meridian_convergence;
  LLtoUTM(fix->latitude, fix->longitude, utm.northing, utm.easting, zone, meridian_convergence);

  utm.easting_err = fix->position_covariance[0];
  utm.northing_err = fix->position_covariance[4];

  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header = fix->header;

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
  utm.easting -= 425823.24;
  utm.northing -= 5807488.92;

  odom->pose.pose.position.x = utm.easting;
  odom->pose.pose.position.y = utm.northing;
  odom->pose.pose.position.z = fix->altitude;

  // FIXME legacy
  double yaw = std::fmod(fix->track + TAU/2, TAU);
  // new
  // double yaw = fix->track * DEG_TO_RAD_FACTOR;

  // We have to flip roll and pitch and flip the yaw to
  // get the orientation in ENU (REP103)
  tf2::Quaternion q;
  q.setRPY(
    (fix->pitch * DEG_TO_RAD_FACTOR),
    (fix->roll * DEG_TO_RAD_FACTOR), 
    -yaw);

  odom->pose.pose.orientation.x = q.x();
  odom->pose.pose.orientation.y = q.y();
  odom->pose.pose.orientation.z = q.z();
  odom->pose.pose.orientation.w = q.w();

  auto err_track = fix->err_track;

  if (estimate_track_error_) {
    err_track = estimate_track_err(utm, previous_utm);
  }

  auto yaw_error = err_track * DEG_TO_RAD_FACTOR;

  if (fix->speed * 0.0771604944 < 0.1) {  //FIXME 0.0771604944
    yaw_error = std::fmax(yaw_error, TAU);
  }

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
    // 0, 0, 0, fix->err_roll, 0, 0,
    // 0, 0, 0, 0, fix->err_pitch, 0,
    0, 0, 0, 0.5, 0, 0,
    0, 0, 0, 0, 0.5, 0,
    0, 0, 0, 0, 0, yaw_error
  }};

  odom->pose.covariance = pose_covariance;

  // Twist

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  odom->twist.twist.linear.x = fix->speed * sin_yaw * 0.0771604944;  //FIXME remove 0.0771604944
  odom->twist.twist.linear.y = fix->speed * cos_yaw * 0.0771604944;
  odom->twist.twist.linear.z = fix->climb;

  if (estimate_speed_error_) {
    // utm_t speed_err = estimate_speed_err(utm, previous_utm);
    odom->twist.covariance[0]  = utm.easting_err + previous_utm.easting_err * 2;  // * 2 for overestimation
    odom->twist.covariance[4]  = utm.northing_err + previous_utm.northing_err * 2;
  }

  // if (fix->err_speed > 0.0) {
  //   // Propagate errors of speed and track - TODO(SivertHavso): confirm algorithm
  //   odom->twist.covariance[0] = sin_yaw * fix->err_speed + fix->speed * cos_yaw * err_track;
  //   odom->twist.covariance[4] = cos_yaw * fix->err_speed + fix->speed * sin_yaw * err_track;
  // }

  odom->twist.covariance[8] = fix->err_climb;

  geometry_msgs::msg::TransformStamped t;
  t.header = odom->header;
  t.header.frame_id = "odom";
  // t.header.stamp = get_clock()->now();  // FIXME?
  t.child_frame_id = "base_link";
  t.transform.translation.x = odom->pose.pose.position.x;
  t.transform.translation.y = odom->pose.pose.position.y;
  t.transform.translation.z = odom->pose.pose.position.z;
  t.transform.rotation = odom->pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);

  odom_pub_->publish(std::move(odom));

  previous_utm = utm;
}
}  // namespace gps_tools

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UTMGPSFixToOdometryComponent)
