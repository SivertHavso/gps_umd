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
#include <tf2_ros/transform_broadcaster.h>

#include <string>

#include "gps_tools/visibility_control.h"

namespace gps_tools
{

typedef struct UTM {
  double easting;
  double easting_err;
  double northing;
  double northing_err;
} utm_t;

// returns radians
inline double estimate_track_err(const utm_t current, const utm_t previous)
{
  if (
    current.easting_err == 0.0 ||
    current.northing_err == 0.0 ||
    previous.easting_err == 0.0 ||
    previous.northing_err == 0.0) {
      return 0.0;
    }

  // Assume arctan through atan2 is used:
  // atan2(delta_nothing, delta_easting) = heading
  //
  // Unless delta_easting = 0 this will use:
  // arctan(delta_nothing / delta_easting)
  // Possibly adding or subtracting PI, but the error for PI is ~0 so we ignore it

  // First calculate the arctan input:
  double delta_easting = current.easting - previous.easting;
  double delta_northing = current.northing - previous.northing;

  if (delta_easting == 0.0) {  // do not divide on zero
    delta_easting = 1e-6;
    // RCLCPP_WARN(rclcpp::get_logger("track_error_estimation"), "delta easting is 0");
  }

  if (delta_northing == 0.0) {  // do not divide on zero
    delta_northing = 1e-6;
    // RCLCPP_WARN(rclcpp::get_logger("track_error_estimation"), "delta northing is 0");
  }

  double northing_over_easting = delta_northing / delta_easting;

  // Then the associated error.
  double delta_easting_err = previous.easting_err + current.easting_err;
  double delta_northing_err = previous.northing_err + current.northing_err;

  // Error propagation for division:
  // err_z/z = err_x/x + err_y/y
  // err_z = (err_x/x + err_y/y) * z


  double northing_over_easting_err
    = std::abs(((delta_easting_err / delta_easting) + (delta_northing_err / delta_northing)) * northing_over_easting);

  // now for arctan(northing / easting) error propogation
  // see https://www.kpu.ca/sites/default/files/Faculty%20of%20Science%20%26%20Horticulture/Physics/PHYS%201120%20Error%20Propagation%20Solutions.pdf  // NOLINT


  return std::abs(northing_over_easting_err / (1 + northing_over_easting * northing_over_easting));
}

class UTMGPSFixToOdometryComponent : public rclcpp::Node
{
public:
  GPS_TOOLS_PUBLIC
  explicit UTMGPSFixToOdometryComponent(const rclcpp::NodeOptions & options);

private:
  void gpsfix_callback(const gps_msgs::msg::GPSFix::ConstSharedPtr fix);

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string frame_id_, child_frame_id_;
  double rot_cov_;
  bool append_zone_;
  bool estimate_speed_error_;
  bool estimate_track_error_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  utm_t previous_utm;
};
}  // namespace gps_tools

#endif  // GPS_TOOLS__UTM_GPSFIX_TO_ODOMETRY_HPP_
