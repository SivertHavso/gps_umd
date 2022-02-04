#include "gpsd_client/client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libgpsmm.h>

#include <cmath>
#include <string>
#include <utility>
#include <memory>

namespace gpsd_client
{

GPSDClientComponent::GPSDClientComponent(const rclcpp::NodeOptions & options)
: Node("gpsd_client", options),
  gps_(nullptr),
  use_gps_time_(true),
  check_fix_by_variance_(true),
  frame_id_("gps")
{}


bool GPSDClientComponent::start()
{
  gps_fix_pub_ = create_publisher<gps_msgs::msg::GPSFix>("extended_fix", 1);
  navsatfix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

  get_parameter_or("use_gps_time", use_gps_time_, use_gps_time_);
  get_parameter_or("check_fix_by_variance", check_fix_by_variance_, check_fix_by_variance_);
  get_parameter_or("frame_id", frame_id_, frame_id_);

  std::string host = "localhost";
  int port = 2947;
  get_parameter_or("host", host, host);
  get_parameter_or("port", port, port);

  char port_s[12];
  snprintf(port_s, sizeof(port_s), "%d", port);

  gps_data_t * resp = nullptr;

#if GPSD_API_MAJOR_VERSION >= 5
  gps_ = new gpsmm(host.c_str(), port_s);
  resp = gps_->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
  gps = new gpsmm();
  gps->open(host.c_str(), port_s);
  resp = gps->stream(WATCH_ENABLE);
#else
  gps = new gpsmm();
  resp = gps->open(host.c_str(), port_s);
  gps->query("w\n");
#endif

  if (resp == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open GPSd");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "GPSd opened");
  return true;
}


void GPSDClientComponent::step()
{
#if GPSD_API_MAJOR_VERSION >= 5
  if (!gps_->waiting(1e6)) {
    return;
  }

  gps_data_t * p = gps_->read();
#else
  gps_data_t * p = gps->poll();
#endif
  process_data(p);
}


void GPSDClientComponent::stop()
{
  // gpsmm doesn't have a close method? OK ...
}


void GPSDClientComponent::process_data(struct gps_data_t * p)
{
  if (p == nullptr) {
    return;
  }

#if GPSD_API_MAJOR_VERSION >= 9
  if (!p->online.tv_sec && !p->online.tv_nsec) {
#else
  if (!p->online) {
#endif
    return;
  }

  process_data_gps(p);
  process_data_navsat(p);
}


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif

void GPSDClientComponent::process_data_gps(struct gps_data_t * p)
{
  rclcpp::Time time = this->get_clock()->now();

  gps_msgs::msg::GPSFix::UniquePtr fix = std::make_unique<gps_msgs::msg::GPSFix>();

  fix->header.stamp = time;
  fix->header.frame_id = frame_id_;

  fix->status.satellites_used = p->satellites_used;

  fix->status.satellite_used_prn.resize(fix->status.satellites_used);
  for (int i = 0; i < fix->status.satellites_used; ++i) {
#if GPSD_API_MAJOR_VERSION > 5
    fix->status.satellite_used_prn[i] = p->skyview[i].used;
#else
    fix->status.satellite_used_prn[i] = p->used[i];
#endif
  }

  fix->status.satellites_visible = SATS_VISIBLE;

  fix->status.satellite_visible_prn.resize(fix->status.satellites_visible);
  fix->status.satellite_visible_z.resize(fix->status.satellites_visible);
  fix->status.satellite_visible_azimuth.resize(fix->status.satellites_visible);
  fix->status.satellite_visible_snr.resize(fix->status.satellites_visible);

  for (int i = 0; i < SATS_VISIBLE; ++i) {
#if GPSD_API_MAJOR_VERSION > 5
    fix->status.satellite_visible_prn[i] = p->skyview[i].PRN;
    fix->status.satellite_visible_z[i] = p->skyview[i].elevation;
    fix->status.satellite_visible_azimuth[i] = p->skyview[i].azimuth;
    fix->status.satellite_visible_snr[i] = p->skyview[i].ss;
#else
    fix->status.satellite_visible_prn[i] = p->PRN[i];
    fix->status.satellite_visible_z[i] = p->elevation[i];
    fix->status.satellite_visible_azimuth[i] = p->azimuth[i];
    fix->status.satellite_visible_snr[i] = p->ss[i];
#endif
  }

  if ((p->status & STATUS_FIX) && !(check_fix_by_variance_ && std::isnan(p->fix.epx))) {
    fix->status.status = 0;  // FIXME: gpsmm puts its constants in the global
    // namespace, so `GPSStatus::STATUS_FIX' is illegal.

// STATUS_DGPS_FIX was removed in API version 6 but re-added afterward
#if GPSD_API_MAJOR_VERSION != 6
    if (p->status & STATUS_DGPS_FIX) {
      fix->status.status |= 18;  // same here
    }
#endif

#if GPSD_API_MAJOR_VERSION >= 9
    fix->time = static_cast<double>(p->fix.time.tv_sec) +
      static_cast<double>(p->fix.time.tv_nsec) / 1000000.;
#else
    fix->time = p->fix.time;
#endif
    fix->latitude = p->fix.latitude;
    fix->longitude = p->fix.longitude;
    fix->altitude = p->fix.altitude;
    fix->track = p->fix.track;
    fix->speed = p->fix.speed;
    fix->climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
    fix->pdop = p->dop.pdop;
    fix->hdop = p->dop.hdop;
    fix->vdop = p->dop.vdop;
    fix->tdop = p->dop.tdop;
    fix->gdop = p->dop.gdop;
#else
    fix->pdop = p->pdop;
    fix->hdop = p->hdop;
    fix->vdop = p->vdop;
    fix->tdop = p->tdop;
    fix->gdop = p->gdop;
#endif

#if GPSD_API_MAJOR_VERSION < 8
    fix->err = p->epe;
#else
    fix->err = p->fix.eph;
#endif
    fix->err_vert = p->fix.epv;
    fix->err_track = p->fix.epd;
    fix->err_speed = p->fix.eps;
    fix->err_climb = p->fix.epc;
    fix->err_time = p->fix.ept;

    /* TODO: attitude */
  } else {
    fix->status.status = -1;  // STATUS_NO_FIX
  }

  gps_fix_pub_->publish(std::move(fix));
}

void GPSDClientComponent::process_data_navsat(struct gps_data_t * p)
{
  sensor_msgs::msg::NavSatFix::UniquePtr fix = std::make_unique<sensor_msgs::msg::NavSatFix>();

  /* TODO: Support SBAS and other GBAS. */

#if GPSD_API_MAJOR_VERSION >= 9
  if (use_gps_time_ && (p->online.tv_sec || p->online.tv_nsec)) {
    fix->header.stamp = rclcpp::Time(p->fix.time.tv_sec, p->fix.time.tv_nsec);
#else
  if (use_gps_time_ && !std::isnan(p->fix.time)) {
    fix->header.stamp = rclcpp::Time(p->fix.time);
#endif
  } else {
    fix->header.stamp = this->get_clock()->now();
  }

  fix->header.frame_id = frame_id_;

  /* gpsmm pollutes the global namespace with STATUS_,
    * so we need to use the ROS message's integer values
    * for status.status
    */
  switch (p->status) {
    case STATUS_NO_FIX:
      fix->status.status = -1;  // NavSatStatus::STATUS_NO_FIX;
      break;
    case STATUS_FIX:
      fix->status.status = 0;  // NavSatStatus::STATUS_FIX;
      break;
// STATUS_DGPS_FIX was removed in API version 6 but re-added afterward
#if GPSD_API_MAJOR_VERSION != 6
    case STATUS_DGPS_FIX:
      fix->status.status = 2;  // NavSatStatus::STATUS_GBAS_FIX;
      break;
#endif
  }

  fix->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  fix->latitude = p->fix.latitude;
  fix->longitude = p->fix.longitude;
  fix->altitude = p->fix.altitude;

  /* gpsd reports status=OK even when there is no current fix,
    * as long as there has been a fix previously. Throw out these
    * fake results, which have NaN variance
    */
  if (std::isnan(p->fix.epx) && check_fix_by_variance_) {
    return;
  }

  fix->position_covariance[0] = p->fix.epx;
  fix->position_covariance[4] = p->fix.epy;
  fix->position_covariance[8] = p->fix.epv;

  fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  navsatfix_pub_->publish(std::move(fix));
}
}  // namespace gpsd_client

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GPSDClientComponent)
