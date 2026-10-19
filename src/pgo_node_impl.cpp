#include <pcl/common/io.h>
#include <pcl/exceptions.h>
#include <pcl/io/pcd_io.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "nlohmann/json.hpp"
#include "syncai_mapping/dynamic_removal/edits.hpp"
#include "syncai_mapping/dynamic_removal/recipe.hpp"
#include "syncai_mapping/pgo_node.hpp"
#include "syncai_mapping/pgos/map_writer.h"

extern char ** environ;

namespace syncai_mapping
{

using namespace std::chrono_literals;

namespace
{

// "/<robot_id>/pgo" -> "/<robot_id>"; "/pgo" or "/" (a bare run) -> "".
std::string parentNamespace(const std::string & ns)
{
  const auto slash = ns.find_last_of('/');
  return (slash == std::string::npos || slash == 0) ? std::string() : ns.substr(0, slash);
}

}  // namespace

PGONode::PGONode() : Node("pgo_node")
{
  RCLCPP_INFO(this->get_logger(), "PGO node started");
  loadParameters();
  m_pgo = std::make_shared<PoseGraphOptimizer>(m_pgo_config);
  rclcpp::QoS qos = rclcpp::QoS(10);
  m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
  m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
  // Outputs are relative (/<robot_id>/pgo/...) so robots sharing a DDS domain never collide; the
  // inputs are absolute because they live in pointlio's namespace.
  m_loop_marker_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_markers", 10000);
  // Depth 1: each message is a multi-MB merge and only the latest matters. rviz only: a large
  // site's merge (tens of MB) does not survive CycloneDDS/UDP on `lo` with the default
  // net.core.rmem_max, hence the file hand-off below.
  m_map_cloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", rclcpp::QoS(1));
  // ~200 B JSON notice naming the PCD in map_cloud_dir (fields: mapCloudNoticeJson). Latched so a
  // (re)started backend gets the current map at once; the reader must request TRANSIENT_LOCAL
  // too. The empty-map notice replaces it, so a late joiner never learns of a deleted file.
  m_map_cloud_file_pub = this->create_publisher<std_msgs::msg::String>(
    "map_cloud_file", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  // Latched for the same reason. Republished at 1 Hz so the counters move and a consumer can age
  // a sample out once this node is gone.
  m_status_pub = this->create_publisher<syncai_common::msg::MappingStatus>(
    "mapping_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  setupMapCloudDir();
  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  m_sync =
    std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
      message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
      m_cloud_sub, m_odom_sub);
  m_sync->setAgePenalty(0.1);
  m_sync->registerCallback(
    std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
  m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
  m_status_timer = this->create_wall_timer(1s, std::bind(&PGONode::statusTimerCB, this));
  m_save_map_srv = this->create_service<syncai_common::srv::SaveMaps>(
    "save_maps",
    std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

  // A blocking handler must not sit on the group that owns the 50 ms TF broadcast. The client
  // needs its own group too, or it deadlocks: the service blocks on the ResetLIO future, so the
  // response must arrive on another executor thread. (Never spin_until_future_complete inside a
  // callback either: it re-enters the busy executor.)
  m_srv_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_cli_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  m_reset_srv = this->create_service<syncai_common::srv::ResetMapping>(
    "reset_mapping",
    std::bind(&PGONode::resetMappingCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);
  // Same group as the reset, so the two beginRun callers serialise.
  m_start_srv = this->create_service<syncai_common::srv::StartMapping>(
    "start_mapping",
    std::bind(&PGONode::startMappingCB, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, m_srv_cb_group);

  m_lio_reset_cli = this->create_client<syncai_common::srv::ResetLIO>(
    m_node_config.lio_reset_service, rmw_qos_profile_services_default, m_cli_cb_group);

  // Logged either way: the symptom of a forgotten Start is silence.
  if (m_node_config.start_on_launch) {
    m_state.phase.store(syncai_common::msg::MappingStatus::MAPPING);
    RCLCPP_WARN(
      this->get_logger(),
      "[PGONode] start_on_launch is set: mapping from the first pair, no start_mapping needed");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode] idle until start_mapping is called; pairs are dropped and only an identity "
      "map -> %s is broadcast meanwhile",
      m_node_config.local_frame.c_str());
  }
  publishStatus();
}

PGONode::~PGONode()
{
  // Join so shutdown does not tear the publisher down under a running merge.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
  // Nobody else removes these (the host tmpfs outlives the container).
  clearMapCloudDir();
  // Neither killed nor waited for: the clean is meant to outlive the mapping session.
  for (const auto & child : m_map_clean_children) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PGONode][map_clean] clean_map pid %d for %s continues detached; its outcome lands in %s",
      child.pid, child.dir.c_str(), dynamic_removal::recipe::kSidecarFile);
  }
}

// ---- The file hand-off (see the map_cloud_file publisher) ----------------

void PGONode::setupMapCloudDir()
{
  m_map_cloud_dir = m_node_config.map_cloud_dir;
  std::error_code ec;
  std::filesystem::create_directories(m_map_cloud_dir, ec);
  if (ec) {
    // Not fatal: a preview must never take down the TF broadcast.
    RCLCPP_ERROR(
      this->get_logger(),
      "[PGONode] cannot create map_cloud_dir %s (%s); the map_cloud_file "
      "hand-off is disabled for this run",
      m_map_cloud_dir.c_str(), ec.message().c_str());
    m_map_cloud_dir_ok = false;
    return;
  }
  m_map_cloud_dir_ok = true;
  // Leftovers of a crash. Only our own files, never the directory.
  clearMapCloudDir();
  RCLCPP_INFO(
    this->get_logger(), "[PGONode] map_cloud merges are handed off as PCD files under %s",
    m_map_cloud_dir.c_str());
}

bool PGONode::isMapCloudFile(const std::filesystem::directory_entry & entry)
{
  // error_code overload: this also runs from the destructor, where a throw is fatal.
  std::error_code ec;
  if (!entry.is_regular_file(ec) || ec) return false;
  const std::string name = entry.path().filename().string();
  if (name.rfind("map_cloud_", 0) != 0) return false;
  const std::string ext = entry.path().extension().string();
  return ext == ".pcd" || ext == ".tmp";
}

// Remove every file this node wrote; failures are only logged.
void PGONode::clearMapCloudDir()
{
  if (!m_map_cloud_dir_ok) return;
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(m_map_cloud_dir, ec)) {
    if (!isMapCloudFile(entry)) continue;
    std::error_code rm_ec;
    std::filesystem::remove(entry.path(), rm_ec);
    if (rm_ec) {
      RCLCPP_WARN(
        this->get_logger(), "[PGONode] could not remove %s: %s", entry.path().c_str(),
        rm_ec.message().c_str());
    }
  }
}

// Keep the newest two PCDs by seq and delete the rest and any stray .tmp. Two: the backend may be
// acting on the notice one behind the latest. Sorted by seq, not "seq - 2", because the empty
// notice also consumes a seq.
void PGONode::pruneMapCloudFiles()
{
  std::vector<std::pair<uint64_t, std::filesystem::path>> pcds;
  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(m_map_cloud_dir, ec)) {
    if (!isMapCloudFile(entry)) continue;
    if (entry.path().extension() == ".tmp") {
      std::error_code rm_ec;
      std::filesystem::remove(entry.path(), rm_ec);
      continue;
    }
    const std::string stem = entry.path().stem().string();  // map_cloud_<seq>
    try {
      pcds.emplace_back(std::stoull(stem.substr(std::string("map_cloud_").size())), entry.path());
    } catch (const std::exception &) {
      // Not ours after all (or hand-made); leave it alone.
    }
  }
  std::sort(
    pcds.begin(), pcds.end(), [](const auto & a, const auto & b) { return a.first > b.first; });
  for (size_t i = 2; i < pcds.size(); ++i) {
    std::error_code rm_ec;
    std::filesystem::remove(pcds[i].second, rm_ec);
  }
}

// The notice the backend parses (field order fixed by ordered_json):
//   seq       monotonic within this process, informational (restarts at 1)
//   path      absolute path of the PCD, "" when the map is empty
//   points    point count after the voxel filter, 0 when empty
//   frame_id  map_frame: the points are already global
//   stamp     the keyframe's time, as in the PointCloud2 header
std::string PGONode::mapCloudNoticeJson(
  uint64_t seq, const std::string & path, size_t points,
  const builtin_interfaces::msg::Time & time) const
{
  nlohmann::ordered_json j;
  j["seq"] = seq;
  j["path"] = path;
  j["points"] = points;
  j["frame_id"] = m_node_config.map_frame;
  j["stamp"]["sec"] = time.sec;
  j["stamp"]["nanosec"] = time.nanosec;
  // `replace`: map_cloud_dir need not be valid UTF-8, and the notice must not throw.
  return j.dump(-1, ' ', false, nlohmann::ordered_json::error_handler_t::replace);
}

void PGONode::publishMapCloudNotice(
  uint64_t seq, const std::string & path, size_t points, const builtin_interfaces::msg::Time & time)
{
  std_msgs::msg::String msg;
  msg.data = mapCloudNoticeJson(seq, path, points, time);
  m_map_cloud_file_pub->publish(msg);
}

// Write `map_cloud_<seq>.pcd` via .tmp + rename and announce it. Every failure is caught here: an
// exception escaping the worker thread is std::terminate. savePCDFileBinary throws on an empty
// cloud and on a failed write (ENOSPC included), so the empty case is handled first.
void PGONode::writeAndAnnounceMapCloud(
  const CloudType & merged, uint64_t seq, const builtin_interfaces::msg::Time & time)
{
  if (merged.empty()) {
    publishMapCloudNotice(seq, "", 0, time);
    return;
  }
  const std::filesystem::path final_path =
    std::filesystem::path(m_map_cloud_dir) / ("map_cloud_" + std::to_string(seq) + ".pcd");
  const std::filesystem::path tmp_path = final_path.string() + ".tmp";
  try {
    if (pcl::io::savePCDFileBinary(tmp_path.string(), merged) != 0) {
      throw std::runtime_error("savePCDFileBinary returned non-zero");
    }
    std::filesystem::rename(tmp_path, final_path);  // throws filesystem_error
    publishMapCloudNotice(seq, final_path.string(), merged.size(), time);
    pruneMapCloudFiles();
  } catch (const pcl::IOException & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode] map_cloud file hand-off failed writing %s: %s",
      tmp_path.c_str(), e.what());
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode] map_cloud file hand-off failed on %s: %s", tmp_path.c_str(),
      e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "[PGONode] map_cloud file hand-off failed: %s", e.what());
  }
  std::error_code ec;
  std::filesystem::remove(tmp_path, ec);
}

// Fed by params/mapping_params.yaml plus the launch's robot_id-dependent overrides. A missing key
// falls back to the struct default, so a misspelt key is silent: the resolved values are logged
// below for that reason. Read once: `ros2 param set` after startup changes nothing.
void PGONode::loadParameters()
{
  // Absolute names under the parent namespace (pointlio is our sibling; empty for a bare run).
  // body_cloud_dense: clean_map needs the density to tell people from structure.
  const std::string robot_ns = parentNamespace(this->get_namespace());
  m_node_config.cloud_topic = robot_ns + "/pointlio/body_cloud_dense";
  m_node_config.odom_topic = robot_ns + "/pointlio/lio_odom";
  m_node_config.lio_reset_service = robot_ns + "/pointlio/reset";
  // Must equal syncai_pointlio's world_frame: it is NOT adopted from the odom header. TF frames
  // are not namespaced by ROS, so the prefix is spelt out (no leading slash).
  m_node_config.local_frame =
    robot_ns.empty() ? "pointlio_odom" : robot_ns.substr(1) + "/pointlio_odom";
  m_node_config.map_cloud_dir =
    this->declare_parameter("map_cloud_dir", m_node_config.map_cloud_dir);
  m_node_config.map_cloud_resolution =
    this->declare_parameter("map_cloud_resolution", m_node_config.map_cloud_resolution);
  m_node_config.map_cloud_pub_period =
    this->declare_parameter("map_cloud_pub_period", m_node_config.map_cloud_pub_period);
  m_node_config.start_on_launch =
    this->declare_parameter("start_on_launch", m_node_config.start_on_launch);

  // PoseGraphOptimizerConfig. The params file must match each key's type: `10` for a double key
  // (or `5.0` for an int one) throws InvalidParameterTypeException at startup.
  m_pgo_config.keyframe_delta_deg =
    this->declare_parameter("keyframe_delta_deg", m_pgo_config.keyframe_delta_deg);
  m_pgo_config.keyframe_delta_m =
    this->declare_parameter("keyframe_delta_m", m_pgo_config.keyframe_delta_m);
  m_pgo_config.loop_search_radius =
    this->declare_parameter("loop_search_radius", m_pgo_config.loop_search_radius);
  m_pgo_config.loop_time_tresh =
    this->declare_parameter("loop_time_tresh", m_pgo_config.loop_time_tresh);
  m_pgo_config.loop_score_tresh =
    this->declare_parameter("loop_score_tresh", m_pgo_config.loop_score_tresh);
  m_pgo_config.loop_gicp_max_corr_dist =
    this->declare_parameter("loop_gicp_max_corr_dist", m_pgo_config.loop_gicp_max_corr_dist);
  m_pgo_config.loop_gicp_num_threads =
    this->declare_parameter("loop_gicp_num_threads", m_pgo_config.loop_gicp_num_threads);
  m_pgo_config.loop_gicp_num_neighbors =
    this->declare_parameter("loop_gicp_num_neighbors", m_pgo_config.loop_gicp_num_neighbors);
  m_pgo_config.loop_planar_correction =
    this->declare_parameter("loop_planar_correction", m_pgo_config.loop_planar_correction);
  m_pgo_config.loop_noise_var_roll_pitch_z = this->declare_parameter(
    "loop_noise_var_roll_pitch_z", m_pgo_config.loop_noise_var_roll_pitch_z);
  m_pgo_config.loop_noise_yaw_sigma_deg =
    this->declare_parameter("loop_noise_yaw_sigma_deg", m_pgo_config.loop_noise_yaw_sigma_deg);
  m_pgo_config.loop_noise_xy_sigma_m =
    this->declare_parameter("loop_noise_xy_sigma_m", m_pgo_config.loop_noise_xy_sigma_m);
  m_pgo_config.keyframe_tilt_sigma_deg =
    this->declare_parameter("keyframe_tilt_sigma_deg", m_pgo_config.keyframe_tilt_sigma_deg);
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] loop verification: gicp, max corr dist %.2f m, fitness gate %.3f, planar "
    "correction %s, roll/pitch/z variance %.1e, planar xy sigma %.3f m%s, planar yaw sigma "
    "%.2f deg%s, keyframe tilt sigma %.2f deg%s",
    m_pgo_config.loop_gicp_max_corr_dist, m_pgo_config.loop_score_tresh,
    m_pgo_config.loop_planar_correction ? "on" : "off", m_pgo_config.loop_noise_var_roll_pitch_z,
    m_pgo_config.loop_noise_xy_sigma_m,
    m_pgo_config.loop_noise_xy_sigma_m > 0.0 ? "" : " (<= 0: variance = fitness)",
    m_pgo_config.loop_noise_yaw_sigma_deg,
    m_pgo_config.loop_noise_yaw_sigma_deg > 0.0 ? "" : " (<= 0: variance = fitness)",
    m_pgo_config.keyframe_tilt_sigma_deg,
    m_pgo_config.keyframe_tilt_sigma_deg > 0.0 ? "" : " (<= 0: off)");
  m_pgo_config.loop_submap_half_range =
    this->declare_parameter("loop_submap_half_range", m_pgo_config.loop_submap_half_range);
  m_pgo_config.loop_submap_resolution =
    this->declare_parameter("loop_submap_resolution", m_pgo_config.loop_submap_resolution);
  m_pgo_config.loop_search_period =
    this->declare_parameter("loop_search_period", m_pgo_config.loop_search_period);
  m_pgo_config.loop_robust_kernel =
    this->declare_parameter("loop_robust_kernel", m_pgo_config.loop_robust_kernel);
  if (
    m_pgo_config.loop_robust_kernel != "huber" && m_pgo_config.loop_robust_kernel != "cauchy" &&
    m_pgo_config.loop_robust_kernel != "none") {
    RCLCPP_ERROR(
      this->get_logger(),
      "[PGONode] loop_robust_kernel '%s' is not huber / cauchy / none; using none",
      m_pgo_config.loop_robust_kernel.c_str());
    m_pgo_config.loop_robust_kernel = "none";
  }
  m_pgo_config.loop_robust_kernel_scale =
    this->declare_parameter("loop_robust_kernel_scale", m_pgo_config.loop_robust_kernel_scale);
  m_pgo_config.loop_inlier_dist =
    this->declare_parameter("loop_inlier_dist", m_pgo_config.loop_inlier_dist);
  m_pgo_config.loop_min_overlap_ratio =
    this->declare_parameter("loop_min_overlap_ratio", m_pgo_config.loop_min_overlap_ratio);
  m_pgo_config.loop_min_inlier_ratio =
    this->declare_parameter("loop_min_inlier_ratio", m_pgo_config.loop_min_inlier_ratio);
  m_pgo_config.loop_min_xy_info_ratio =
    this->declare_parameter("loop_min_xy_info_ratio", m_pgo_config.loop_min_xy_info_ratio);
  m_pgo_config.loop_max_drift_ratio =
    this->declare_parameter("loop_max_drift_ratio", m_pgo_config.loop_max_drift_ratio);
  m_pgo_config.loop_radius_drift_scaled =
    this->declare_parameter("loop_radius_drift_scaled", m_pgo_config.loop_radius_drift_scaled);
  m_pgo_config.loop_max_candidates =
    this->declare_parameter("loop_max_candidates", m_pgo_config.loop_max_candidates);
  m_pgo_config.loop_sc_enabled =
    this->declare_parameter("loop_sc_enabled", m_pgo_config.loop_sc_enabled);
  m_pgo_config.loop_sc_max_range =
    this->declare_parameter("loop_sc_max_range", m_pgo_config.loop_sc_max_range);
  m_pgo_config.loop_sc_dist_tresh =
    this->declare_parameter("loop_sc_dist_tresh", m_pgo_config.loop_sc_dist_tresh);
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] loop gates: overlap >= %.2f, inlier ratio (< %.2f m) >= %.2f, xy info ratio >= "
    "%.3f%s, correction <= %.1f %% of the path, up to %d radius candidates (reach %s) | robust "
    "kernel %s "
    "(scale %.1f) | scan context %s (range %.1f m, dist <= %.2f)",
    m_pgo_config.loop_min_overlap_ratio, m_pgo_config.loop_inlier_dist,
    m_pgo_config.loop_min_inlier_ratio, m_pgo_config.loop_min_xy_info_ratio,
    m_pgo_config.loop_min_xy_info_ratio > 0.0 ? "" : " (<= 0: off)",
    100.0 * m_pgo_config.loop_max_drift_ratio, m_pgo_config.loop_max_candidates,
    m_pgo_config.loop_radius_drift_scaled ? "drift-scaled" : "loop_search_radius",
    m_pgo_config.loop_robust_kernel.c_str(), m_pgo_config.loop_robust_kernel_scale,
    m_pgo_config.loop_sc_enabled ? "on" : "off", m_pgo_config.loop_sc_max_range,
    m_pgo_config.loop_sc_dist_tresh);

  // Post-save cleaning: passed on clean_map's command line and recorded in the sidecar. Ints:
  // nice, sparse_min_hits, dynamic_min_miss; bools: enabled, clip_below_floor, octomap; the rest
  // doubles.
  m_node_config.map_clean_enabled =
    this->declare_parameter("map_clean_enabled", m_node_config.map_clean_enabled);
  m_node_config.map_clean_nice =
    this->declare_parameter("map_clean_nice", m_node_config.map_clean_nice);
  auto & mc = m_node_config.map_clean;
  mc.voxel_size = this->declare_parameter("map_clean_voxel_size", mc.voxel_size);
  mc.max_range = this->declare_parameter("map_clean_max_range", mc.max_range);
  mc.min_range = this->declare_parameter("map_clean_min_range", mc.min_range);
  mc.lidar_height = this->declare_parameter("map_clean_lidar_height", mc.lidar_height);
  mc.floor_band = this->declare_parameter("map_clean_floor_band", mc.floor_band);
  mc.floor_band_dynamic =
    this->declare_parameter("map_clean_floor_band_dynamic", mc.floor_band_dynamic);
  mc.dynamic_max_height =
    this->declare_parameter("map_clean_dynamic_max_height", mc.dynamic_max_height);
  mc.floor_radius = this->declare_parameter("map_clean_floor_radius", mc.floor_radius);
  mc.sparse_min_hits = this->declare_parameter("map_clean_sparse_min_hits", mc.sparse_min_hits);
  mc.dynamic_miss_ratio =
    this->declare_parameter("map_clean_dynamic_miss_ratio", mc.dynamic_miss_ratio);
  mc.dynamic_min_miss = this->declare_parameter("map_clean_dynamic_min_miss", mc.dynamic_min_miss);
  mc.clip_below_floor = this->declare_parameter("map_clean_clip_below_floor", mc.clip_below_floor);
  mc.octomap = this->declare_parameter("map_clean_octomap", mc.octomap);
  mc.floor_fill_radius =
    this->declare_parameter("map_clean_floor_fill_radius", mc.floor_fill_radius);
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] map cleaning after save: %s | voxel %.3f m, range %.1f-%.1f m, lidar height "
    "%.3f m, floor band %.2f m (dynamic %.2f m), dynamic max height %.1f m, floor radius %.1f m, "
    "sparse min hits "
    "%d, dynamic miss ratio %.2f, dynamic min miss %d, clip below floor %s, octomap %s, floor "
    "fill radius %.1f m, nice %d",
    m_node_config.map_clean_enabled ? "on" : "off", mc.voxel_size, mc.min_range, mc.max_range,
    mc.lidar_height, mc.floor_band, mc.floor_band_dynamic, mc.dynamic_max_height, mc.floor_radius,
    mc.sparse_min_hits, mc.dynamic_miss_ratio, mc.dynamic_min_miss,
    mc.clip_below_floor ? "on" : "off", mc.octomap ? "on" : "off", mc.floor_fill_radius,
    m_node_config.map_clean_nice);

  // No robot_id prefix here: started bare, or the overrides came before the params file.
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode] inputs: %s + %s | TF %s -> %s | LIO reset: %s | map_cloud_dir: %s | "
    "start_on_launch: %s",
    m_node_config.cloud_topic.c_str(), m_node_config.odom_topic.c_str(),
    m_node_config.map_frame.c_str(), m_node_config.local_frame.c_str(),
    m_node_config.lio_reset_service.c_str(), m_node_config.map_cloud_dir.c_str(),
    m_node_config.start_on_launch ? "true" : "false");
}

void PGONode::syncCB(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  // cloud_msg: the scan in the body frame; odom_msg: the body pose in the LIO odom frame.

  PendingPair pair;
  pair.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
  m_state.last_seen_time.store(pair.pose.second);

  // Gate first, so a dropped pair costs nothing. A pair outside MAPPING belongs to no run.
  // IDLE keeps an identity TF alive for the console's live scan; RESETTING stays silent.
  const uint8_t phase = m_state.phase.load();
  if (phase != syncai_common::msg::MappingStatus::MAPPING) {
    if (phase == syncai_common::msg::MappingStatus::IDLE) sendIdentityTF(cloud_msg->header.stamp);
    return;
  }

  // The old run's tail (<= the boundary pointlio reported). One straddling frame would add a
  // BetweenFactor carrying the whole old drift at 1e-6 variance, and the graph never recovers.
  if (pair.pose.second <= m_state.accept_after_time.load()) return;

  if (pair.pose.second < m_state.last_message_time) {
    RCLCPP_WARN(this->get_logger(), "Received out of order message");
    return;
  }

  pair.pose.r = Eigen::Quaterniond(
                  odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)
                  .toRotationMatrix();
  pair.pose.t = V3D(
    odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
  pair.cloud_msg = cloud_msg;

  std::lock_guard<std::mutex> lock(m_state.message_mutex);
  m_state.last_message_time = pair.pose.second;
  m_state.latest_pair = std::move(pair);
}

void PGONode::sendBroadCastTF(builtin_interfaces::msg::Time & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_node_config.map_frame;
  transformStamped.child_frame_id = m_node_config.local_frame;
  transformStamped.header.stamp = time;
  Eigen::Quaterniond q(m_pgo->offsetR());
  V3D t = m_pgo->offsetT();
  transformStamped.transform.translation.x = t.x();
  transformStamped.transform.translation.y = t.y();
  transformStamped.transform.translation.z = t.z();
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  m_tf_broadcaster->sendTransform(transformStamped);
}

// Identity is a fresh optimizer's offset, so the first keyframe after a Start causes no jump. No
// m_pgo access: this runs on the intake thread while a Start may be replacing m_pgo.
void PGONode::sendIdentityTF(const builtin_interfaces::msg::Time & time)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.frame_id = m_node_config.map_frame;
  transformStamped.child_frame_id = m_node_config.local_frame;
  transformStamped.header.stamp = time;
  transformStamped.transform.rotation.w = 1.0;
  m_tf_broadcaster->sendTransform(transformStamped);
}

void PGONode::publishStatus()
{
  syncai_common::msg::MappingStatus msg;
  msg.state = m_state.phase.load();
  msg.key_poses = m_key_pose_count.load();
  msg.loop_closures = m_loop_count.load();
  msg.stamp = this->get_clock()->now();
  m_status_pub->publish(msg);
}

void PGONode::statusTimerCB()
{
  publishStatus();
  // Here because this timer shares saveMapsCB's group, the other writer of m_map_clean_children.
  reapMapCleans();
}

void PGONode::publishLoopMarkers(builtin_interfaces::msg::Time & time)
{
  if (m_loop_marker_pub->get_subscription_count() == 0) return;
  if (m_pgo->historyPairs().size() == 0) return;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker nodes_marker;
  visualization_msgs::msg::Marker edges_marker;
  nodes_marker.header.frame_id = m_node_config.map_frame;
  nodes_marker.header.stamp = time;
  nodes_marker.ns = "pgo_nodes";
  nodes_marker.id = 0;
  nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes_marker.action = visualization_msgs::msg::Marker::ADD;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.scale.x = 0.3;
  nodes_marker.scale.y = 0.3;
  nodes_marker.scale.z = 0.3;
  nodes_marker.color.r = 1.0;
  nodes_marker.color.g = 0.8;
  nodes_marker.color.b = 0.0;
  nodes_marker.color.a = 1.0;

  edges_marker.header.frame_id = m_node_config.map_frame;
  edges_marker.header.stamp = time;
  edges_marker.ns = "pgo_edges";
  edges_marker.id = 1;
  edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges_marker.action = visualization_msgs::msg::Marker::ADD;
  edges_marker.pose.orientation.w = 1.0;
  edges_marker.scale.x = 0.1;
  edges_marker.color.r = 0.0;
  edges_marker.color.g = 0.8;
  edges_marker.color.b = 0.0;
  edges_marker.color.a = 1.0;

  std::vector<KeyPoseWithCloud> & poses = m_pgo->keyPoses();
  std::vector<std::pair<size_t, size_t>> & pairs = m_pgo->historyPairs();
  for (size_t i = 0; i < pairs.size(); i++) {
    size_t i1 = pairs[i].first;
    size_t i2 = pairs[i].second;
    geometry_msgs::msg::Point p1, p2;
    p1.x = poses[i1].t_global.x();
    p1.y = poses[i1].t_global.y();
    p1.z = poses[i1].t_global.z();

    p2.x = poses[i2].t_global.x();
    p2.y = poses[i2].t_global.y();
    p2.z = poses[i2].t_global.z();

    nodes_marker.points.push_back(p1);
    nodes_marker.points.push_back(p2);
    edges_marker.points.push_back(p1);
    edges_marker.points.push_back(p2);
  }

  marker_array.markers.push_back(nodes_marker);
  marker_array.markers.push_back(edges_marker);
  m_loop_marker_pub->publish(marker_array);
}

void PGONode::timerCB()
{
  // Whole body; see the m_pgo_mutex declaration.
  std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

  PendingPair pair;
  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    if (!m_state.latest_pair) return;
    pair = std::move(*m_state.latest_pair);
    m_state.latest_pair.reset();
  }

  builtin_interfaces::msg::Time cur_time;
  cur_time.sec = pair.pose.sec;
  cur_time.nanosec = pair.pose.nsec;

  // Not a keyframe: broadcast the TF only; the cloud is never decoded.
  if (!m_pgo->isKeyPose(pair.pose)) {
    sendBroadCastTF(cur_time);
    return;
  }

  CloudWithPose cp;
  cp.pose = pair.pose;
  cp.cloud = CloudType::Ptr(new CloudType);
  pcl::fromROSMsg(*pair.cloud_msg, *cp.cloud);
  pair.cloud_msg.reset();
  // Re-runs isKeyPose(), which cannot change in between: we hold m_pgo_mutex.
  if (m_pgo->addKeyPose(cp) && m_node_config.map_cloud_resolution > 0) {
    // The live merge's input, made here once rather than per merge (see preview_cloud).
    const double res = m_node_config.map_cloud_resolution;
    CloudType::Ptr preview(new CloudType);
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(res, res, res);
    voxel_filter.setInputCloud(cp.cloud);
    voxel_filter.filter(*preview);
    m_pgo->keyPoses().back().preview_cloud = preview;
  }

  m_pgo->searchForLoopPairs();

  // Every verified candidate, accepted or not: what the loop gates are calibrated from.
  for (const LoopCandidateReport & c : m_pgo->lastCandidates()) {
    char sc[32] = "";
    if (c.scan_context) std::snprintf(sc, sizeof(sc), " (sc dist %.3f)", c.sc_dist);
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][loop] candidate %zu -> %zu via %s%s | overlap %.2f inlier %.2f fitness %.4f "
      "xy info %.3f correction %.2f / %.2f m | %s%s",
      c.target_id, c.source_id,
      c.scan_context ? "scan context" : (c.coarse ? "radius (coarse)" : "radius"), sc,
      c.overlap_ratio, c.inlier_ratio, c.fitness, c.xy_info_ratio, c.correction_m, c.allowed_m,
      c.accepted ? "passed" : "rejected: ", c.reject_reason);
  }

  // One line per accepted loop, BEFORE the graph consumes it: the only record of what a closure
  // asked for. z is printed separately (before and, below, after the update) because loops lifting
  // the map into a double floor is a failure that has happened; see README, "Loop verification".
  const std::vector<LoopPair> & loops = m_pgo->pendingLoops();
  std::vector<double> src_z_before;
  src_z_before.reserve(loops.size());
  for (const LoopPair & lp : loops) {
    const KeyPoseWithCloud & tgt = m_pgo->keyPoses()[lp.target_id];
    const KeyPoseWithCloud & src = m_pgo->keyPoses()[lp.source_id];
    src_z_before.push_back(src.t_global.z());
    const double rot_deg = Eigen::AngleAxisd(lp.gicp_r).angle() * 180.0 / M_PI;
    // The source keyframe's displacement, not the raw t: the rotation is about the world origin.
    const V3D moved = lp.gicp_r * src.t_global + lp.gicp_t - src.t_global;
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][loop] target %zu -> source %zu | fitness %.4f | GICP moves source by "
      "(%.3f, %.3f, %.3f) m, rot %.2f deg | z before: target %.3f source %.3f (source-target "
      "%+.3f)",
      lp.target_id, lp.source_id, lp.score, moved.x(), moved.y(), moved.z(), rot_deg,
      tgt.t_global.z(), src.t_global.z(), src.t_global.z() - tgt.t_global.z());
  }
  // The ids survive the update (they index m_key_poses); the pairs do not.
  std::vector<std::pair<size_t, size_t>> loop_ids;
  for (const LoopPair & lp : loops) loop_ids.emplace_back(lp.target_id, lp.source_id);

  m_pgo->smoothAndUpdate();
  m_key_pose_count.store(static_cast<uint32_t>(m_pgo->keyPoses().size()));
  m_loop_count.store(static_cast<uint32_t>(m_pgo->historyPairs().size()));

  // The same keyframes after the update: a source z moving AWAY from the target's is the bug.
  for (size_t i = 0; i < loop_ids.size(); i++) {
    const KeyPoseWithCloud & tgt = m_pgo->keyPoses()[loop_ids[i].first];
    const KeyPoseWithCloud & src = m_pgo->keyPoses()[loop_ids[i].second];
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][loop] target %zu -> source %zu | z after: target %.3f source %.3f "
      "(source-target %+.3f, source moved %+.3f) | %d iSAM2 steps",
      loop_ids[i].first, loop_ids[i].second, tgt.t_global.z(), src.t_global.z(),
      src.t_global.z() - tgt.t_global.z(), src.t_global.z() - src_z_before[i],
      m_pgo->lastLoopIterations());
  }

  sendBroadCastTF(cur_time);

  publishLoopMarkers(cur_time);

  // Only a keyframe tick gets here, which is exactly when the merged map changed.
  publishMapCloud(cur_time);
}

void PGONode::publishMapCloud(builtin_interfaces::msg::Time & time)
{
  // With nobody on either output (backend: file notice; rviz: PointCloud2) the merge is waste.
  const bool want_msg = m_map_cloud_pub->get_subscription_count() > 0;
  const bool want_file = m_map_cloud_dir_ok && m_map_cloud_file_pub->get_subscription_count() > 0;
  if (!want_msg && !want_file) return;
  if (m_pgo->keyPoses().empty()) return;

  // Rate-floored on keyframe (sensor) time, so it also behaves under bag time.
  double now_s = m_pgo->keyPoses().back().time;
  if (now_s - m_last_map_cloud_time < m_node_config.map_cloud_pub_period) return;

  // One worker at a time: a merge still running skips this keyframe, never queues.
  if (m_map_cloud_busy.exchange(true)) return;
  m_last_map_cloud_time = now_s;

  // busy was false, so the previous thread has finished; join() returns at once.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();

  // Snapshot by value under m_pgo_mutex. Cloud Ptrs are shared safely: a keyframe's cloud is
  // never written after insertion. (Not getSubMap(), which reads keyPoses live.)
  std::vector<KeyPoseWithCloud> snapshot = m_pgo->keyPoses();
  const uint64_t seq = ++m_map_cloud_seq;
  m_map_cloud_thread =
    std::thread(&PGONode::mergeAndPublishMapCloud, this, std::move(snapshot), time, seq);
}

// Runs on m_map_cloud_thread: late in a run this takes hundreds of ms, and the timer's TF
// broadcast must not gap for that long.
void PGONode::mergeAndPublishMapCloud(
  std::vector<KeyPoseWithCloud> snapshot, builtin_interfaces::msg::Time time, uint64_t seq)
{
  // Stacked from the per-keyframe previews, already at map_cloud_resolution, and re-voxelised
  // whenever the stack passes kMergeVoxelEvery points: the merge stays the size of the place, not
  // of the run. It used to stack every dense keyframe first, a second full copy of the run every
  // map_cloud_pub_period, overlapping a save's. A voxel of voxels is not bit-identical to one
  // pass over the dense clouds (the centroids shift by a fraction of a voxel); it is a preview.
  constexpr size_t kMergeVoxelEvery = 2'000'000;
  const double res = m_node_config.map_cloud_resolution;
  pcl::VoxelGrid<PointType> voxel_filter;
  if (res > 0) voxel_filter.setLeafSize(res, res, res);
  auto voxelize = [&](CloudType::Ptr & cloud) {
    CloudType::Ptr out(new CloudType);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*out);
    cloud = out;
  };

  CloudType::Ptr merged(new CloudType);
  CloudType world_cloud;
  size_t since_voxel = 0;
  for (const auto & kp : snapshot) {
    const CloudType::Ptr & src = kp.preview_cloud ? kp.preview_cloud : kp.body_cloud;
    pcl::transformPointCloud(*src, world_cloud, kp.t_global, Eigen::Quaterniond(kp.r_global));
    *merged += world_cloud;
    since_voxel += world_cloud.size();
    if (res > 0 && since_voxel >= kMergeVoxelEvery) {
      voxelize(merged);
      since_voxel = 0;
    }
  }
  if (res > 0 && since_voxel > 0) voxelize(merged);

  // Already in the map frame. Subscriber counts re-read so a reader that left mid-merge costs
  // nothing.
  if (m_map_cloud_pub->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    msg.header.frame_id = m_node_config.map_frame;
    msg.header.stamp = time;
    m_map_cloud_pub->publish(msg);
  }
  if (m_map_cloud_dir_ok && m_map_cloud_file_pub->get_subscription_count() > 0) {
    writeAndAnnounceMapCloud(*merged, seq, time);
  }

  // Last: this lets publishMapCloud spawn the next worker.
  m_map_cloud_busy.store(false);
}

// Tell every consumer the map is gone. UNGATED, unlike the normal path: it is tiny, and a late
// subscriber must not keep showing the discarded map.
//
// Caller (stopRunLocked) holds m_pgo_mutex and has joined the worker, so no .tmp is mid-write.
// The empty notice goes out BEFORE the files are removed and replaces the latched one.
void PGONode::publishEmptyMapCloud(const builtin_interfaces::msg::Time & time)
{
  CloudType empty;
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(empty, msg);
  msg.header.frame_id = m_node_config.map_frame;
  msg.header.stamp = time;
  m_map_cloud_pub->publish(msg);

  publishMapCloudNotice(++m_map_cloud_seq, "", 0, time);
  clearMapCloudDir();
}

void PGONode::publishLoopMarkerDeleteAll()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);
  m_loop_marker_pub->publish(marker_array);
}

void PGONode::stopRunLocked(const builtin_interfaces::msg::Time & now)
{
  // Not for memory safety (the worker has its own snapshot): an in-flight merge must not publish
  // the OLD map after the empty one.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();
  m_map_cloud_busy.store(false);
  m_last_map_cloud_time = 0.0;

  // The constructor IS the reset (gtsam::ISAM2 has no clear, so a reset() method would be a second
  // definition of "empty"). After a save this also frees the keyframe clouds.
  m_pgo = std::make_shared<PoseGraphOptimizer>(m_pgo_config);

  {
    std::lock_guard<std::mutex> lock(m_state.message_mutex);
    // A pair from the old run must not become the new run's first keyframe.
    m_state.latest_pair.reset();
    m_state.last_message_time = -1.0;
  }
  m_key_pose_count.store(0);
  m_loop_count.store(0);

  publishEmptyMapCloud(now);
  publishLoopMarkerDeleteAll();
}

// Ordered so the only fallible step runs before anything is destroyed; every failure exits
// through the guard that restores the previous phase, so there is no half-reset.
bool PGONode::beginRun(
  bool reset_lio, uint8_t required_phase, std::string & message, double & last_odom_time,
  uint32_t & dropped)
{
  using syncai_common::msg::MappingStatus;
  const bool is_start = required_phase == MappingStatus::IDLE;
  const char * tag = is_start ? "startMappingCB" : "resetMappingCB";
  const char * kept = is_start ? "nothing started" : "map kept";
  last_odom_time = 0.0;
  dropped = 0;

  // ---- Phase 1: pause. ----
  //
  // RESETTING keeps the odometry discontinuity out of the graph: no sleep or slack window needed.
  if (m_transitioning.exchange(true)) {
    message = "A start or reset is already running";
    return false;
  }
  // Every early return (or exception) restores the phase and clears the flag. `armed`: paused, as
  // opposed to refused before anything changed.
  struct ResumeGuard
  {
    PGONode * self;
    uint8_t prev = MappingStatus::IDLE;
    bool armed = false;
    bool committed = false;
    ~ResumeGuard()
    {
      if (armed && !committed) {
        self->m_state.phase.store(prev);
        self->publishStatus();
      }
      self->m_transitioning.store(false);
    }
  } resume_guard{this};

  {
    // saveMapsCB stores IDLE under this mutex, so a racing reset either waits behind the save
    // or sees IDLE and is refused -- never pauses the run the save just ended.
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);
    resume_guard.prev = m_state.phase.load();
    if (resume_guard.prev != required_phase) {
      message = is_start
                  ? "Already mapping: use reset_mapping to start over, or save_maps to finish."
                  : "Not mapping (idle): nothing to discard. Call start_mapping to begin a run.";
      return false;
    }
    m_state.phase.store(MappingStatus::RESETTING);
    resume_guard.armed = true;
  }
  publishStatus();

  // ---- Phase 2: reset the front end. The only fallible step. ----
  //
  // Outside m_pgo_mutex: it blocks for seconds, which would stall the timer's TF broadcast.
  if (reset_lio) {
    if (!m_lio_reset_cli->wait_for_service(std::chrono::seconds(2))) {
      message =
        "LIO reset service " + m_node_config.lio_reset_service + " is not available; " + kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }

    auto future = m_lio_reset_cli->async_send_request(
      std::make_shared<syncai_common::srv::ResetLIO::Request>());
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      message = std::string("Timed out waiting for the LIO reset; ") + kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }

    auto lio_response = future.get();
    if (!lio_response->success) {
      message = "LIO refused the reset (" + lio_response->message + "); " + kept;
      RCLCPP_ERROR(this->get_logger(), "[PGONode][%s] %s", tag, message.c_str());
      return false;
    }
    last_odom_time = lio_response->last_odom_time;
  }

  // ---- Phase 3: rebuild the graph. Nothing below can fail. ----
  builtin_interfaces::msg::Time now = this->get_clock()->now();
  {
    std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);
    dropped = static_cast<uint32_t>(m_pgo->keyPoses().size());
    stopRunLocked(now);
    // The new run's boundary: pointlio's reported stamp with a LIO reset; otherwise the last
    // pair seen for a start, and 0.0 for a reset (held pairs pass, as replays expect).
    m_state.accept_after_time.store(
      reset_lio ? last_odom_time : (is_start ? m_state.last_seen_time.load() : 0.0));
    m_state.phase.store(MappingStatus::MAPPING);
    resume_guard.committed = true;
  }

  // ---- Phase 4: resume (the guard clears m_transitioning). ----
  publishStatus();
  return true;
}

// IDLE -> MAPPING; see StartMapping.srv for the contract.
void PGONode::startMappingCB(
  const std::shared_ptr<syncai_common::srv::StartMapping::Request> request,
  std::shared_ptr<syncai_common::srv::StartMapping::Response> response)
{
  std::string message;
  double last_odom_time = 0.0;
  uint32_t dropped = 0;
  const bool ok = beginRun(
    request->reset_lio, syncai_common::msg::MappingStatus::IDLE, message, last_odom_time, dropped);
  response->success = ok;
  response->lio_last_odom_time = last_odom_time;
  if (!ok) {
    response->message = message;
    return;
  }
  // UI copy, rendered verbatim by the console, and the last chance for the stillness warning:
  // the static IMU init runs in the seconds after this returns.
  response->message =
    request->reset_lio
      ? "Mapping started. The map begins building once the lidar has re-levelled — keep "
        "the robot still until then."
      : "Mapping started over the running odometry; the map origin is where the LIO started.";
  RCLCPP_WARN(this->get_logger(), "[PGONode][startMappingCB] %s", response->message.c_str());
}

// Discard the graph and start a new map in place: MAPPING -> MAPPING. Refused while IDLE, so a
// no-op never resets the LIO as a side effect.
void PGONode::resetMappingCB(
  const std::shared_ptr<syncai_common::srv::ResetMapping::Request> request,
  std::shared_ptr<syncai_common::srv::ResetMapping::Response> response)
{
  std::string message;
  double last_odom_time = 0.0;
  uint32_t dropped = 0;
  const bool ok = beginRun(
    request->reset_lio, syncai_common::msg::MappingStatus::MAPPING, message, last_odom_time,
    dropped);
  response->success = ok;
  response->lio_last_odom_time = last_odom_time;
  response->dropped_key_poses = dropped;
  if (!ok) {
    response->message = message;
    return;
  }
  // UI copy, as in startMappingCB.
  response->message = request->reset_lio
                        ? "Map discarded. The new one starts building once the "
                          "lidar has re-levelled — keep the robot still until then."
                        : "Pose graph reset. The LIO front end was left running.";
  RCLCPP_WARN(
    this->get_logger(), "[PGONode][resetMappingCB] %s (dropped %u key poses)",
    response->message.c_str(), dropped);
}

// Serialise the run to disk and END it: MAPPING -> IDLE (keyframes freed, empty map published,
// /dev/shm cleared). A failed save changes nothing.
void PGONode::saveMapsCB(
  const std::shared_ptr<syncai_common::srv::SaveMaps::Request> request,
  std::shared_ptr<syncai_common::srv::SaveMaps::Response> response)
{
  using syncai_common::msg::MappingStatus;
  // Whole body: a reset must wait behind the save rather than interleave with it (it then finds
  // IDLE and is refused).
  std::lock_guard<std::mutex> pgo_lock(m_pgo_mutex);

  // RESETTING is observable here: beginRun's phase 2 runs without the mutex.
  const uint8_t phase = m_state.phase.load();
  if (phase != MappingStatus::MAPPING) {
    response->success = false;
    response->message =
      phase == MappingStatus::IDLE
        ? "Not mapping (idle): nothing to save. Call start_mapping to begin a run."
        : "A start or reset is in progress; try again in a moment.";
    return;
  }

  if (!std::filesystem::exists(request->file_path)) {
    response->success = false;
    response->message = request->file_path + " IS NOT EXISTS!";
    return;
  }

  if (m_pgo->keyPoses().size() == 0) {
    response->success = false;
    response->message = "NO POSES!";
    return;
  }

  // A live merge still running holds its own copy of the run's clouds; let it finish before the
  // save adds to the peak. No new one can start: the timer that spawns them needs m_pgo_mutex.
  if (m_map_cloud_thread.joinable()) m_map_cloud_thread.join();

  std::filesystem::path p_dir(request->file_path);
  std::filesystem::path patches_dir = p_dir / "patches";       // per-keyframe patch clouds
  std::filesystem::path poses_txt_path = p_dir / "poses.txt";  // per-frame pose list
  std::filesystem::path map_path = p_dir / "map.pcd";          // the merged full map

  // pcl throws on a write failure; a throw out of a service callback would take the node down.
  // Caught, reported, and the run kept.
  try {
    if (request->save_patches) {
      if (std::filesystem::exists(patches_dir)) {
        std::filesystem::remove_all(patches_dir);
      }

      std::filesystem::create_directories(patches_dir);

      if (std::filesystem::exists(poses_txt_path)) {
        std::filesystem::remove(poses_txt_path);
      }
      // A clean of the previous save here may still be running; its input is being replaced.
      stopMapCleanFor(p_dir);
      removeMapCleanOutputs(p_dir);
      RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
    }
    RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

    if (request->save_patches) {
      // Opening it truncates an existing poses.txt.
      std::ofstream txt_file(poses_txt_path);
      for (size_t i = 0; i < m_pgo->keyPoses().size(); i++) {
        std::string patch_name = std::to_string(i) + ".pcd";
        std::filesystem::path patch_path = patches_dir / patch_name;
        pcl::io::savePCDFileBinary(patch_path.string(), *m_pgo->keyPoses()[i].body_cloud);
        Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
        V3D t = m_pgo->keyPoses()[i].t_global;
        txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w()
                 << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
      }
    }

    // Streamed, never merged in memory: see map_writer.h. Atomic, so a failed save leaves the
    // previous map.pcd whole rather than truncated.
    writeMergedMapPcd(map_path, m_pgo->keyPoses());
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string("Save failed: ") + e.what() + "; the run is kept.";
    RCLCPP_ERROR(this->get_logger(), "[PGONode][saveMapsCB] %s", response->message.c_str());
    return;
  }

  // ---- The run is on disk: end it. ----
  const size_t saved = m_pgo->keyPoses().size();
  stopRunLocked(this->get_clock()->now());
  m_state.phase.store(MappingStatus::IDLE);
  publishStatus();

  response->success = true;
  // After the run has ended, so the keyframes' RAM is back before the clean starts; a clean that
  // cannot start is reported in the message and the sidecar, never as a failed save. The message
  // is UI copy, rendered verbatim by the console.
  const std::string clean_note = startMapClean(p_dir, request->save_patches);
  response->message = "Map saved (" + std::to_string(saved) +
                      " keyframes). Mapping stopped — start it again for another map." + clean_note;
  RCLCPP_WARN(
    this->get_logger(), "[PGONode][saveMapsCB] %s (%s)", response->message.c_str(),
    map_path.string().c_str());
}

// ---- The post-save map cleaning ---------------------------------------------
//
// clean_map rewrites map.pcd without people and one-off returns (see README, "Cleaning map.pcd").
// A child process, not a thread:
//   - Memory: its voxel hash map (hundreds of MB) goes back to the OS at exit, and its
//     oom_score_adj 500 makes it, not this node, the OOM victim.
//   - Lifetime: a mode switch SIGHUPs this pane right after a save; POSIX_SPAWN_SETSID gives the
//     clean its own session so it survives.
//   - One entry point: the same executable is the by-hand re-clean.
// posix_spawn, not fork(): a fork()ed multithreaded process may only call async-signal-safe
// functions before exec.
std::string PGONode::startMapClean(const std::filesystem::path & map_dir, bool have_patches)
{
  if (!m_node_config.map_clean_enabled) return "";
  // UI copy, appended to the save's message.
  if (!have_patches) {
    RCLCPP_INFO(
      this->get_logger(),
      "[PGONode][map_clean] save_patches was false: nothing to ray-cast, %s stays uncleaned",
      map_dir.c_str());
    return " People were not removed from this map: that needs the per-scan data this save "
           "did not keep.";
  }
  const auto & oc = m_node_config.map_clean;
  const std::string started_at = dynamic_removal::recipe::isoUtcNow();
  // Before the spawn: the sidecar must say "converting" by the time the backend reads it.
  try {
    dynamic_removal::recipe::writeAtomic(
      map_dir, dynamic_removal::recipe::converting(oc, started_at));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "[PGONode][map_clean] %s in %s; no clean", e.what(), map_dir.c_str());
    return " Removing people from the map could not be started; the saved map is unaffected.";
  }

  std::string exe;
  try {
    exe = ament_index_cpp::get_package_prefix("syncai_mapping") + "/lib/syncai_mapping/clean_map";
  } catch (const ament_index_cpp::PackageNotFoundError &) {
    return failMapClean(map_dir, started_at, "clean_map executable not found");
  }
  if (access(exe.c_str(), X_OK) != 0) {
    return failMapClean(map_dir, started_at, "clean_map executable not found");
  }

  auto num = [](double v) {
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os << std::setprecision(12) << v;
    return os.str();
  };
  std::vector<std::string> args = {
    exe,
    map_dir.string(),
    "--voxel-size",
    num(oc.voxel_size),
    "--max-range",
    num(oc.max_range),
    "--min-range",
    num(oc.min_range),
    "--lidar-height",
    num(oc.lidar_height),
    "--floor-band",
    num(oc.floor_band),
    "--floor-band-dynamic",
    num(oc.floor_band_dynamic),
    "--dynamic-max-height",
    num(oc.dynamic_max_height),
    "--floor-radius",
    num(oc.floor_radius),
    "--sparse-min-hits",
    std::to_string(oc.sparse_min_hits),
    "--dynamic-miss-ratio",
    num(oc.dynamic_miss_ratio),
    "--dynamic-min-miss",
    std::to_string(oc.dynamic_min_miss),
    "--clip-below-floor",
    oc.clip_below_floor ? "1" : "0",
    "--octomap",
    oc.octomap ? "1" : "0",
    "--floor-fill-radius",
    num(oc.floor_fill_radius),
    "--nice",
    std::to_string(m_node_config.map_clean_nice),
    "--started-at",
    started_at};
  std::vector<char *> argv;
  for (auto & a : args) argv.push_back(a.data());
  argv.push_back(nullptr);

  // SETSIGDEF also resets signals IGNORED here (exec keeps those); SETSIGMASK clears this
  // thread's blocked set.
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  sigset_t defaults, none;
  sigemptyset(&defaults);
  for (const int sig : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGPIPE, SIGCHLD}) {
    sigaddset(&defaults, sig);
  }
  sigemptyset(&none);
  posix_spawnattr_setsigdefault(&attr, &defaults);
  posix_spawnattr_setsigmask(&attr, &none);
  posix_spawnattr_setflags(
    &attr, POSIX_SPAWN_SETSID | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
  pid_t pid = 0;
  // stdout / stderr inherited: progress lands in this pane's log while the pane lives.
  const int rc = posix_spawn(&pid, exe.c_str(), nullptr, &attr, argv.data(), environ);
  posix_spawnattr_destroy(&attr);
  if (rc != 0) {
    return failMapClean(
      map_dir, started_at, std::string("cannot start clean_map: ") + std::strerror(rc));
  }
  m_map_clean_children.push_back({pid, map_dir, std::chrono::steady_clock::now()});
  RCLCPP_INFO(
    this->get_logger(),
    "[PGONode][map_clean] clean_map pid %d started for %s (voxel %.3f m, nice %d); outcome in %s",
    pid, map_dir.c_str(), oc.voxel_size, m_node_config.map_clean_nice,
    dynamic_removal::recipe::kSidecarFile);
  return " People and stray points are being removed from the map in the background; it can "
         "take a few minutes.";
}

std::string PGONode::failMapClean(
  const std::filesystem::path & map_dir, const std::string & started_at, const std::string & error)
{
  RCLCPP_ERROR(
    this->get_logger(), "[PGONode][map_clean] %s (for %s)", error.c_str(), map_dir.c_str());
  try {
    dynamic_removal::recipe::writeAtomic(
      map_dir, dynamic_removal::recipe::failed(
                 m_node_config.map_clean, started_at, dynamic_removal::recipe::isoUtcNow(), error));
  } catch (const std::exception & e) {
    // The "converting" written a moment ago stays; a reader ages it out.
    RCLCPP_ERROR(this->get_logger(), "[PGONode][map_clean] %s", e.what());
  }
  return " Removing people from the map could not be started; the saved map is unaffected.";
}

void PGONode::reapMapCleans()
{
  for (auto it = m_map_clean_children.begin(); it != m_map_clean_children.end();) {
    int st = 0;
    const pid_t r = waitpid(it->pid, &st, WNOHANG);
    if (r == 0) {
      ++it;
      continue;
    }
    const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - it->started).count();
    if (r < 0) {
      // ECHILD: something set SIGCHLD to SIG_IGN here. The sidecar still has the outcome.
      RCLCPP_WARN(
        this->get_logger(),
        "[PGONode][map_clean] cannot wait for clean_map pid %d (%s); see %s in %s", it->pid,
        std::strerror(errno), dynamic_removal::recipe::kSidecarFile, it->dir.c_str());
    } else if (WIFEXITED(st) && WEXITSTATUS(st) == 0) {
      RCLCPP_INFO(
        this->get_logger(), "[PGONode][map_clean] clean_map pid %d finished %s in %.0f s", it->pid,
        it->dir.c_str(), secs);
    } else if (WIFEXITED(st)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[PGONode][map_clean] clean_map pid %d exited %d after %.0f s; %s in %s has the reason",
        it->pid, WEXITSTATUS(st), secs, dynamic_removal::recipe::kSidecarFile, it->dir.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[PGONode][map_clean] clean_map pid %d killed by signal %d after %.0f s (OOM?); %s in "
        "%s is left at converting",
        it->pid, WIFSIGNALED(st) ? WTERMSIG(st) : 0, secs, dynamic_removal::recipe::kSidecarFile,
        it->dir.c_str());
    }
    it = m_map_clean_children.erase(it);
  }
}

void PGONode::stopMapCleanFor(const std::filesystem::path & map_dir)
{
  std::error_code ec;
  const auto target = std::filesystem::weakly_canonical(map_dir, ec);
  for (const auto & child : m_map_clean_children) {
    std::error_code cec;
    if (std::filesystem::weakly_canonical(child.dir, cec) != target) continue;
    // Not yet reaped, so the pid cannot have been recycled.
    if (kill(child.pid, SIGTERM) == 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "[PGONode][map_clean] a new save into %s: stopping clean_map pid %d, which was "
        "cleaning the previous one",
        map_dir.c_str(), child.pid);
    }
  }
}

void PGONode::removeMapCleanOutputs(const std::filesystem::path & map_dir)
{
  std::error_code ec;
  std::filesystem::remove(map_dir / dynamic_removal::recipe::kSidecarFile, ec);
  // A killed clean's half-written map.pcd (map.pcd itself is rewritten by the save, through its
  // own map.pcd.save.tmp).
  std::filesystem::remove(map_dir / (std::string(dynamic_removal::kMapFile) + ".tmp"), ec);
  // An old octomap.bt would describe the previous run, and so would the copy
  // apply_octomap_edits set aside before patching it. octomap_edits.json
  // stays, like keepout.json: hand-marked, and the operator's to redraw.
  std::filesystem::remove(map_dir / dynamic_removal::kOctomapFile, ec);
  std::filesystem::remove(map_dir / (std::string(dynamic_removal::kOctomapFile) + ".tmp"), ec);
  std::filesystem::remove(map_dir / dynamic_removal::edits::kRawOctomapFile, ec);
  // The sidecar's per-process `map_clean.recipe.json.<pid>.tmp`.
  const std::string prefix = std::string(dynamic_removal::recipe::kSidecarFile) + ".";
  for (const auto & entry : std::filesystem::directory_iterator(map_dir, ec)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0 && entry.path().extension() == ".tmp") {
      std::error_code rm_ec;
      std::filesystem::remove(entry.path(), rm_ec);
    }
  }
}

}  // namespace syncai_mapping
