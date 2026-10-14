#ifndef SYNCAI_MAPPING__PGO_NODE_HPP_
#define SYNCAI_MAPPING__PGO_NODE_HPP_

// The ROS shell around the pose graph (pgos/pose_graph_optimizer.*): cloud+odom intake, the 50 ms
// timer that feeds the graph and broadcasts map -> local_frame, the "map so far" merge worker and
// its file hand-off, and the run services start_mapping / reset_mapping / save_maps, with the run
// state latched on mapping_status.
//
// The ROS surface (node pgo_node at /<robot_id>/pgo, its services, topics, TF, the
// /dev/shm/syncai_pgo/<robot_id> hand-off and the save_maps layout) is a contract with the
// Workspace's session specs and the backend.

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "syncai_common/msg/mapping_status.hpp"
#include "syncai_common/srv/reset_lio.hpp"
#include "syncai_common/srv/reset_mapping.hpp"
#include "syncai_common/srv/save_maps.hpp"
#include "syncai_common/srv/start_mapping.hpp"
#include "syncai_mapping/dynamic_removal/remover.hpp"
#include "syncai_mapping/pgos/commons.h"
#include "syncai_mapping/pgos/pose_graph_optimizer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace syncai_mapping
{

struct NodeConfig
{
  // Inputs, frames and the LIO reset service: not parameters, derived by loadParameters() from the
  // namespace so they always match syncai_pointlio at /<robot_id>/pointlio.
  std::string cloud_topic;
  std::string odom_topic;
  std::string map_frame = "map";
  std::string local_frame;
  // The "map so far" merge (see publishMapCloud).
  double map_cloud_resolution = 0.2;
  double map_cloud_pub_period = 3.0;
  // File hand-off directory, a tmpfs shared with the backend's container (`ipc: host` on both: a
  // private /dev/shm is 64 MB, too small for two merges of a large site). The launch appends
  // /<robot_id> so two robots on one host never share it.
  std::string map_cloud_dir = "/dev/shm/syncai_pgo";
  std::string lio_reset_service;
  // Come up MAPPING instead of IDLE: the escape hatch for bag replays. False (the mapping
  // session's case): nothing reaches the graph until start_mapping.
  bool start_on_launch = false;
  // Post-save cleaning (see startMapClean). dynamic_removal::Params documents each parameter.
  bool map_clean_enabled = true;
  int map_clean_nice = 10;
  dynamic_removal::Params map_clean;
};

// One synchronised cloud+odom pair: pose decoded, cloud still the ROS message, because
// pcl::fromROSMsg copies a whole lidar frame and timerCB does it only for a keyframe.
struct PendingPair
{
  PoseWithTime pose;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;
};

struct NodeState
{
  std::mutex message_mutex;
  // The newest untaken pair. One slot, not a queue: syncCB overwrites it, timerCB moves it out,
  // so a busy timerCB never accumulates decoded frames.
  std::optional<PendingPair> latest_pair;
  // -1.0 = no message seen yet.
  double last_message_time = -1.0;

  // The intake gate. Atomics so syncCB's hot path takes no m_pgo_mutex and there is no lock
  // ordering to get wrong.
  //
  // phase is a MappingStatus constant; only MAPPING lets a pair in. IDLE: before start_mapping
  // and after a successful save_maps. RESETTING: inside start_mapping / reset_mapping while
  // pointlio resets, so nothing it publishes meanwhile can reach the graph. Stored only under
  // m_pgo_mutex; loads need no lock.
  //
  // accept_after_time drops the old run's tail still held by the synchroniser; both sides are
  // lidar header stamps, so the comparison is exact. last_seen_time is the stamp of the last
  // pair seen, accepted or not: the boundary for a start_mapping without reset_lio.
  std::atomic<uint8_t> phase{syncai_common::msg::MappingStatus::IDLE};
  std::atomic<double> accept_after_time{-1.0};
  std::atomic<double> last_seen_time{-1.0};
};

class PGONode : public rclcpp::Node
{
public:
  PGONode();
  ~PGONode();

  // ---- The file hand-off (see the map_cloud_file publisher) ----------------
  void setupMapCloudDir();
  static bool isMapCloudFile(const std::filesystem::directory_entry & entry);
  void clearMapCloudDir();
  void pruneMapCloudFiles();
  std::string mapCloudNoticeJson(
    uint64_t seq, const std::string & path, size_t points,
    const builtin_interfaces::msg::Time & time) const;
  void publishMapCloudNotice(
    uint64_t seq, const std::string & path, size_t points,
    const builtin_interfaces::msg::Time & time);
  void writeAndAnnounceMapCloud(
    const CloudType & merged, uint64_t seq, const builtin_interfaces::msg::Time & time);

  // Declares every parameter with the struct default as its default. Read once at startup.
  void loadParameters();

  // Intake: one synchronised cloud+odom pair, gated by the run state.
  void syncCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg);

  void sendBroadCastTF(builtin_interfaces::msg::Time & time);
  // IDLE stand-in for sendBroadCastTF: identity map -> local_frame, no m_pgo access (no lock).
  // Keeps the console's live scan, which needs this TF, visible before Start.
  void sendIdentityTF(const builtin_interfaces::msg::Time & time);
  void publishLoopMarkers(builtin_interfaces::msg::Time & time);

  // Publishes mapping_status. Lock-free (atomics only), so callable from anywhere.
  void publishStatus();
  void statusTimerCB();

  // ---- The post-save map cleaning --------------------------------------------
  // Spawn `clean_map <map_dir>` detached, after the save has ended the run. Returns the sentence
  // appended to the save's message ("" when disabled). Never fails the save.
  std::string startMapClean(const std::filesystem::path & map_dir, bool have_patches);
  // Record a clean that could not start in the sidecar; returns the sentence.
  std::string failMapClean(
    const std::filesystem::path & map_dir, const std::string & started_at,
    const std::string & error);
  // waitpid(WNOHANG) every spawned clean; one log line per outcome. 1 Hz.
  void reapMapCleans();
  // SIGTERM any clean still running on map_dir before a new save replaces its input. Not waited
  // for; clean_map's poses.txt fingerprint keeps it from renaming over the new save.
  void stopMapCleanFor(const std::filesystem::path & map_dir);
  // Remove an earlier save's sidecar, octomap.bt and the clean's .tmp files: their input
  // (patches/, poses.txt) is about to be replaced.
  static void removeMapCleanOutputs(const std::filesystem::path & map_dir);

  // 50 ms timer: keyframe selection, loop search, smoothing, TF, outputs.
  void timerCB();

  // The "map so far" merge: claimed on the timer thread, run on the worker.
  void publishMapCloud(builtin_interfaces::msg::Time & time);
  void mergeAndPublishMapCloud(
    std::vector<KeyPoseWithCloud> snapshot, builtin_interfaces::msg::Time time, uint64_t seq);
  void publishEmptyMapCloud(const builtin_interfaces::msg::Time & time);
  void publishLoopMarkerDeleteAll();

  // start_mapping and reset_mapping share one MutuallyExclusive group (so they serialise) and
  // block on the ResetLIO client; save_maps shares the timer's group.
  void startMappingCB(
    const std::shared_ptr<syncai_common::srv::StartMapping::Request> request,
    std::shared_ptr<syncai_common::srv::StartMapping::Response> response);
  void resetMappingCB(
    const std::shared_ptr<syncai_common::srv::ResetMapping::Request> request,
    std::shared_ptr<syncai_common::srv::ResetMapping::Response> response);
  void saveMapsCB(
    const std::shared_ptr<syncai_common::srv::SaveMaps::Request> request,
    std::shared_ptr<syncai_common::srv::SaveMaps::Response> response);

private:
  // Cleans spawned and not yet reaped. Touched only by saveMapsCB and statusTimerCB (same default
  // MutuallyExclusive group, so no lock) and the destructor, which only logs them.
  struct MapCleanChild
  {
    pid_t pid;
    std::filesystem::path dir;
    std::chrono::steady_clock::time_point started;
  };
  std::vector<MapCleanChild> m_map_clean_children;
  // The sequence start_mapping (required_phase IDLE) and reset_mapping (MAPPING) share: pause,
  // reset the LIO, rebuild the graph, resume. On false, `message` says why and the state is
  // unchanged; on true the node is MAPPING.
  bool beginRun(
    bool reset_lio, uint8_t required_phase, std::string & message, double & last_odom_time,
    uint32_t & dropped);
  // Turn "a run" into "no run"; caller holds m_pgo_mutex. Joins the merge worker, replaces the
  // optimizer, empties the buffer and counters, publishes the empty map (clearing /dev/shm) and
  // deletes the loop markers. Does NOT touch phase: the caller stores IDLE or MAPPING.
  void stopRunLocked(const builtin_interfaces::msg::Time & now);

  NodeConfig m_node_config;
  PoseGraphOptimizerConfig m_pgo_config;
  NodeState m_state;
  std::shared_ptr<PoseGraphOptimizer> m_pgo;
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
  // The merge worker. busy is claimed by publishMapCloud (timer thread) and released by
  // mergeAndPublishMapCloud as its last act; the thread is joined only while busy is false (or in
  // the destructor / stopRunLocked).
  std::atomic<bool> m_map_cloud_busy{false};
  double m_last_map_cloud_time = 0.0;
  std::thread m_map_cloud_thread;
  // The file hand-off. m_map_cloud_seq is touched only under m_pgo_mutex; the worker gets a copy.
  // dir_ok false: the directory could not be created, file output is off (PointCloud2 unaffected).
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_map_cloud_file_pub;
  uint64_t m_map_cloud_seq = 0;
  std::string m_map_cloud_dir;
  bool m_map_cloud_dir_ok = false;
  rclcpp::Service<syncai_common::srv::SaveMaps>::SharedPtr m_save_map_srv;
  // mapping_status and its counters: mirrors of keyPoses().size() / historyPairs().size(), written
  // under m_pgo_mutex so publishStatus never needs the lock.
  rclcpp::Publisher<syncai_common::msg::MappingStatus>::SharedPtr m_status_pub;
  rclcpp::TimerBase::SharedPtr m_status_timer;
  std::atomic<uint32_t> m_key_pose_count{0};
  std::atomic<uint32_t> m_loop_count{0};
  // m_transitioning rejects a concurrent start/reset rather than queueing it, which would read a
  // boundary the first had invalidated. Belt and braces: the handlers already share a
  // MutuallyExclusive group.
  rclcpp::CallbackGroup::SharedPtr m_srv_cb_group;
  rclcpp::CallbackGroup::SharedPtr m_cli_cb_group;
  rclcpp::Service<syncai_common::srv::StartMapping>::SharedPtr m_start_srv;
  rclcpp::Service<syncai_common::srv::ResetMapping>::SharedPtr m_reset_srv;
  rclcpp::Client<syncai_common::srv::ResetLIO>::SharedPtr m_lio_reset_cli;
  std::atomic<bool> m_transitioning{false};
  // Guards m_pgo, which is REPLACED (not mutated) on start/reset/save, so readers must be excluded
  // too. Needed because main() runs a MultiThreadedExecutor. Held by:
  //
  //   timerCB      whole body (graph updates, TF broadcast, merge-worker handshake)
  //   saveMapsCB   whole body (it serialises what a reset would destroy, then -> IDLE)
  //   beginRun     phase 1 (precondition + RESETTING store) and phase 3 -- NEVER while waiting
  //                on the LIO future, which would stall the TF broadcast for seconds
  //
  // Every store to m_state.phase happens under it, which makes the precondition checks exact.
  std::mutex m_pgo_mutex;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>
    m_sync;
};

}  // namespace syncai_mapping

#endif  // SYNCAI_MAPPING__PGO_NODE_HPP_
