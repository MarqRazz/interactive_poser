// Copyright (c) 2026 Marq Rasmussen. BSD 3-Clause.
#pragma once

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <interactive_poser/interactive_poser_parameters.hpp>

namespace interactive_poser
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

/**
 * @brief Interactively measures the as-built offset of one fixed joint.
 *
 * Edits one calibration joint at a time, chosen from the file named by the
 * `calibration_file` parameter. Parent, child and joint type come from
 * `/robot_description`; a joint that is not fixed, or that sits too far from its
 * nominal to be build error, is refused at load.
 *
 * Offsets are always relative to the target's nominal, so the parameters and the
 * marker read as error rather than as absolute geometry. The live TF tree is
 * never modified: the target's subtree is mirrored onto `shadow_suffix` frames
 * and sensor data is relayed into them.
 *
 * See the package README for the calibration workflow and the conventions this
 * relies on.
 */
class InteractivePoser
{
public:
  explicit InteractivePoser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  /// A joint the file offers and the URDF confirms is fixed.
  struct Target
  {
    std::string joint;
    std::string parent;
    std::string child;
    std::vector<std::string> cloud_topics;
    std::string image_topic;        ///< optional; drives the RViz Camera overlay
    std::string camera_info_topic;  ///< required alongside image_topic
    tf2::Transform from_file{tf2::Transform::getIdentity()};  ///< as read at startup
    /// Where zero error sits for this joint. Identity under the mount-joint
    /// convention, and whatever the file's `nominal` block says otherwise, so a
    /// joint that carries its own layout geometry is still calibratable.
    tf2::Transform nominal{tf2::Transform::getIdentity()};
    bool nominal_declared{false};
    /// What the description loaded for this joint, sampled from TF the first
    /// time it is activated. "Revert" returns here.
    tf2::Transform loaded{tf2::Transform::getIdentity()};
    bool loaded_valid{false};
    /// The operator's in-progress measurement, held across target switches.
    /// Without this, coming back to a joint would re-read TF and silently
    /// discard the work, because TF still shows what the URDF loaded.
    tf2::Transform edited{tf2::Transform::getIdentity()};
    bool has_edit{false};
    bool hasSensor() const { return !cloud_topics.empty() || !image_topic.empty(); }
  };

  /// One relayed sensor topic and the shadow frame its messages are rewritten into.
  struct Relay
  {
    std::string topic;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub;
    PointCloud2::ConstSharedPtr latest;
    PointCloud2 snapshot;  ///< frozen copy, already reframed
    PointCloud2 preview;   ///< decimated copy, sent while the offset moves
    bool has_snapshot{false};
  };

  // --- startup ------------------------------------------------------------
  void tryInitialise();                 ///< retried until the description arrives
  bool loadTargets(std::string & error);
  bool selectTarget(const std::string & joint);
  void activateTarget(std::size_t index);
  void teardownTarget();

  // --- shadow tree and relays --------------------------------------------
  bool seedOffsetFromTf();
  void publishShadowTransforms();
  void setupPublishers();
  void setupRelays();
  void onCloud(std::size_t index, const PointCloud2::ConstSharedPtr & msg);
  /// Ensures a shadow transform exists for a frame seen on incoming data. The
  /// frame is learnt from the message rather than assumed, because a colour
  /// cloud may ride the colour optical frame and guessing "depth" would bridge
  /// a frame no message ever carries.
  void bridgeFrame(const std::string & frame);
  /// Draws the target's own visual geometry in the shadow frame. For a fixture
  /// with no sensor this is the only thing that moves when the marker moves.
  void publishGhost();
  bool captureRelay(Relay & relay);
  /// Re-derives every drag preview from the frozen snapshots. Needed because
  /// stream_decimation would otherwise only take effect at the next capture.
  void rebuildPreviews();
  void captureSnapshots();
  void republishSnapshots(bool preview);
  /// Preview now if the pose moved enough, and schedule the full cloud once quiet.
  void refreshAfterOffsetChange();
  static PointCloud2 decimate(const PointCloud2 & src, int stride);

  // --- marker and parameters ---------------------------------------------
  void setupMarker();
  void syncMarkerPose();
  void onFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & fb);
  void onPreSetParameters(std::vector<rclcpp::Parameter> & to_set);
  void onParameterChange(const std::vector<rclcpp::Parameter> & changed);
  void applyOffset(const tf2::Transform & offset, bool write_params, bool refresh_marker);
  tf2::Transform offsetFromParams(const Params & p) const;
  void writeOffsetToParams(const tf2::Transform & offset);
  /// Clamps to the validator bounds AND quantises to the display resolution
  /// (0.01 mm / 0.01 deg). Applied to the stored transform, not just to the
  /// parameters: if the two representations differ, rqt shows one number while
  /// the file records another.
  static tf2::Transform normaliseOffset(const tf2::Transform & t, const tf2::Transform & nominal);
  /// The editable quantity: how far the joint sits from its nominal.
  tf2::Transform deltaFromNominal() const;

  // --- persistence --------------------------------------------------------
  void setupServices();
  bool saveCalibration(std::string & message);
  /// The live pose of a target: the value being edited for the active one,
  /// whatever the description loaded for the rest.
  bool currentOffsetOf(const Target & t, tf2::Transform & out) const;
  /// Warns when the running cell is not the file the tool was pointed at --
  /// almost always a calibration_file the cell launch never received.
  void warnIfDescriptionDisagrees() const;
  void logOffset(const char * prefix) const;

  std::string shadowOf(const std::string & frame) const;
  const Target & target() const { return targets_.at(active_); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Owned, not stack-local: the prototype's listener died with its constructor,
  // which silently stopped the buffer being fed.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  std::string robot_description_;

  std::vector<Target> targets_;
  std::size_t active_{0};
  bool initialised_{false};

  tf2::Transform offset_{tf2::Transform::getIdentity()};
  tf2::Transform offset_at_activate_{tf2::Transform::getIdentity()};
  tf2::Transform last_published_{tf2::Transform::getIdentity()};
  std::vector<Relay> relays_;
  /// Outputs live at fixed names and outlive a target switch:
  /// /interactive_poser/{snapshot,image,camera_info,ghost}.
  std::vector<rclcpp::Publisher<PointCloud2>::SharedPtr> cloud_pubs_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ghost_pub_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  Image::ConstSharedPtr latest_image_;
  Image image_snapshot_;
  bool has_image_{false};
  std::set<std::string> bridged_;

  /// Guards the marker <-> parameter binding so an update from one source does
  /// not bounce back through the other.
  bool applying_{false};
  std::chrono::steady_clock::time_point last_preview_{};

  using Trigger = std_srvs::srv::Trigger;
  rclcpp::Service<Trigger>::SharedPtr capture_srv_, save_srv_, zero_srv_, revert_srv_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr settle_timer_;
  rclcpp::TimerBase::SharedPtr image_heartbeat_;
  rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr pre_param_cb_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  static constexpr const char * kMarkerName = "as_built_offset";
};
}  // namespace interactive_poser
