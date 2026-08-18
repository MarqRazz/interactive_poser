// Copyright (c) 2026 Marq Rasmussen. BSD 3-Clause.

#include <interactive_poser/interactive_poser.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
constexpr auto kNodeName = "interactive_poser";
constexpr double kMm = 1000.0;
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;
// Must match the bounds<> validators in interactive_poser_parameters.yaml.
constexpr double kMaxShiftM = 1.000;
constexpr double kMaxTiltRad = 15.0 * M_PI / 180.0;
// Display resolution: two decimals in millimetres and degrees.
constexpr double kShiftQuantumM = 0.01 / 1000.0;
constexpr double kTiltQuantumRad = 0.01 * M_PI / 180.0;
// Below this a resend would not be visible, so it is not worth the bandwidth.
constexpr double kVisibleShiftM = 0.0002;
constexpr double kVisibleTiltRad = 0.02 * M_PI / 180.0;

using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;

struct AxisSpec
{
  double x, y, z, w;
  const char * name;
};
constexpr AxisSpec kAxes[] = {
  {1.0, 0.0, 0.0, 1.0, "x"},
  {0.0, 0.0, 1.0, 1.0, "y"},
  {0.0, 1.0, 0.0, 1.0, "z"},
};

/// tf2::toMsg on a Transform yields a Transform; the marker wants a Pose.
geometry_msgs::msg::Pose toPose(const tf2::Transform & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
  p.orientation = tf2::toMsg(t.getRotation());
  return p;
}

std::string utcNow(const char * format)
{
  const auto t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[40];
  std::strftime(buf, sizeof(buf), format, &tm);
  return buf;
}
std::string isoNow() { return utcNow("%Y-%m-%dT%H:%M:%SZ"); }

tf2::Transform fromUrdf(const urdf::Pose & p)
{
  return tf2::Transform(
    tf2::Quaternion(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w),
    tf2::Vector3(p.position.x, p.position.y, p.position.z));
}
/// Colons are legal on Linux but hostile in filenames, so the stamp is compact.
std::string fileStamp() { return utcNow("%Y%m%dT%H%M%SZ"); }
}  // namespace

namespace interactive_poser
{
InteractivePoser::InteractivePoser(const rclcpp::NodeOptions & options)
: node_{std::make_shared<rclcpp::Node>(kNodeName, options)}
{
  param_listener_ = std::make_shared<ParamListener>(node_);
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    kNodeName, node_->get_node_base_interface(), node_->get_node_clock_interface(),
    node_->get_node_logging_interface(), node_->get_node_topics_interface(),
    node_->get_node_services_interface());

  // Latched by robot_state_publisher, so this arrives once and stays valid.
  description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) { robot_description_ = msg->data; });

  // shared_from_this() is unavailable in a constructor and TF needs a spin to
  // fill, so the real work is deferred and retried until the inputs exist.
  init_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(250), [this]() { tryInitialise(); });
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr InteractivePoser::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

std::string InteractivePoser::shadowOf(const std::string & frame) const
{
  return frame + params_.shadow_suffix;
}

// ---------------------------------------------------------------- startup --

void InteractivePoser::tryInitialise()
{
  if (initialised_) {
    return;
  }
  static int waited = 0;
  if (robot_description_.empty()) {
    if (++waited * 0.25 > params_.startup_timeout) {
      RCLCPP_FATAL(
        node_->get_logger(), "No /robot_description after %.0fs. Is the cell running?",
        params_.startup_timeout);
      rclcpp::shutdown();
    }
    return;
  }

  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
  setupPublishers();

  std::string error;
  if (!loadTargets(error)) {
    RCLCPP_FATAL(node_->get_logger(), "%s", error.c_str());
    rclcpp::shutdown();
    return;
  }

  if (!selectTarget(params_.target_calibration_joint)) {
    rclcpp::shutdown();
    return;
  }

  pre_param_cb_ = node_->add_pre_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> & to_set) { onPreSetParameters(to_set); });
  param_cb_ = node_->add_post_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & changed) { onParameterChange(changed); });
  setupServices();

  // Slow re-send of the frozen image, so the Camera display always has another
  // to pair with a CameraInfo. See docs/design.md, Making the display move.
  if (params_.image_refresh_period > 0.0) {
    image_heartbeat_ = node_->create_wall_timer(
      std::chrono::duration<double>(params_.image_refresh_period), [this]() {
        if (!params_.live_relay && has_image_) {
          image_pub_->publish(image_snapshot_);
        }
      });
  }

  warnIfDescriptionDisagrees();
  initialised_ = true;
  init_timer_->cancel();
}

bool InteractivePoser::loadTargets(std::string & error)
{
  const std::filesystem::path path{params_.calibration_file};
  if (!std::filesystem::exists(path)) {
    error = "calibration_file does not exist: " + path.string();
    return false;
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception & e) {
    error = "calibration_file is not readable YAML: " + std::string(e.what());
    return false;
  }
  const YAML::Node joints = root["calibration"] ? root["calibration"]["joints"] : YAML::Node();
  if (!joints || !joints.IsMap() || joints.size() == 0) {
    error = "no calibration.joints in " + path.string();
    return false;
  }

  // Parent, child and joint type come from the description, not from config.
  // See docs/design.md, The file is the schema.
  urdf::Model model;
  if (!model.initString(robot_description_)) {
    error = "could not parse /robot_description";
    return false;
  }

  for (const auto & entry : joints) {
    const auto name = entry.first.as<std::string>();
    const auto joint = model.getJoint(name);
    if (!joint) {
      RCLCPP_WARN(
        node_->get_logger(), "Skipping '%s': no such joint in /robot_description.", name.c_str());
      continue;
    }
    if (joint->type != urdf::Joint::FIXED) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Refusing '%s': it is not a fixed joint. Joint values belong to the robot controller.",
        name.c_str());
      continue;
    }
    Target t;
    t.joint = name;
    t.parent = joint->parent_link_name;
    t.child = joint->child_link_name;
    if (entry.second.IsMap()) {
      if (entry.second["cloud_topics"]) {
        t.cloud_topics = entry.second["cloud_topics"].as<std::vector<std::string>>();
      }
      if (entry.second["image_topic"]) {
        t.image_topic = entry.second["image_topic"].as<std::string>();
      }
      if (entry.second["camera_info_topic"]) {
        t.camera_info_topic = entry.second["camera_info_topic"].as<std::string>();
      }
      if (!t.image_topic.empty() && t.camera_info_topic.empty()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "'%s' has image_topic but no camera_info_topic; the RViz Camera overlay needs both.",
          name.c_str());
      }
      auto num = [&](const char * k) {
        return entry.second[k] ? entry.second[k].as<double>(0.0) : 0.0;
      };
      if (entry.second["nominal"]) {
        const auto n = entry.second["nominal"];
        auto nn = [&](const char * k) { return n[k] ? n[k].as<double>(0.0) : 0.0; };
        tf2::Quaternion nq;
        nq.setRPY(nn("roll"), nn("pitch"), nn("yaw"));
        t.nominal = tf2::Transform(nq, tf2::Vector3(nn("x"), nn("y"), nn("z")));
        t.nominal_declared = true;
      }
      tf2::Quaternion q;
      q.setRPY(num("roll"), num("pitch"), num("yaw"));
      t.from_file = tf2::Transform(q, tf2::Vector3(num("x"), num("y"), num("z")));
    }
    // Bounds are sized for build error, so a joint carrying layout geometry
    // would be clamped on sight and saved over the cell's real dimensions.
    // Refuse it instead. See docs/design.md, Offsets are error, not geometry.
    const auto declared = fromUrdf(joint->parent_to_joint_origin_transform);
    const auto gap = t.nominal.inverse() * declared;
    const double reach = gap.getOrigin().length();
    const double tilt = gap.getRotation().getAngle();
    if (reach > kMaxShiftM || (tilt > kMaxTiltRad && tilt < 2.0 * M_PI - kMaxTiltRad)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Refusing '%s': it sits %.0f mm / %.0f deg from nominal, past what this tool treats as "
        "build error. Either give the device a '*_mount_link' at its nominal pose and calibrate "
        "the joint below that, or declare a 'nominal:' block for it in the calibration file.",
        name.c_str(), reach * kMm, tilt * kRad2Deg);
      continue;
    }
    targets_.push_back(std::move(t));
  }

  if (targets_.empty()) {
    error = "no usable calibration targets in " + path.string();
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "%zu calibratable joint(s):", targets_.size());
  for (const auto & t : targets_) {
    RCLCPP_INFO(
      node_->get_logger(), "  %s  (%s -> %s)%s", t.joint.c_str(), t.parent.c_str(),
      t.child.c_str(),
      t.nominal_declared ? "  [nominal declared in file]"
                         : (t.hasSensor() ? "" : "  [no sensor to relay]"));
  }
  return true;
}

bool InteractivePoser::selectTarget(const std::string & joint)
{
  std::size_t index = 0;
  if (!joint.empty()) {
    const auto it = std::find_if(
      targets_.begin(), targets_.end(), [&](const Target & t) { return t.joint == joint; });
    if (it == targets_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "'%s' is not a target in the calibration file.",
                   joint.c_str());
      return false;
    }
    index = static_cast<std::size_t>(std::distance(targets_.begin(), it));
  }
  activateTarget(index);
  return true;
}

void InteractivePoser::activateTarget(std::size_t index)
{
  teardownTarget();
  active_ = index;
  RCLCPP_INFO(
    node_->get_logger(), "Calibrating '%s':  %s -> %s", target().joint.c_str(),
    target().parent.c_str(), target().child.c_str());

  auto & t = targets_[active_];
  if (!t.loaded_valid) {
    if (!seedOffsetFromTf()) {
      RCLCPP_FATAL(node_->get_logger(), "Cannot proceed without that transform.");
      rclcpp::shutdown();
      return;
    }
    t.loaded = offset_;
    t.loaded_valid = true;
  }
  // An edit made earlier in this session wins over TF: TF still reports what
  // the description loaded, so re-seeding from it would throw the work away the
  // moment the operator looked at another joint.
  offset_ = t.has_edit ? t.edited : t.loaded;
  writeOffsetToParams(offset_);
  last_published_ = offset_;
  offset_at_activate_ = t.loaded;
  if (t.has_edit) {
    RCLCPP_INFO(node_->get_logger(), "Restored this session's in-progress edit.");
  }
  setupMarker();
  setupRelays();
  publishShadowTransforms();
  publishGhost();
  logOffset("starting at");
  if (!target().hasSensor()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "'%s' has no relay_topics, so there is nothing to see move. A fixture with no sensor of "
      "its own needs the model-ghost attachment, which is not implemented yet.",
      target().joint.c_str());
  }
}

void InteractivePoser::teardownTarget()
{
  relays_.clear();   // drops subscriptions; cloud_pubs_ deliberately survive
  image_sub_.reset();
  info_sub_.reset();
  latest_image_.reset();
  has_image_ = false;
  bridged_.clear();
  server_->clear();
  server_->applyChanges();
  menu_ = interactive_markers::MenuHandler();
  if (settle_timer_) {
    settle_timer_->cancel();
    settle_timer_.reset();
  }
}

bool InteractivePoser::seedOffsetFromTf()
{
  // With nominal at identity this lookup IS the current offset, so a resumed
  // session starts where the last one left off with no bookkeeping.
  std::string err;
  if (!tf_buffer_->canTransform(
        target().parent, target().child, tf2::TimePointZero,
        tf2::durationFromSec(params_.startup_timeout), &err))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "No transform %s -> %s after %.0fs: %s", target().parent.c_str(),
      target().child.c_str(), params_.startup_timeout, err.c_str());
    return false;
  }
  const auto tf = tf_buffer_->lookupTransform(target().parent, target().child, tf2::TimePointZero);
  tf2::Transform seeded;
  tf2::fromMsg(tf.transform, seeded);
  offset_ = normaliseOffset(seeded, target().nominal);
  last_published_ = offset_;
  writeOffsetToParams(offset_);
  return true;
}

// ------------------------------------------------------- shadow tree ------

void InteractivePoser::publishShadowTransforms()
{
  std::vector<geometry_msgs::msg::TransformStamped> out;
  const auto stamp = node_->get_clock()->now();

  // Static, not dynamic: a timeless transform is what lets a frozen snapshot
  // still resolve against the newest offset. See docs/design.md, Shadow frames.
  geometry_msgs::msg::TransformStamped root;
  root.header.stamp = stamp;
  root.header.frame_id = target().parent;
  root.child_frame_id = shadowOf(target().child);
  root.transform = tf2::toMsg(offset_);
  out.push_back(root);

  for (const auto & frame : bridged_) {
    if (frame == target().child) {
      continue;
    }
    try {
      auto leaf = tf_buffer_->lookupTransform(target().child, frame, tf2::TimePointZero);
      leaf.header.stamp = stamp;
      leaf.header.frame_id = shadowOf(target().child);
      leaf.child_frame_id = shadowOf(frame);
      out.push_back(leaf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000, "Shadow leaf %s: %s", frame.c_str(),
        e.what());
    }
  }
  static_broadcaster_->sendTransform(out);
}

void InteractivePoser::setupPublishers()
{
  // Created once and never torn down, so a target switch does not rename the
  // topics. transient_local so a display attached later still gets the current
  // snapshot. See docs/design.md, Making the display move.
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  cloud_pubs_.push_back(node_->create_publisher<PointCloud2>("~/snapshot", latched));
  image_pub_ = node_->create_publisher<Image>("~/image", latched);
  // Not a free choice: RViz derives the Camera display's info topic from the
  // image's parent namespace, so ~/image must pair with ~/camera_info.
  info_pub_ = node_->create_publisher<CameraInfo>("~/camera_info", latched);
  ghost_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/ghost", latched);
}

void InteractivePoser::bridgeFrame(const std::string & frame)
{
  if (frame.empty() || bridged_.count(frame)) {
    return;
  }
  bridged_.insert(frame);
  RCLCPP_INFO(
    node_->get_logger(), "Shadowing %s as %s", frame.c_str(), shadowOf(frame).c_str());
  publishShadowTransforms();
}

void InteractivePoser::publishGhost()
{
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker wipe;
  wipe.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(wipe);

  urdf::Model model;
  if (params_.ghost_alpha <= 0.0 || !model.initString(robot_description_)) {
    ghost_pub_->publish(arr);
    return;
  }
  const auto root = model.getLink(target().child);
  if (!root) {
    ghost_pub_->publish(arr);
    return;
  }

  // Anything below another calibration joint is posed independently, so the
  // walk stops there rather than dragging a separately-measured device along.
  std::set<std::string> stop;
  for (const auto & t : targets_) {
    if (t.joint != target().joint) {
      stop.insert(t.joint);
    }
  }

  int id = 0;
  std::vector<std::pair<urdf::LinkConstSharedPtr, tf2::Transform>> pending{
    {root, tf2::Transform::getIdentity()}};
  while (!pending.empty()) {
    const auto node = pending.back();
    pending.pop_back();
    const auto & link = node.first;
    const auto & to_link = node.second;

    for (const auto & vis : link->visual_array) {
      if (!vis || !vis->geometry) {
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header.frame_id = shadowOf(target().child);
      m.ns = "ghost";
      m.id = id++;
      m.action = visualization_msgs::msg::Marker::ADD;
      // Marker (unlike PointCloud2) does have frame_locked, so RViz
      // re-transforms it every update cycle: the ghost follows the handle live
      // with nothing republished.
      m.frame_locked = true;
      m.pose = toPose(to_link * fromUrdf(vis->origin));
      m.color.r = static_cast<float>(params_.ghost_color.at(0));
      m.color.g = static_cast<float>(params_.ghost_color.at(1));
      m.color.b = static_cast<float>(params_.ghost_color.at(2));
      m.color.a = static_cast<float>(params_.ghost_alpha);
      m.scale.x = m.scale.y = m.scale.z = 1.0;

      switch (vis->geometry->type) {
        case urdf::Geometry::MESH: {
          const auto mesh = urdf::static_pointer_cast<urdf::Mesh>(vis->geometry);
          m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
          m.mesh_resource = mesh->filename;  // RViz resolves package:// itself
          // Embedded materials would paint it the real colour, and a ghost that
          // looks like the model is useless for spotting a mismatch.
          m.mesh_use_embedded_materials = false;
          m.scale.x = mesh->scale.x;
          m.scale.y = mesh->scale.y;
          m.scale.z = mesh->scale.z;
          break;
        }
        case urdf::Geometry::BOX: {
          const auto box = urdf::static_pointer_cast<urdf::Box>(vis->geometry);
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.scale.x = box->dim.x;
          m.scale.y = box->dim.y;
          m.scale.z = box->dim.z;
          break;
        }
        case urdf::Geometry::CYLINDER: {
          const auto cyl = urdf::static_pointer_cast<urdf::Cylinder>(vis->geometry);
          m.type = visualization_msgs::msg::Marker::CYLINDER;
          m.scale.x = m.scale.y = 2.0 * cyl->radius;
          m.scale.z = cyl->length;
          break;
        }
        case urdf::Geometry::SPHERE: {
          const auto sph = urdf::static_pointer_cast<urdf::Sphere>(vis->geometry);
          m.type = visualization_msgs::msg::Marker::SPHERE;
          m.scale.x = m.scale.y = m.scale.z = 2.0 * sph->radius;
          break;
        }
        default:
          continue;
      }
      arr.markers.push_back(m);
    }

    for (const auto & joint : link->child_joints) {
      if (!joint || joint->type != urdf::Joint::FIXED || stop.count(joint->name)) {
        continue;
      }
      const auto child = model.getLink(joint->child_link_name);
      if (child) {
        pending.emplace_back(child, to_link * fromUrdf(joint->parent_to_joint_origin_transform));
      }
    }
  }

  ghost_pub_->publish(arr);
  RCLCPP_INFO(
    node_->get_logger(), "Ghost: %zu visual(s) from the %s subtree", arr.markers.size() - 1,
    target().child.c_str());
}

void InteractivePoser::setupRelays()
{
  relays_.resize(target().cloud_topics.size());
  while (cloud_pubs_.size() < relays_.size()) {
    cloud_pubs_.push_back(node_->create_publisher<PointCloud2>(
      "~/snapshot_" + std::to_string(cloud_pubs_.size()),
      rclcpp::QoS(1).transient_local().reliable()));
  }
  for (std::size_t i = 0; i < relays_.size(); ++i) {
    auto & relay = relays_[i];
    relay.topic = target().cloud_topics[i];
    // SensorDataQoS (best effort): a reliable subscription silently never
    // connects to a best-effort sensor publisher.
    relay.sub = node_->create_subscription<PointCloud2>(
      relay.topic, rclcpp::SensorDataQoS(),
      [this, i](const PointCloud2::ConstSharedPtr msg) { onCloud(i, msg); });
    RCLCPP_INFO(
      node_->get_logger(), "Relay %s -> %s", relay.topic.c_str(),
      cloud_pubs_[i]->get_topic_name());
  }

  if (!target().image_topic.empty()) {
    image_sub_ = node_->create_subscription<Image>(
      target().image_topic, rclcpp::SensorDataQoS(),
      [this](const Image::ConstSharedPtr msg) {
        latest_image_ = msg;
        bridgeFrame(msg->header.frame_id);
        if (params_.live_relay) {
          Image out = *msg;
          out.header.frame_id = shadowOf(msg->header.frame_id);
          image_pub_->publish(out);
          return;
        }
        if (params_.auto_capture && !has_image_) {
          image_snapshot_ = *msg;
          image_snapshot_.header.frame_id = shadowOf(msg->header.frame_id);
          has_image_ = true;
          image_pub_->publish(image_snapshot_);
        }
      });
    RCLCPP_INFO(
      node_->get_logger(), "Relay %s -> %s", target().image_topic.c_str(),
      image_pub_->get_topic_name());
  }
  if (!target().camera_info_topic.empty()) {
    // Intrinsics are tiny and unchanging, so this is relayed live rather than
    // snapshotted -- the Camera display needs one to render at all.
    info_sub_ = node_->create_subscription<CameraInfo>(
      target().camera_info_topic, rclcpp::SensorDataQoS(),
      [this](const CameraInfo::ConstSharedPtr msg) {
        bridgeFrame(msg->header.frame_id);
        CameraInfo out = *msg;
        out.header.frame_id = shadowOf(msg->header.frame_id);
        info_pub_->publish(out);
      });
  }
}

void InteractivePoser::onCloud(std::size_t index, const PointCloud2::ConstSharedPtr & msg)
{
  if (index >= relays_.size()) {
    return;  // a target switch raced this callback
  }
  auto & relay = relays_[index];
  const bool first = !relay.latest;
  relay.latest = msg;
  // Order matters: the shadow leaf must exist before the reframed cloud is
  // published, or RViz drops the message for an unknown frame.
  bridgeFrame(msg->header.frame_id);

  if (params_.live_relay) {
    // Straight through, decimated. No snapshot is involved and no offset change
    // is needed to refresh the view -- the next frame carries the shadow frame,
    // so the marker is tracked as a side effect.
    auto out = decimate(*msg, static_cast<int>(params_.stream_decimation));
    out.header.frame_id = shadowOf(msg->header.frame_id);
    cloud_pubs_[index]->publish(out);
    return;
  }
  if (first && params_.auto_capture) {
    captureRelay(relay);
  }
}

PointCloud2 InteractivePoser::decimate(const PointCloud2 & src, int stride)
{
  if (stride <= 1) {
    return src;
  }
  PointCloud2 out = src;
  const std::size_t total = static_cast<std::size_t>(src.width) * src.height;
  const std::size_t step = src.point_step;
  std::vector<uint8_t> kept;
  kept.reserve((total / stride + 1) * step);
  for (std::size_t p = 0; p < total; p += stride) {
    const auto * begin = src.data.data() + p * step;
    kept.insert(kept.end(), begin, begin + step);
  }
  out.data = std::move(kept);
  out.width = static_cast<uint32_t>(out.data.size() / step);
  out.height = 1;
  out.row_step = static_cast<uint32_t>(out.data.size());
  out.is_dense = false;
  return out;
}

bool InteractivePoser::captureRelay(Relay & relay)
{
  if (!relay.latest) {
    return false;
  }
  relay.snapshot = decimate(*relay.latest, static_cast<int>(params_.snapshot_decimation));
  relay.snapshot.header.frame_id = shadowOf(relay.latest->header.frame_id);
  relay.has_snapshot = true;
  relay.preview = decimate(relay.snapshot, static_cast<int>(params_.stream_decimation));
  relay.preview.header.frame_id = relay.snapshot.header.frame_id;
  const auto index = static_cast<std::size_t>(&relay - relays_.data());
  cloud_pubs_[index]->publish(relay.snapshot);
  RCLCPP_INFO(
    node_->get_logger(), "Captured %s: %u points (%.1f MB), %u-point drag preview, in %s",
    relay.topic.c_str(), relay.snapshot.width * relay.snapshot.height,
    static_cast<double>(relay.snapshot.data.size()) / 1e6,
    relay.preview.width * relay.preview.height, relay.snapshot.header.frame_id.c_str());
  return true;
}

void InteractivePoser::rebuildPreviews()
{
  // The preview is derived from the frozen snapshot rather than from the live
  // message, so changing the stride re-slices the same frame the operator is
  // aligning against instead of swapping it for a newer one.
  for (auto & relay : relays_) {
    if (!relay.has_snapshot) {
      continue;
    }
    relay.preview = decimate(relay.snapshot, static_cast<int>(params_.stream_decimation));
    relay.preview.header.frame_id = relay.snapshot.header.frame_id;
  }
}

void InteractivePoser::captureSnapshots()
{
  if (latest_image_) {
    image_snapshot_ = *latest_image_;
    image_snapshot_.header.frame_id = shadowOf(latest_image_->header.frame_id);
    has_image_ = true;
    image_pub_->publish(image_snapshot_);
  }
  int captured = 0;
  for (auto & relay : relays_) {
    if (captureRelay(relay)) {
      ++captured;
    } else {
      RCLCPP_WARN(node_->get_logger(), "No message yet on %s", relay.topic.c_str());
    }
  }
  if (captured == 0 && !relays_.empty() && !has_image_) {
    RCLCPP_WARN(node_->get_logger(), "Nothing captured. Is the sensor publishing?");
  }
}

void InteractivePoser::republishSnapshots(bool preview)
{
  // Resending is what makes the cloud track the marker; see the README's
  // troubleshooting notes for why. Preview while the offset moves, full cloud
  // once it settles, so a drag costs a fraction of the sensor's bandwidth.
  for (std::size_t i = 0; i < relays_.size(); ++i) {
    if (relays_[i].has_snapshot) {
      cloud_pubs_[i]->publish(preview ? relays_[i].preview : relays_[i].snapshot);
    }
  }
  // The Camera display re-renders the overlay from current TF every frame, so
  // the image itself only needs resending when it actually changes.
}

void InteractivePoser::refreshAfterOffsetChange()
{
  if (params_.live_relay) {
    return;  // the stream already carries every change
  }
  // Gate on movement, not on time alone: an unchanged pose is not worth a
  // resend, and POSE_UPDATE repeats while a handle is merely held.
  const double moved = (offset_.getOrigin() - last_published_.getOrigin()).length();
  const double turned = offset_.getRotation().angleShortestPath(last_published_.getRotation());
  const bool visible = moved > kVisibleShiftM || turned > kVisibleTiltRad;

  const auto now = std::chrono::steady_clock::now();
  if (visible && now - last_preview_ >= std::chrono::milliseconds(50)) {
    last_preview_ = now;
    last_published_ = offset_;
    republishSnapshots(true);
  }

  // Nothing announces the end of a drag, so a debounce is the only signal.
  if (settle_timer_) {
    settle_timer_->cancel();
  }
  settle_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(params_.settle_time), [this]() {
      settle_timer_->cancel();
      last_published_ = offset_;
      republishSnapshots(false);
    });
}

// ------------------------------------------------------------- marker -----

void InteractivePoser::setupMarker()
{
  visualization_msgs::msg::InteractiveMarker marker;
  marker.header.frame_id = target().parent;  // makes the pose *be* the offset
  marker.name = kMarkerName;
  marker.description = target().joint;
  marker.scale = params_.marker_scale;
  marker.pose = toPose(offset_);
  // No frame_locked here: the field is on Marker, not InteractiveMarker, and
  // RViz re-resolves an interactive marker's header frame anyway.

  visualization_msgs::msg::Marker body;
  body.type = visualization_msgs::msg::Marker::CUBE;
  body.scale.x = body.scale.y = body.scale.z = params_.marker_scale * 0.25;
  body.color.r = 0.9f;
  body.color.g = 0.45f;
  body.color.b = 0.1f;
  body.color.a = 0.85f;

  visualization_msgs::msg::InteractiveMarkerControl grab;
  grab.always_visible = true;
  grab.markers.push_back(body);
  grab.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  grab.name = "grab";
  marker.controls.push_back(grab);

  for (const auto & axis : kAxes) {
    tf2::Quaternion q(axis.x, axis.y, axis.z, axis.w);
    q.normalize();
    visualization_msgs::msg::InteractiveMarkerControl c;
    c.orientation = tf2::toMsg(q);
    c.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    c.name = std::string("rotate_") + axis.name;
    c.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(c);
    c.name = std::string("move_") + axis.name;
    c.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(c);
  }

  server_->insert(marker);
  server_->setCallback(marker.name, [this](const Feedback::ConstSharedPtr & fb) { onFeedback(fb); });

  menu_.insert("Re-capture snapshot (latest frame)", [this](const Feedback::ConstSharedPtr &) {
    captureSnapshots();
  });
  menu_.insert("Save calibration", [this](const Feedback::ConstSharedPtr &) {
    std::string msg;
    saveCalibration(msg);
  });
  const auto live_handle =
    menu_.insert("Live relay (stream, do not freeze)", [this](const Feedback::ConstSharedPtr &) {
      node_->set_parameter(rclcpp::Parameter("live_relay", !params_.live_relay));
    });
  menu_.setCheckState(
    live_handle, params_.live_relay ? interactive_markers::MenuHandler::CHECKED
                                    : interactive_markers::MenuHandler::UNCHECKED);
  menu_.insert("Revert (undo edits to this joint)", [this](const Feedback::ConstSharedPtr &) {
    applyOffset(offset_at_activate_, true, true);
    republishSnapshots(false);
    logOffset("reverted to");
  });
  menu_.insert("Zero (built as drawn)", [this](const Feedback::ConstSharedPtr &) {
    applyOffset(tf2::Transform::getIdentity(), true, true);
    republishSnapshots(false);
    logOffset("zeroed to");
  });

  // rqt_reconfigure has no enum editor, so the target picker lives here, where
  // the operator already is. Radio-checked, one entry per joint in the file.
  if (targets_.size() > 1) {
    const auto submenu = menu_.insert("Switch target");
    for (std::size_t i = 0; i < targets_.size(); ++i) {
      const auto handle = menu_.insert(
        submenu, targets_[i].joint, [this, i](const Feedback::ConstSharedPtr &) {
          if (i != active_) {
            // Deferred: rebuilding the marker from inside its own menu callback
            // would destroy the server entry currently being dispatched.
            auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
            *timer = node_->create_wall_timer(std::chrono::milliseconds(1), [this, i, timer]() {
              (*timer)->cancel();
              activateTarget(i);
            });
          }
        });
      menu_.setCheckState(
        handle, i == active_ ? interactive_markers::MenuHandler::CHECKED
                             : interactive_markers::MenuHandler::UNCHECKED);
    }
  }

  menu_.apply(*server_, marker.name);
  server_->applyChanges();
}

void InteractivePoser::syncMarkerPose()
{
  server_->setPose(kMarkerName, toPose(offset_));
  server_->applyChanges();
}

void InteractivePoser::onFeedback(const Feedback::ConstSharedPtr & fb)
{
  if (fb->event_type != Feedback::POSE_UPDATE && fb->event_type != Feedback::MOUSE_UP) {
    return;
  }
  tf2::Transform dragged;
  tf2::fromMsg(fb->pose, dragged);
  // The marker lives in the parent frame, so its pose is already the offset. No
  // composition, no inverse -- the whole point of the mount convention.
  applyOffset(dragged, true, false);

  if (fb->event_type == Feedback::MOUSE_UP) {
    if (settle_timer_) {
      settle_timer_->cancel();
    }
    // Push the settled pose back into the server. Without this the server still
    // holds the pose the marker was created with, and anything that makes RViz
    // re-request markers -- toggling "Enable Transparency", a display reset --
    // snaps the handle back to where the drag started.
    syncMarkerPose();
    last_published_ = offset_;
    republishSnapshots(false);
    logOffset("offset now");
    return;
  }
  refreshAfterOffsetChange();
}

// --------------------------------------------------------- parameters -----

void InteractivePoser::onPreSetParameters(std::vector<rclcpp::Parameter> & to_set)
{
  // Rounded before the value is stored: doing it afterwards would mean calling
  // set_parameters from inside a parameter callback, which rclcpp warns against.
  for (auto & p : to_set) {
    if (p.get_name().rfind("offset.", 0) != 0 ||
        p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      continue;
    }
    const double rounded = std::round(p.as_double() * 100.0) / 100.0;
    p = rclcpp::Parameter(p.get_name(), rounded == 0.0 ? 0.0 : rounded);
  }
}

void InteractivePoser::onParameterChange(const std::vector<rclcpp::Parameter> & changed)
{
  if (applying_) {
    return;  // our own write-back; do not bounce
  }
  const bool touched_offset = std::any_of(
    changed.begin(), changed.end(),
    [](const rclcpp::Parameter & p) { return p.get_name().rfind("offset.", 0) == 0; });
  const bool touched_target = std::any_of(
    changed.begin(), changed.end(),
    [](const rclcpp::Parameter & p) { return p.get_name() == "target_calibration_joint"; });

  params_ = param_listener_->get_params();

  if (touched_target && !params_.target_calibration_joint.empty() &&
      params_.target_calibration_joint != target().joint)
  {
    const auto wanted = params_.target_calibration_joint;
    auto timer = std::make_shared<rclcpp::TimerBase::SharedPtr>();
    *timer = node_->create_wall_timer(std::chrono::milliseconds(1), [this, wanted, timer]() {
      (*timer)->cancel();
      selectTarget(wanted);
    });
    return;
  }
  const bool touched_live = std::any_of(
    changed.begin(), changed.end(),
    [](const rclcpp::Parameter & p) { return p.get_name() == "live_relay"; });
  if (touched_live) {
    if (params_.live_relay) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Live relay ON: streaming at the sensor's rate, decimated 1/%ld. Raise "
        "stream_decimation if RViz struggles.",
        params_.stream_decimation);
    } else {
      // Freeze what is on screen right now. Republishing the existing snapshot
      // would rewind to whatever was captured when the node started, which
      // reads as the view jumping backwards the moment you leave live mode.
      RCLCPP_INFO(node_->get_logger(), "Live relay OFF: freezing the current frame.");
      captureSnapshots();
    }
  }
  const bool touched_stream = std::any_of(
    changed.begin(), changed.end(),
    [](const rclcpp::Parameter & p) { return p.get_name() == "stream_decimation"; });
  if (touched_stream) {
    // Live mode reads the stride per message, so it already followed. Frozen
    // mode baked it into the preview at capture time, which made the same
    // parameter look like it applied only sometimes.
    rebuildPreviews();
    republishSnapshots(true);
    RCLCPP_INFO(
      node_->get_logger(), "Drag preview now 1/%ld of the snapshot.",
      params_.stream_decimation);
  }
  const bool touched_snapshot = std::any_of(
    changed.begin(), changed.end(),
    [](const rclcpp::Parameter & p) { return p.get_name() == "snapshot_decimation"; });
  if (touched_snapshot) {
    RCLCPP_INFO(
      node_->get_logger(),
      "snapshot_decimation applies at the next capture; the current frozen frame is unchanged.");
  }

  const bool touched_ghost = std::any_of(
    changed.begin(), changed.end(), [](const rclcpp::Parameter & p) {
      return p.get_name() == "ghost_alpha" || p.get_name() == "ghost_color";
    });
  if (touched_ghost && !targets_.empty()) {
    publishGhost();  // latched, so redrawing costs one small message
  }
  if (!touched_offset) {
    return;
  }
  applyOffset(offsetFromParams(params_), false, true);
  refreshAfterOffsetChange();
  logOffset("offset set to");
}

tf2::Transform InteractivePoser::offsetFromParams(const Params & p) const
{
  tf2::Quaternion q;
  q.setRPY(p.offset.roll_deg * kDeg2Rad, p.offset.pitch_deg * kDeg2Rad, p.offset.yaw_deg * kDeg2Rad);
  return target().nominal *
         tf2::Transform(
           q, tf2::Vector3(p.offset.x_mm / kMm, p.offset.y_mm / kMm, p.offset.z_mm / kMm));
}

tf2::Transform InteractivePoser::deltaFromNominal() const
{
  return target().nominal.inverse() * offset_;
}

tf2::Transform InteractivePoser::normaliseOffset(
  const tf2::Transform & t, const tf2::Transform & nominal)
{
  // Works on the offset FROM NOMINAL, never on the absolute origin, and limits
  // the MAGNITUDE rather than each axis: a rotated handle drags along several
  // parent axes at once, so per-axis clamping makes the marker shear sideways
  // at the limit. See docs/design.md, Keeping one number.
  const tf2::Transform delta = nominal.inverse() * t;
  tf2::Vector3 shift = delta.getOrigin();
  const double reach = shift.length();
  if (reach > kMaxShiftM) {
    shift *= kMaxShiftM / reach;
  }

  // Same argument for rotation: slerp back along the same axis rather than
  // clipping Euler terms, which would tilt the axis as it saturates.
  tf2::Quaternion turn = delta.getRotation().normalized();
  if (turn.getW() < 0.0) {
    turn = tf2::Quaternion(-turn.getX(), -turn.getY(), -turn.getZ(), -turn.getW());
  }
  const double angle = turn.getAngle();
  if (angle > kMaxTiltRad && angle > 1e-9) {
    turn = tf2::Quaternion::getIdentity().slerp(turn, kMaxTiltRad / angle);
  }

  // Quantise so the stored transform, the parameters and the saved file all
  // carry the same number. 0.01 mm is far below what hand alignment resolves.
  auto snap = [](double v, double lim, double quantum) {
    const double clamped = std::max(-lim, std::min(lim, v));
    const double q = std::round(clamped / quantum) * quantum;
    return q == 0.0 ? 0.0 : q;  // also kills -0
  };
  double roll, pitch, yaw;
  tf2::Matrix3x3(turn).getRPY(roll, pitch, yaw);
  tf2::Quaternion out;
  // Per-axis limits are a backstop, not the real constraint: after the slerp
  // they should never bind. They exist so a component cannot land outside its
  // parameter's bounds<> and get the whole set rejected.
  out.setRPY(
    snap(roll, kMaxTiltRad, kTiltQuantumRad), snap(pitch, kMaxTiltRad, kTiltQuantumRad),
    snap(yaw, kMaxTiltRad, kTiltQuantumRad));
  const tf2::Transform clamped(
    out, tf2::Vector3(
           snap(shift.x(), kMaxShiftM, kShiftQuantumM), snap(shift.y(), kMaxShiftM, kShiftQuantumM),
           snap(shift.z(), kMaxShiftM, kShiftQuantumM)));
  return nominal * clamped;
}

void InteractivePoser::writeOffsetToParams(const tf2::Transform & offset)
{
  // Parameters carry the offset from nominal, so they read as build error
  // whether or not the joint carries layout geometry of its own.
  const tf2::Transform delta = target().nominal.inverse() * offset;
  double roll, pitch, yaw;
  tf2::Matrix3x3(delta.getRotation()).getRPY(roll, pitch, yaw);
  const auto & o = delta.getOrigin();
  // Already quantised, but the rad->deg conversion reintroduces float dust, so
  // round once more in the units the GUI actually shows.
  auto r2 = [](double v) {
    const double q = std::round(v * 100.0) / 100.0;
    return q == 0.0 ? 0.0 : q;
  };
  applying_ = true;
  node_->set_parameters({
    rclcpp::Parameter("offset.x_mm", r2(o.x() * kMm)),
    rclcpp::Parameter("offset.y_mm", r2(o.y() * kMm)),
    rclcpp::Parameter("offset.z_mm", r2(o.z() * kMm)),
    rclcpp::Parameter("offset.roll_deg", r2(roll * kRad2Deg)),
    rclcpp::Parameter("offset.pitch_deg", r2(pitch * kRad2Deg)),
    rclcpp::Parameter("offset.yaw_deg", r2(yaw * kRad2Deg)),
  });
  applying_ = false;
  params_ = param_listener_->get_params();
}

void InteractivePoser::applyOffset(
  const tf2::Transform & offset, bool write_params, bool refresh_marker)
{
  offset_ = normaliseOffset(offset, target().nominal);
  if (!targets_.empty()) {
    targets_[active_].edited = offset_;
    targets_[active_].has_edit = true;
  }
  if (write_params) {
    writeOffsetToParams(offset_);
  }
  publishShadowTransforms();
  if (refresh_marker) {
    syncMarkerPose();
  }
}

void InteractivePoser::logOffset(const char * prefix) const
{
  const tf2::Transform delta = deltaFromNominal();
  double roll, pitch, yaw;
  tf2::Matrix3x3(delta.getRotation()).getRPY(roll, pitch, yaw);
  const auto & o = delta.getOrigin();
  RCLCPP_INFO(
    node_->get_logger(), "%s  x %+.2f  y %+.2f  z %+.2f mm   roll %+.2f  pitch %+.2f  yaw %+.2f deg",
    prefix, o.x() * kMm, o.y() * kMm, o.z() * kMm, roll * kRad2Deg, pitch * kRad2Deg,
    yaw * kRad2Deg);
}

// -------------------------------------------------------- persistence -----

void InteractivePoser::setupServices()
{
  auto reply = [](std::shared_ptr<Trigger::Response> res, bool ok, std::string msg) {
    res->success = ok;
    res->message = std::move(msg);
  };
  capture_srv_ = node_->create_service<Trigger>(
    "~/capture",
    [this, reply](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      captureSnapshots();
      reply(res, true, "captured");
    });
  save_srv_ = node_->create_service<Trigger>(
    "~/save",
    [this, reply](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      // Sequenced deliberately: reply(res, saveCalibration(msg), msg) leaves the
      // read of msg unsequenced against the call that fills it.
      std::string msg;
      const bool ok = saveCalibration(msg);
      reply(res, ok, msg);
    });
  zero_srv_ = node_->create_service<Trigger>(
    "~/zero",
    [this, reply](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      applyOffset(tf2::Transform::getIdentity(), true, true);
      republishSnapshots(false);
      logOffset("zeroed to");
      reply(res, true, "offset zeroed");
    });
  revert_srv_ = node_->create_service<Trigger>(
    "~/revert",
    [this, reply](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
      applyOffset(offset_at_activate_, true, true);
      republishSnapshots(false);
      logOffset("reverted to");
      reply(res, true, "reverted to session start");
    });
}

bool InteractivePoser::currentOffsetOf(const Target & t, tf2::Transform & out) const
{
  if (t.joint == target().joint) {
    out = offset_;  // the value under the operator's hand
    return true;
  }
  if (t.has_edit) {
    out = t.edited;  // measured earlier this session, not yet in the description
    return true;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(t.parent, t.child, tf2::TimePointZero);
    tf2::fromMsg(tf.transform, out);
    out = normaliseOffset(out, t.nominal);
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void InteractivePoser::warnIfDescriptionDisagrees() const
{
  for (const auto & t : targets_) {
    tf2::Transform live;
    if (!currentOffsetOf(t, live)) {
      continue;
    }
    const double moved = (live.getOrigin() - t.from_file.getOrigin()).length();
    const double turned = live.getRotation().angleShortestPath(t.from_file.getRotation());
    if (moved > kVisibleShiftM || turned > kVisibleTiltRad) {
      RCLCPP_WARN(
        node_->get_logger(),
        "'%s' reads %.2f mm from the running cell but %.2f mm in the file. The cell was probably "
        "launched without calibration_file, or with a different one. Saving will record what the "
        "cell is actually doing.",
        t.joint.c_str(), live.getOrigin().length() * kMm,
        t.from_file.getOrigin().length() * kMm);
    }
  }
}

bool InteractivePoser::saveCalibration(std::string & message)
{
  const std::filesystem::path input{params_.calibration_file};
  // The input file is only the template: it supplies relay_topics and anything
  // else a human put there. Every offset is re-read from the live system, so
  // the file always describes the cell as it is actually running.
  YAML::Node root;
  try {
    root = YAML::LoadFile(input.string());
  } catch (const YAML::Exception & e) {
    message = "calibration file is not readable YAML";
    RCLCPP_ERROR(node_->get_logger(), "%s (%s); refusing to write.", message.c_str(), e.what());
    return false;
  }

  std::filesystem::path path = input;
  if (params_.timestamp_output) {
    const std::filesystem::path dir =
      params_.output_directory.empty() ? input.parent_path()
                                       : std::filesystem::path{params_.output_directory};
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    // Fixed stem, not the input's. Deriving it produced names like
    // default_calibration_<stamp>.yaml, and a measured calibration is the one
    // thing it is not. Which cell it belongs to is the directory's job.
    path = dir / ("calibration_" + fileStamp() + input.extension().string());
  }
  YAML::Node cal = root["calibration"];
  cal["captured"] = isoNow();
  if (const char * user = std::getenv("USER")) {
    cal["operator"] = std::string(user);
  }

  // yaml-cpp stringifies a scalar the moment it is assigned, at max_digits10 --
  // Emitter::SetDoublePrecision never gets a look in. Round here so 12.35 mm
  // reads as 0.01235 rather than 0.012350000000000002; the file is meant to be
  // read by a person, and float dust makes it look untrustworthy.
  auto tidy = [](double v) {
    const double r = std::round(v * 1e9) / 1e9;
    return r == 0.0 ? 0.0 : r;
  };

  // Every target, and every value from the live system rather than an earlier
  // file. See docs/design.md, What a save means.
  for (const auto & t : targets_) {
    tf2::Transform live;
    if (!currentOffsetOf(t, live)) {
      RCLCPP_WARN(
        node_->get_logger(), "No transform for '%s'; leaving its file entry alone.",
        t.joint.c_str());
      continue;
    }
    double roll, pitch, yaw;
    tf2::Matrix3x3(live.getRotation()).getRPY(roll, pitch, yaw);
    const auto & o = live.getOrigin();
    YAML::Node entry = cal["joints"][t.joint];
    entry["x"] = tidy(o.x());
    entry["y"] = tidy(o.y());
    entry["z"] = tidy(o.z());
    entry["roll"] = tidy(roll);
    entry["pitch"] = tidy(pitch);
    entry["yaw"] = tidy(yaw);
    RCLCPP_INFO(
      node_->get_logger(), "  %-46s x %+.2f  y %+.2f  z %+.2f mm   yaw %+.2f deg%s",
      t.joint.c_str(), o.x() * kMm, o.y() * kMm, o.z() * kMm, yaw * kRad2Deg,
      t.joint == target().joint ? "   <- edited now" : "");
  }

  std::ofstream f(path);
  if (!f) {
    message = "cannot write " + path.string();
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return false;
  }
  f << "# As-built calibration -- offsets from nominal, metres and radians.\n"
    << "# All zeros means built as drawn: every value here is build error.\n"
    << "# Loaded by the cell URDF and by interactive_poser; the tool rewrites\n"
    << "# this file on save, so comments inside the data are not preserved.\n"
    << root << "\n";
  f.close();

  logOffset("saved");
  RCLCPP_INFO(node_->get_logger(), "Wrote %s [%s]", path.c_str(), target().joint.c_str());
  if (params_.timestamp_output) {
    RCLCPP_INFO(
      node_->get_logger(), "Input file untouched. To adopt this result:\n  cp %s %s",
      path.c_str(), input.c_str());
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "Restart robot_state_publisher (and move_group, if running) to apply.");
  message = "wrote " + path.string();
  return true;
}
}  // namespace interactive_poser
