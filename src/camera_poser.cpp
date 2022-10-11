/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Marq Rasmussen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Marq Rasmussen. */

#include <interactive_poser/camera_poser.hpp>

namespace
{
constexpr auto kNodeName = "interactive_poser_node";
const auto kLogger = rclcpp::get_logger(kNodeName);
}

namespace interactive_poser
{
CameraPoser::CameraPoser(std::shared_ptr<rclcpp::Node> node, 
                        const std::shared_ptr<tf2_ros::Buffer> transform_buffer, 
                        std::string topic_name, std::string poser_frame) 
  : node_(node)
  , transform_buffer_(transform_buffer)
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;  

  // TODO: make these parameters
  world_frame_ = "base_link";
  std::string camera_name = "scene_camera";
  camera_base_ = camera_name + "_mount_link";
  std::string cloud_topic = camera_name + "/points";

  RCLCPP_INFO(kLogger, "Subscribing to PointCloud %s", cloud_topic.c_str());
  cloud_subscriber_ = node_->create_subscription<PointCloud2>(
        cloud_topic, rclcpp::QoS(1).reliable().keep_last(1), 
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr camera_point_cloud) {
          cloudCB(camera_point_cloud);
        },
        options);
  pub_ = node_->create_publisher<PointCloud2>("/cloud_snapshot", rclcpp::SensorDataQoS().reliable());

  // std::this_thread::sleep_for( std::chrono::milliseconds(2000) );
  received_cloud_ = false;

  // captureCloud();

}

void CameraPoser::initializeFrames()
{
  // Check that transform is possible. Fail without transforming if not possible.
  if (!transform_buffer_->canTransform(world_frame_, camera_base_, latest_cloud_->header.stamp))
  {
    RCLCPP_ERROR(kLogger, "Failed to lookup Camera Sensor frame: %s to imitate", camera_base_.c_str());
  }
  // Transform is confirmed possible, get the transform and transform the point cloud.
  target_camera_tf =
      transform_buffer_->lookupTransform(world_frame_, camera_base_, latest_cloud_->header.stamp);

  // Check that transform is possible. Fail without transforming if not possible.
  if (!transform_buffer_->canTransform(camera_base_, latest_cloud_->header.frame_id, latest_cloud_->header.stamp))
  {
    RCLCPP_ERROR(kLogger, "Failed to lookup Camera Sensor frame: %s to imitate", latest_cloud_->header.frame_id.c_str());
  }
  // Transform is confirmed possible, get the transform and transform the point cloud.
  camera_base_to_sensor_tf =
      transform_buffer_->lookupTransform(camera_base_, latest_cloud_->header.frame_id, latest_cloud_->header.stamp);

  RCLCPP_INFO(kLogger, "Setting up poser frames");
  poser_tf = target_camera_tf; // initialize the poser frame in the same location as the target_camera
  poser_tf.set__child_frame_id(camera_base_ + "_ip");
  poser_sensor_tf = camera_base_to_sensor_tf; // initialize the poser frame in the same location as the target_sensor
  poser_sensor_tf.set__child_frame_id(latest_cloud_->header.frame_id + "_ip");
  poser_sensor_tf.header.frame_id = camera_base_ + "_ip";
}

void CameraPoser::initializeMarker()
{
  poser_marker_.header.frame_id = "base_link";
  poser_marker_.pose.position.x = poser_tf.transform.translation.x;
  poser_marker_.pose.position.y = poser_tf.transform.translation.y;
  poser_marker_.pose.position.z = poser_tf.transform.translation.z;
  poser_marker_.pose.orientation.x = poser_tf.transform.rotation.x;
  poser_marker_.pose.orientation.y = poser_tf.transform.rotation.y;
  poser_marker_.pose.orientation.z = poser_tf.transform.rotation.z;
  poser_marker_.pose.orientation.w = poser_tf.transform.rotation.w;
  poser_marker_.scale = 0.1;

  poser_marker_.name = "scene_camera_poser";
  poser_marker_.description = "Posable Camera Marker";

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.02;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl control_base;
  control_base.always_visible = true;
  control_base.markers.push_back(marker);
  control_base.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  poser_marker_.controls.push_back(control_base);

  visualization_msgs::msg::InteractiveMarkerControl control;
  tf2::Quaternion orientation(1.0, 0.0, 0.0, 1.0);
  orientation.normalize();
  control.orientation = tf2::toMsg(orientation);
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  poser_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  poser_marker_.controls.push_back(control);

  orientation = tf2::Quaternion(0.0, 1.0, 0.0, 1.0);
  orientation.normalize();
  control.orientation = tf2::toMsg(orientation);
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  poser_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  poser_marker_.controls.push_back(control);

  orientation = tf2::Quaternion(0.0, 0.0, 1.0, 1.0);
  orientation.normalize();
  control.orientation = tf2::toMsg(orientation);
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  poser_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  poser_marker_.controls.push_back(control);
  

}

void CameraPoser::cloudCB (const PointCloud2::SharedPtr cloud_msg)
{
  if(snapshot_mutex_.try_lock())
  {
    RCLCPP_INFO_ONCE(kLogger, "Received PointCloud message");    
    latest_cloud_ = cloud_msg;    
    if(!received_cloud_)
    {
      initializeFrames();
      received_cloud_ = true;
    }
    snapshot_mutex_.unlock();
  }
}

void CameraPoser::captureCloud()
{
  if(!received_cloud_)
  {
    RCLCPP_WARN(kLogger, "No PointCloud yet");
    return;
  }

  if(snapshot_mutex_.try_lock())
  {
    snapshot_cloud_ = latest_cloud_;
    //update the transform timestamp
    poser_tf.header.stamp = snapshot_cloud_->header.stamp;
    poser_sensor_tf.header.stamp = snapshot_cloud_->header.stamp;;
    snapshot_mutex_.unlock();

    snapshot_cloud_->header.frame_id = snapshot_cloud_->header.frame_id + "_ip";
    pub_->publish(*snapshot_cloud_);
    // RCLCPP_INFO(kLogger, "Received PointCloud snapshot: frame_id: %s", snapshot_cloud_->header.frame_id.c_str());
  }
}
}  // namespace interactive_poser