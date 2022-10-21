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

#include <interactive_poser/pointcloud_camera_poser.hpp>

namespace
{
constexpr auto kNodeName = "interactive_poser_node";
const auto kLogger = rclcpp::get_logger(kNodeName);

using namespace visualization_msgs::msg;
}

namespace interactive_poser
{
PointcloudCameraPoser::PointcloudCameraPoser(std::string topic_name, std::string poser_frame)
  : pointcloud_topic_(topic_name)
{
}

bool PointcloudCameraPoser::init(const std::string& name,
                   std::shared_ptr<tf2_ros::Buffer> buffer,
                   rclcpp::Node::SharedPtr node,
                   std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server)
{
  if (!Poser::init(name, buffer, node, int_marker_server))
  {
    return false;
  }

  clock_ = node->get_clock();

  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;  

  // TODO: make these parameters
  world_frame_ = "base_link";
  std::string camera_name = "scene_camera";
  camera_base_ = camera_name + "_mount_link";
  // std::string cloud_topic = camera_name + "/points";

  // Fix the subscriber to support hardware: rclcpp::SensorDataQoS()
  // this is required by Gazebo: rclcpp::QoS(1).reliable()
  RCLCPP_INFO(kLogger, "Subscribing to PointCloud %s", pointcloud_topic_.c_str());
  cloud_subscriber_ = node->create_subscription<PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS().keep_last(1), 
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr camera_point_cloud) {
          cloudCB(camera_point_cloud);
        },
        options);
  poser_cloud_publisher_ = node->create_publisher<PointCloud2>(pointcloud_topic_ + "_ip", rclcpp::SensorDataQoS().reliable());

  // TODO implement timeout
  received_cloud_ = false;
  rclcpp::Rate r {std::chrono::milliseconds(100)};
  while(!received_cloud_)
  {
    rclcpp::spin_some(node);
    r.sleep();
  }

  return true;
}

void PointcloudCameraPoser::initializeFrames()
{
  // Check that transform is possible. Fail without transforming if not possible.
  if (!transform_buffer_->canTransform(world_frame_, camera_base_, latest_cloud_->header.stamp))
  {
    RCLCPP_ERROR(kLogger, "Failed to lookup Camera Sensor frame: %s to imitate", camera_base_.c_str());
  }
  initial_poser_tf_ =
      transform_buffer_->lookupTransform(world_frame_, camera_base_, latest_cloud_->header.stamp);

  // Check that transform is possible. Fail without transforming if not possible.
  if (!transform_buffer_->canTransform(camera_base_, latest_cloud_->header.frame_id, latest_cloud_->header.stamp))
  {
    RCLCPP_ERROR(kLogger, "Failed to lookup Camera Sensor frame: %s to imitate", latest_cloud_->header.frame_id.c_str());
  }
  initial_target_to_sensor_tf_ =
      transform_buffer_->lookupTransform(camera_base_, latest_cloud_->header.frame_id, latest_cloud_->header.stamp);

  RCLCPP_INFO(kLogger, "Setting up poser frames");
  poser_tf_ = initial_poser_tf_; // initialize the poser frame in the same location as the target_camera
  poser_tf_.set__child_frame_id(camera_base_ + "_ip");
  poser_sensor_tf_ = initial_target_to_sensor_tf_; // initialize the poser frame in the same location as the target_sensor
  poser_sensor_tf_.set__child_frame_id(latest_cloud_->header.frame_id + "_ip");
  poser_sensor_tf_.header.frame_id = camera_base_ + "_ip";

  // initializeMarker();
}

void PointcloudCameraPoser::initializeMarker()
{
  poser_marker_.header.frame_id = poser_tf_.header.frame_id;
  poser_marker_.pose.position.x = poser_tf_.transform.translation.x;
  poser_marker_.pose.position.y = poser_tf_.transform.translation.y;
  poser_marker_.pose.position.z = poser_tf_.transform.translation.z;
  poser_marker_.pose.orientation.x = poser_tf_.transform.rotation.x;
  poser_marker_.pose.orientation.y = poser_tf_.transform.rotation.y;
  poser_marker_.pose.orientation.z = poser_tf_.transform.rotation.z;
  poser_marker_.pose.orientation.w = poser_tf_.transform.rotation.w;
  poser_marker_.scale = 0.1;

  poser_marker_.name = this->getName();
  poser_marker_.description = this->getName() + " Interactive Marker";

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.02;
  marker.scale.y = 0.1;
  marker.scale.z = 0.02;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.pose.position.z = 0.01; // move the marker half the z scale to put the origin at the base of the cube

  visualization_msgs::msg::InteractiveMarkerControl control_base;
  control_base.always_visible = true;
  control_base.markers.push_back(marker);
  control_base.name = this->getName() + "_control";
  control_base.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  poser_marker_.controls.push_back(control_base);
}

void PointcloudCameraPoser::initializeMarkerMenu(interactive_markers::MenuHandler& menu_handler)
{
  Poser::initializeMarkerMenu(menu_handler);

  // how do we tell the action server that the user is done estimating the pose?
  // menu_handler.insert( "Action Complete",
  //   [this](const InteractiveMarkerFeedback::ConstSharedPtr msg) 
  //   { estimateComplete(); 
  //   });
}

void PointcloudCameraPoser::cloudCB (const PointCloud2::SharedPtr cloud_msg)
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

bool PointcloudCameraPoser::triggerSensorCapture()
{
  if(!received_cloud_)
  {
    RCLCPP_WARN(kLogger, "No PointCloud yet");
    return false;
  }

  if(snapshot_mutex_.try_lock())
  {
    snapshot_cloud_ = latest_cloud_;
    //update the transform timestamp
    poser_tf_.header.stamp = snapshot_cloud_->header.stamp;
    poser_sensor_tf_.header.stamp = snapshot_cloud_->header.stamp;;
    snapshot_mutex_.unlock();

    snapshot_cloud_->header.frame_id = poser_sensor_tf_.child_frame_id;
    poser_cloud_publisher_->publish(*snapshot_cloud_);
    // RCLCPP_INFO(kLogger, "Received PointCloud snapshot: frame_id: %s", snapshot_cloud_->header.frame_id.c_str());
  }
  return true;
}

std::vector<geometry_msgs::msg::TransformStamped> PointcloudCameraPoser::getPoserFrames()
{
  return {poser_tf_, poser_sensor_tf_};
}

void PointcloudCameraPoser::setPoserFrame(geometry_msgs::msg::TransformStamped& tf_update)
{
  poser_tf_.transform = tf_update.transform;
}
}  // namespace interactive_poser