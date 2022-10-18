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

#include <interactive_poser/tf_poser.hpp>

namespace
{
constexpr auto kNodeName = "interactive_poser_node";
const auto kLogger = rclcpp::get_logger(kNodeName);
}

namespace interactive_poser
{
TfPoser::TfPoser(std::string& parent_frame_id, std::string& frame_id_to_mirror, bool lookup_initial_tf)
  : parent_frame_id_(parent_frame_id)
  , target_frame_id_(frame_id_to_mirror)
  , lookup_initial_pose_(lookup_initial_pose_)
{
  initial_poser_tf_.header.frame_id = parent_frame_id;
  initial_poser_tf_.child_frame_id = frame_id_to_mirror;
}

bool TfPoser::init(const std::string& name,
                   std::shared_ptr<tf2_ros::Buffer> buffer,
                   rclcpp::Node::SharedPtr node)
{
  if (!Poser::init(name, buffer, node))
  {
    return false;
  }

  clock_ = node->get_clock();

  if(lookup_initial_pose_)
  {
    try{
      initial_poser_tf_ = transform_buffer_->lookupTransform(parent_frame_id_, target_frame_id_, clock_->now(),
                                                             rclcpp::Duration::from_seconds(3.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(kLogger, "TransformException: %s. \nCould NOT lookup transform from: %s to %s",
                   ex.what(), parent_frame_id_.c_str(), target_frame_id_.c_str());
      return false;
    }
  }

  // initialize the poser_tf at the same place as the target but give it a poser frame_id
  poser_tf_ = initial_poser_tf_;
  poser_tf_.child_frame_id = target_frame_id_ + "_ip";

  initializeMarker();

  return true;
}

void TfPoser::initializeMarker()
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
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
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

std::vector<geometry_msgs::msg::TransformStamped> TfPoser::getPoserFrames()
{
  // std::vector<geometry_msgs::msg::TransformStamped> tf_frames;
  // tf_frames.push_back(poser_tf_);
  // return tf_frames;
  return {poser_tf_};
}

}  // namespace interactive_poser