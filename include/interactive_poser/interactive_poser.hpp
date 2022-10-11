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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <interactive_poser/camera_poser.hpp>

#include <map>

namespace interactive_poser
{
/**
 * @brief The InteractivePoserNode class provides a ROS .
 */
class InteractivePoserNode
{
public:
  /**
   * @brief Constructor for InteractivePoserNode.
   * @details This initializes the 
   *
   * @param options rclcpp::NodeOptions to pass to node_ when initializing it.
   */
  InteractivePoserNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief The destructor for InteractivePoserNode.
   */
  ~InteractivePoserNode() = default;

  /**
   * @brief Gets the NodeBaseInterface of node_.
   * @details This function exists to allow running InteractivePoserNode as a component in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of node_.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  void frameCallback();

  void processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  /**
   * @brief RCLCPP node for InteractivePoserNode
   */
  std::shared_ptr<rclcpp::Node> node_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  std::unique_ptr<CameraPoser> scene_cam;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr frame_timer_;

  // std::map<std::string, geometry_msgs::msg::TransformStamped> frames_to_pub;
  // std::map<std::string, geometry_msgs::msg::TransformStamped> frames_to_sub;


};
}  // namespace interactive_poser
