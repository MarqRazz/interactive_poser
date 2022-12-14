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

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>

namespace interactive_poser
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;
/**
 * @brief The InteractivePoserNode class provides a ROS .
 */
class CameraPoser
{
public:
  /**
   * @brief Constructor for 
   * @details This initializes the 
   *
   * @param options 
   */
  CameraPoser(std::shared_ptr<rclcpp::Node> node, const std::shared_ptr<tf2_ros::Buffer> transform_buffer, 
              std::string topic_name, std::string poser_frame);

  /**
   * @brief The destructor for 
   */
  // ~CameraPoser() = default;

  geometry_msgs::msg::TransformStamped target_camera_tf;  
  geometry_msgs::msg::TransformStamped camera_base_to_sensor_tf;
  geometry_msgs::msg::TransformStamped poser_tf;
  geometry_msgs::msg::TransformStamped poser_sensor_tf;

  visualization_msgs::msg::InteractiveMarker poser_marker_;

  void captureCloud ();
  void initializeMarker();

private:
  /**
   * @brief RCLCPP node for InteractivePoserNode
   */
  std::shared_ptr<rclcpp::Node> node_;
  std::string cloud_topic_;
  std::string camera_base_;
  std::string world_frame_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::mutex snapshot_mutex_;
  PointCloud2::SharedPtr latest_cloud_;
  PointCloud2::SharedPtr snapshot_cloud_;
  std::atomic<bool> received_cloud_;

  std::shared_ptr<tf2_ros::Buffer> transform_buffer_;

  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;  

  void cloudCB (const PointCloud2::SharedPtr cloud_msgs);
  void initializeFrames();
  
  
};
}  // namespace interactive_poser
