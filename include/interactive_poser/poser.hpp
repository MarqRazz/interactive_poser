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
#include <interactive_markers/menu_handler.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/interactive_marker.hpp>


namespace interactive_poser
{

void addMoveControl(visualization_msgs::msg::InteractiveMarker &i_marker);
bool deleteMoveControl(visualization_msgs::msg::InteractiveMarker &i_marker);

/**
 * @brief The InteractivePoserNode class provides a ROS .
 */
class Poser
{
public:
  // pluginlib requires empty constructor
  Poser() {};
  virtual ~Poser() {};
  /**
   *  @brief Initialize the poser.
   *  @param name The name of this poser.
   *  @param node The node to use when loading feature
   *         finder configuration data.
   *  @returns True/False if the poser was able to be initialized (this can take time to receive sensor data)
   */
  virtual bool init(const std::string& name,
                    std::shared_ptr<tf2_ros::Buffer> buffer,
                    rclcpp::Node::SharedPtr node,
                    std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server)
  {
    name_ = name;
    transform_buffer_ = buffer;
    node_ptr_ = node;
    int_marker_server_ = int_marker_server;
    return true;
  };

  /**
   *  @brief Get the name of this poser.
   */
  std::string getName()
  {
    return name_;
  }

  virtual void initializeMarkerMenu(interactive_markers::MenuHandler& menu_handler);

  virtual bool triggerSensorCapture() = 0;
  virtual std::vector<geometry_msgs::msg::TransformStamped> getPoserFrames() = 0;
  virtual void setPoserFrame(geometry_msgs::msg::TransformStamped& tf_update) = 0;
  visualization_msgs::msg::InteractiveMarker getPoserMarker()
  {
    return poser_marker_;
  }
  void setPoserMarker(visualization_msgs::msg::InteractiveMarker &marker)
  {
    poser_marker_ = marker;
  }

  void setMoveFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback );  

protected:

  rclcpp::Node::WeakPtr node_ptr_;
  // Stored as weak pointer, need to grab a real shared pointer
  // auto node = node_ptr_.lock();

  std::shared_ptr<tf2_ros::Buffer> transform_buffer_;
  rclcpp::Clock::SharedPtr clock_;

  // copy of the interactive marker server so posers can lookup their marker
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_marker_server_;

  // the initial frame we are mirroring (usually published by robot_state_publisher)
  geometry_msgs::msg::TransformStamped initial_poser_tf_;
  // the poser frame
  geometry_msgs::msg::TransformStamped poser_tf_;
  // interactive marker to interact with this poser
  visualization_msgs::msg::InteractiveMarker poser_marker_;
  
private:
  std::string name_;
};
}  // namespace interactive_poser
