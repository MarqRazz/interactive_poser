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

#include "interactive_poser/poser.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_ros/buffer.h>


namespace interactive_poser
{
class TfPoser : public Poser
{
public:
  /**
   * @brief Constructor for 
   * @details This initializes the 
   *
   * @param options 
   */
  TfPoser(std::string& parent_frame_id, std::string& frame_id_to_mirror, bool lookup_initial_tf = false);

  /**
   *  @brief Initialize the poser.
   *  @param name The name of this poser.
   *  @param node The node to use when loading feature
   *         finder configuration data.
   *  @returns True/False if the poser was able to be initialized (this can take time to receive sensor data)
   */
  virtual bool init(const std::string& name,
                    std::shared_ptr<tf2_ros::Buffer> buffer,
                    rclcpp::Node::SharedPtr node);

  bool triggerSensorCapture();
  std::vector<geometry_msgs::msg::TransformStamped> getPoserFrames();
  visualization_msgs::msg::InteractiveMarker getPoserMarker(interactive_markers::MenuHandler& menu_handler);                    
  
private:
  void initializeMarker();  

  std::string parent_frame_id_; 
  std::string target_frame_id_;
  bool lookup_initial_pose_ {false};
};

}  // namespace interactive_poser
