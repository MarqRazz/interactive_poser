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

#include <rclcpp/rclcpp.hpp>
#include <interactive_poser/joint_pose_from_yaml.hpp>
#include <yaml-cpp/yaml.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("interactive_poser");

namespace interactive_poser
{

// Load a joint pose from yaml file
bool getPoseFromYaml(const std::string& filename,
                      geometry_msgs::msg::TransformStamped& joint_pose)
{

  RCLCPP_INFO(LOGGER, "Opening %s", filename.c_str());
  YAML::Node yaml_poses = YAML::LoadFile(filename);

  // for (auto pose : yaml_poses)
  // {
    // robot_calibration_msgs::msg::CaptureConfig msg;    

    // for (auto it = pose.begin(); it != pose.end(); ++it)
    // {
    //   RCLCPP_INFO(LOGGER, "Opening %s", it->first.as<std::string>());
      // if (it->first.as<std::string>() == "joints")
      // {
      //   for (auto joint = it->second.begin(); joint != it->second.end(); ++joint)
      //   {
      //     // msg.joint_states.name.push_back(joint->as<std::string>());
      //   }
      // }
      // else if (it->first.as<std::string>() == "positions")
      // {
      //   for (auto position = it->second.begin(); position != it->second.end(); ++position)
      //   {
      //     // msg.joint_states.position.push_back(position->as<double>());
      //   }
      // }
      // else if (it->first.as<std::string>() == "features")
      // {
      //   for (auto feature = it->second.begin(); feature != it->second.end(); ++feature)
      //   {
      //     // msg.features.push_back(feature->as<std::string>());
      //   }
      // }
    // }

    // if (//!msg.joint_states.name.empty() &&
    //     msg.joint_states.name.size() == msg.joint_states.position.size())
    // {
    //   poses.push_back(msg);
    // }
    // else
    // {
    //   RCLCPP_WARN(LOGGER, "Discarding pose due to invalid joint_states");
    // }
  // }

  return true;
}

}  // namespace interactive_poser
