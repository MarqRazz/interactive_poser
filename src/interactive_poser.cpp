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

#include <interactive_poser/interactive_poser.hpp>

#include <iterator>
#include <sstream>
#include <string>

namespace
{
using namespace std::chrono_literals;

constexpr auto kNodeName = "interactive_poser_node";
const auto kLogger = rclcpp::get_logger(kNodeName);
// The default transform buffer size is 10 seconds.
constexpr auto kTransformBufferSizeSec = 5;


void declareParameters(const std::shared_ptr<rclcpp::Node>& node)
{
  // TODO: Explicitly declare ROS parameters used by this node
  // node->declare_parameter<std::string>("", "");
  // node->declare_parameter<std::string>("", "");
}
}  // namespace

namespace interactive_poser
{
InteractivePoserNode::InteractivePoserNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>(kNodeName, options) }
  , menu_handler_()
{
  // Do parameter declaration
  declareParameters(node_);

  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
    kNodeName,
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_services_interface());

  // Create a transform listener so that we can output the door handle pose in the desired frame.  
  std::shared_ptr<tf2_ros::Buffer> transform_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), std::chrono::seconds(kTransformBufferSizeSec));
  tf2_ros::TransformListener transform_listener(*transform_buffer);
  // Wait a little bit to give TF time to update.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  scene_cam = std::make_unique<CameraPoser>(node_, transform_buffer, "topic", "poser");

  // create a timer to update the published transforms
  frame_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&InteractivePoserNode::frameCallback, this));

  rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue( true ) );
  node_->set_parameter( simTime );
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr InteractivePoserNode::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

void InteractivePoserNode::frameCallback()
{
  if (!tf_broadcaster_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_->shared_from_this());
  }

  tf2::TimePoint tf_time_point(std::chrono::nanoseconds(node_->get_clock()->now().nanoseconds()));

  // It would be nice if we looped through a vector of frames to be published
  // std::map<std::string, geometry_msgs::msg::TransformStamped>::iterator it;
  // for(it=frames_to_pub.begin(); it!=frames_to_pub.end(); ++it){
  //   // ........
  //   tf_broadcaster_->sendTransform(it->second);
  // }

  scene_cam->captureCloud(); 
     
  if(!scene_cam->poser_tf.header.frame_id.empty())
  {  
    if(scene_cam->poser_marker_.controls.size() == 0)
    {
      scene_cam->initializeMarker();
      server_->insert(scene_cam->poser_marker_);
      server_->setCallback(scene_cam->poser_marker_.name,
      [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
          processFeedback(feedback);
        }
      );
      server_->applyChanges();
    }
    tf_broadcaster_->sendTransform(scene_cam->poser_tf);
    tf_broadcaster_->sendTransform(scene_cam->poser_sensor_tf);
  }
}

void InteractivePoserNode::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{  
  //Pose to Transform conversion is not implemented :(
  // tf2::fromMsg(feedback->pose, scene_cam->poser_tf.transform);
  tf2::Transform tmp;
  tf2::fromMsg(feedback->pose, tmp);
  tf2::toMsg(tmp, scene_cam->poser_tf.transform);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tmp.getRotation()).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(kLogger, "\ncamera_pose:\n\tx: %f \n\ty: %f \n\tz: %f \n\troll: %f \n\tpitch: %f \n\tyaw: %f", // "xyz=: %f %f %f , rpy=: %f %f %f",
           tmp.getOrigin().getX(), tmp.getOrigin().getY(), tmp.getOrigin().getZ(),
           roll, pitch, yaw);

  // use this example for transform multiplication and conversion
  // Eigen::Isometry3d input_to_poi;
  // tf2::fromMsg(user_input_pose.pose, input_to_poi);
  // // calculate the transform from the target frame to the clicked pose
  // const Eigen::Isometry3d target_to_poi = target_to_user_input * input_to_poi;

  // // Update the user_input_pose to the new target frame.
  // geometry_msgs::msg::PoseStamped transformed_user_input_pose;
  // transformed_user_input_pose.header.frame_id = target_frame;
  // transformed_user_input_pose.pose = tf2::toMsg(target_to_poi);
}
}  // namespace interactive_poser