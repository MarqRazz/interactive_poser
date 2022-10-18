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
constexpr auto kCalibratePoseActionName = "calibrate_pose";


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

  // Start the interactive marker server
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    kNodeName,
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_services_interface());

  calibrate_pose_server_ = rclcpp_action::create_server<CalibratePose>(
      node_, kCalibratePoseActionName, 
      [this](rclcpp_action::GoalUUID const& uuid, std::shared_ptr<CalibratePose::Goal const> goal) {
        return onGoalCalibratePose(uuid, std::move(goal));
      },
      [this](std::shared_ptr<GoalHandleCalibratePose> goal_handle) {
        return onCancelCalibratePose(std::move(goal_handle));
      },
      [this](std::shared_ptr<GoalHandleCalibratePose> goal_handle) { 
        onAcceptedCalibratePose(std::move(goal_handle));
      });

  // Create a transform listener so that we can output the door handle pose in the desired frame.  
  std::shared_ptr<tf2_ros::Buffer> transform_buffer = std::make_shared<tf2_ros::Buffer>(
    node_->get_clock(), std::chrono::seconds(kTransformBufferSizeSec));
  tf2_ros::TransformListener transform_listener(*transform_buffer);
  // Wait a little bit to give TF time to update.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  auto cam_poser = std::make_unique<PointcloudCameraPoser>("/scene_camera/depth/color/points", "not used");
  auto success = cam_poser->init("scene_camera", transform_buffer, node_, server_);
  cam_poser->initializeMarker();
  cam_poser->initializeMarkerMenu(menu_handler_);
  cam_poser->triggerSensorCapture();
  server_->insert(cam_poser->getPoserMarker());
  server_->setCallback(cam_poser->getName(),
  [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback) {
      processFeedback(feedback);
    }
  );
  menu_handler_.apply(*server_, cam_poser->getName());
  server_->applyChanges();

  active_posers.emplace_back(std::move(cam_poser));

  // create a timer to update the published transforms
  frame_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                          [this]() { return frameCallback(); }); 

  rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue( true ) );
  node_->set_parameter( simTime );
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr InteractivePoserNode::get_node_base_interface()
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse InteractivePoserNode::onGoalCalibratePose(const rclcpp_action::GoalUUID&,
                                                                   const std::shared_ptr<const CalibratePose::Goal> goal)
{
  // The rclcpp_action server callbacks are mutually-exclusive, so while multiple goal requests can be sent in parallel,
  // the action server will evaluate them one at a time.

  for(auto calibration_pose : goal->estimated_poses)
  {
    if (calibration_pose.header.frame_id.empty() || calibration_pose.child_frame_id.empty())
    {
      RCLCPP_WARN(kLogger, "Rejecting goal request: required fields `header.frame_id` or `child_frame_id` is empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // TODO: we need to validate that IP knows of all the calibration frames or we need to dynamically create them
  }

  // If a new action goal request is received while a different action goal from a previous request is already being
  // executed, the new goal will be rejected. The previous goal must be explicitly canceled before it can be replaced.
  if (has_active_action_goal_)
  {
    RCLCPP_WARN(kLogger, "Rejecting goal request: a previous CalibratePose action goal is already in-progress.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // While rclcpp action servers can handle executing multiple goals in parallel, the server can only
  // execute a single goal at a time, so this function needs to set that a goal is in progress at the same time as it
  // tells the client that the goal has been accepted.
  has_active_action_goal_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse InteractivePoserNode::onCancelCalibratePose(const std::shared_ptr<GoalHandleCalibratePose>)
{
  // TODO remove all move/rotate interactive markers

  return rclcpp_action::CancelResponse::ACCEPT;
}

void InteractivePoserNode::onAcceptedCalibratePose(const std::shared_ptr<GoalHandleCalibratePose> goal_handle)
{
  // Make sure any previous action execution thread has been joined before replacing it with a new one
  if (calibrate_pose_thread_.joinable())
  {
    calibrate_pose_thread_.join();
  }
  // Spin off action execution into a new thread
  calibrate_pose_thread_ = std::thread{ [=]() { onCalibratePose(goal_handle); } };
}

void InteractivePoserNode::onCalibratePose(const std::shared_ptr<GoalHandleCalibratePose> goal_handle)
{
  auto do_calibrate_action_result = std::make_shared<CalibratePose::Result>();

  const auto goal = goal_handle->get_goal();  

  std::atomic<bool> user_approval {false};
  interactive_markers::MenuHandler::EntryHandle approve_menu;
  std::string target_poser_name;
  for(auto calibration_pose : goal->estimated_poses)
  {
    RCLCPP_WARN(kLogger, "Goal: %s", calibration_pose.child_frame_id.c_str());
    // loop through all of the active_posers and see if we know of this calibration frame    
    for(auto& p : active_posers)
    {
      RCLCPP_WARN(kLogger, "Poser: %s", p->getName().c_str());
      if("scene_camera_mount_link" == calibration_pose.child_frame_id)
      {
        target_poser_name = p->getName();
        RCLCPP_WARN(kLogger, "Adding approval menu: %s", p->getName().c_str());
        approve_menu = menu_handler_.insert( "Approve Action",
        [this, &user_approval](const InteractiveMarkerFeedback::ConstSharedPtr msg) 
        { user_approval = approveActionMenuCallback(msg); 
        });
        // add the move controls to signal to the user we want them to estimate this pose
        InteractiveMarker int_marker;
        server_->get(target_poser_name, int_marker);
        addMoveControl(int_marker);
        server_->erase(int_marker.name); //remove our old marker from the server
        server_->insert(int_marker); //insert our updated copy
        menu_handler_.apply(*server_, p->getName());
        server_->applyChanges();
      }
    }
  }

  // TODO should this include a timeout?
  rclcpp::Rate r {std::chrono::seconds(1)};
  while(!user_approval)
  {
    RCLCPP_INFO(kLogger, "Waiting for user approval %s", user_approval ? "true" : "false");
    r.sleep();
  }
  // hide the approve menu
  menu_handler_.setVisible(approve_menu, false);
  menu_handler_.apply(*server_, target_poser_name);
  server_->applyChanges();

  // find the target_poser and add their frames to the result
  for(auto& p : active_posers)
  {
    if(p->getName() == target_poser_name)
      for(auto& frame : p->getPoserFrames())
      {
        do_calibrate_action_result->calibrated_poses.push_back(frame);
      }
  }
  

  // If we make it here, running the calibration succeeded
  has_active_action_goal_ = false;
  RCLCPP_INFO(kLogger, "Success!!!");
  goal_handle->succeed(do_calibrate_action_result);
}

void InteractivePoserNode::frameCallback()
{
  if (!tf_broadcaster_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_->shared_from_this());
  }

  tf2::TimePoint tf_time_point(std::chrono::nanoseconds(node_->get_clock()->now().nanoseconds()));

  // loop through all of the active_posers and publish all of their frames
  for(auto& p : active_posers)
  {
    for(auto& frame : p->getPoserFrames())
    {
      tf_broadcaster_->sendTransform(frame);
    }
  }
}

void InteractivePoserNode::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{  
  //Pose to Transform conversion is not implemented :(
  // tf2::fromMsg(feedback->pose, scene_cam->poser_tf.transform);
  tf2::Transform tmp;
  tf2::fromMsg(feedback->pose, tmp);
  geometry_msgs::msg::TransformStamped tmp2;
  tf2::toMsg(tmp, tmp2.transform);
  active_posers[0]->setPoserFrame(tmp2);
  active_posers[0]->triggerSensorCapture(); // we need to trigger to force Rviz to update the cloud pose.
  // this does not work very well! How should we keep a copy up to date or should everyone just read it from the server?
  InteractiveMarker int_marker;
  server_->get(feedback->marker_name, int_marker);
  active_posers[0]->setPoserMarker(int_marker);// keep our internal copy up to date.
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(tmp.getRotation()).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(kLogger, "\ncamera_pose:\n\tx: %f \n\ty: %f \n\tz: %f \n\troll: %f \n\tpitch: %f \n\tyaw: %f", // "xyz=: %f %f %f , rpy=: %f %f %f",
           tmp.getOrigin().getX(), tmp.getOrigin().getY(), tmp.getOrigin().getZ(),
           roll, pitch, yaw);
  // RCLCPP_WARN(kLogger, "Event type: %i", feedback->event_type);

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

bool InteractivePoserNode::approveActionMenuCallback( 
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  RCLCPP_INFO(kLogger, "User approved!!!");
  // active_posers.front()->setMoveFeedback(feedback); // toggle off the move controls  
  InteractiveMarker int_marker;
  server_->get(feedback->marker_name, int_marker); //grab a copy of the marker from the server

  deleteMoveControl(int_marker); //toggle off the move controls to the interactive marker

  server_->erase(feedback->marker_name); //remove our old marker from the server
  server_->insert(int_marker); //insert our updated copy
  server_->applyChanges();
  return true;
}
}  // namespace interactive_poser