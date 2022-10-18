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
#include <rclcpp_action/create_server.hpp>

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
#include <interactive_poser/poser.hpp>
#include <interactive_poser/pointcloud_camera_poser.hpp>

#include <robot_calibration_msgs/action/calibrate_pose.hpp>

#include <map>

namespace
{
using CalibratePose = robot_calibration_msgs::action::CalibratePose;
using GoalHandleCalibratePose = rclcpp_action::ServerGoalHandle<CalibratePose>;
using namespace visualization_msgs::msg;
}  // namespace

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
  /**
   * @brief Callback function called by do_objective_server_ when a CalibratePose action goal request is received.
   * @details Sets @ref has_active_action_goal_ = true when the goal request is accepted.
   *
   * @param uuid Goal UUID (ignored since only one goal request will be executed at a time).
   * @param goal Goal message
   * @return Returns rclcpp_action::GoalResponse::REJECT if the goal message's predefined_objective_name field is empty,
   * or if ObjectiveServerNode was already executing a CalibratePose goal.
   * Returns rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE otherwise.
   */
  rclcpp_action::GoalResponse onGoalCalibratePose(const rclcpp_action::GoalUUID& uuid,
                                                const std::shared_ptr<const CalibratePose::Goal> goal);

  /**
   * @brief Callback function called by do_objective_server_ when a CalibratePose action cancellation request is received.
   * @details This calls ObjectiveServer::haltTree() on objective_server_.
   *
   * @param goal_handle Goal handle to cancel (ignored since only one goal request will be executed at a time)
   * @return Returns rclcpp_action::CancelResponse::REJECT if the behavior tree in objective_server_ is not currently
   * running, since there is nothing to cancel in that case. Otherwise, returns rclcpp_action::CancelResponse::ACCEPT.
   */
  rclcpp_action::CancelResponse onCancelCalibratePose(const std::shared_ptr<GoalHandleCalibratePose> goal_handle);

  /**
   * @brief Callback function called by do_objective_server_ when starting to execute a CalibratePose action goal.
   * @details Joins any existing do_objective_thread_ and replaces it with a new thread that will run
   * ObjectiveServerNode::onCalibratePose.
   *
   * @param goal_handle Goal handle to execute.
   */
  void onAcceptedCalibratePose(const std::shared_ptr<GoalHandleCalibratePose> goal_handle);

  /**
   * @brief Execution function for CalibratePose action goals.
   * @details We run this function in a separate thread that is stored in do_objective_thread_.
   *
   * This function does the work to create and tick the behavior tree associated with the Objective:
   * - Loads XML text associated with the named Objective in the CalibratePose goal message.
   * - Creates a new behavior tree from this XML text.
   * - Runs the tree until it finishes or an error result is received.
   *
   * The CalibratePose action goal_handle will be aborted if:
   * - Retrieval of XML text for the specified Objective does not succeed.
   * - The behavior tree cannot be created from the XML text.
   * - The behavior tree is cancelled before it completed.
   * - The behavior tree returns an unexpected error state.
   * - The behavior tree finishes without returning BT::NodeStatus::SUCCESS.
   *
   * Otherwise, the CalibratePose action goal_handle will succeed.
   *
   * has_active_action_goal_ will be set to false when this function completes.
   *
   * @param goal_handle CalibratePose action goal_handle to execute
   */
  void onCalibratePose(const std::shared_ptr<GoalHandleCalibratePose> goal_handle);

  void frameCallback();

  void processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  bool approveActionMenuCallback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  /**
   * @brief RCLCPP node for InteractivePoserNode
   */
  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  std::shared_ptr<rclcpp_action::Server<CalibratePose>> calibrate_pose_server_;

  std::vector<std::unique_ptr<Poser>> active_posers;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr frame_timer_;

  /**
   * @brief Set to true while a CalibratePose action goal has been accepted and is in progress.
   */
  std::atomic_bool has_active_action_goal_{ false };

  /**
   * @brief The @ref onCalibratePose function runs within this thread.
   */
  std::thread calibrate_pose_thread_;

  // std::map<std::string, geometry_msgs::msg::TransformStamped> frames_to_pub;
  // std::map<std::string, geometry_msgs::msg::TransformStamped> frames_to_sub;


};
}  // namespace interactive_poser
