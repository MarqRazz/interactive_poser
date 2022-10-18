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

#include <interactive_poser/poser.hpp>

namespace
{
constexpr auto kNodeName = "interactive_poser_node";
const auto kLogger = rclcpp::get_logger(kNodeName);

using namespace visualization_msgs::msg;
}

namespace interactive_poser
{

void Poser::initializeMarkerMenu( interactive_markers::MenuHandler& menu_handler )
{
  menu_handler.insert( "Move Layout",
    [this](const InteractiveMarkerFeedback::ConstSharedPtr msg) 
    { setMoveFeedback(msg); 
    });
}

void Poser::setMoveFeedback( const InteractiveMarkerFeedback::ConstSharedPtr feedback )
{
  InteractiveMarker int_marker;
  int_marker_server_->get(feedback->marker_name, int_marker); //grab a copy of the marker from the server

  addMoveControl(int_marker); //toggle on/off the move controls to the interactive marker

  int_marker_server_->erase(feedback->marker_name); //remove our old marker from the server
  int_marker_server_->insert(int_marker); //insert our updated copy
  int_marker_server_->applyChanges();
}

void addMoveControl( InteractiveMarker &i_marker )
{
  // if we already have controls all we want to do is remove them
  if (deleteMoveControl(i_marker))
    return;

  InteractiveMarkerControl control;
  control.always_visible = false;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
 control.name = "rotate_x";
 control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
 i_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
 control.name = "rotate_z";
 control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
 i_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
 control.name = "rotate_y";
 control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
 i_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  i_marker.controls.push_back(control);
}

bool deleteMoveControl( InteractiveMarker &i_marker )
{
  //remove all move/rotate controls before we add more
  std::string move = "move";
  std::string rotate = "rotate";
  int i = 0;
  int num_controls = i_marker.controls.size();
  bool controls_found = false;
  while( i< num_controls)
  {
    std::string n = i_marker.controls[i].name;
    std::size_t found = n.find(move);
    if (found!=std::string::npos)
    {
      i_marker.controls.erase(i_marker.controls.begin() + i);
      num_controls--;
      controls_found = true;
      continue;
    }
    found = n.find(rotate);
    if (found!=std::string::npos)
    {
      i_marker.controls.erase(i_marker.controls.begin() + i);
      num_controls--;
      controls_found = true;
      continue;
    }
    i++;
  }
  return controls_found;
}

}  // namespace interactive_poser