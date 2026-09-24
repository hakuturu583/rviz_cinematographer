/** @file
 *
 * Helper functions for plugins.
 *
 * @author Jan Razlaw
 */

#ifndef RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H
#define RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H

#include <cmath>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

namespace rviz2_cinematographer_gui
{

/**
 * @breif Creates box marker.
 *
 * @param[in] scale     scale.
 * @return Box marker.
 */
inline visualization_msgs::msg::Marker makeBox(float scale)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;
  marker.scale.x = scale * 0.15;
  marker.scale.y = scale * 0.45;
  marker.scale.z = scale * 0.25;
  marker.color.r = 0.f;
  marker.color.g = 0.f;
  marker.color.b = 0.f;
  marker.color.a = 1.f;
  return marker;
}

/**
 * @breif Creates arrow marker.
 *
 * @param[in] scale     scale.
 * @return Arrow marker.
 */
inline visualization_msgs::msg::Marker makeArrow(float scale)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.orientation.w = M_SQRT1_2;
  marker.pose.orientation.y = M_SQRT1_2;
  marker.scale.x = scale * 0.7;
  marker.scale.y = scale * 0.1;
  marker.scale.z = scale * 0.1;
  marker.color.r = 1.f;
  marker.color.g = 1.f;
  marker.color.b = 1.f;
  marker.color.a = 0.6f;
  return marker;
}

/**
 * @breif Augments marker with control.
 *
 * @param[in,out] marker    marker that is augmented with control.
 */
inline void makeBoxControl(visualization_msgs::msg::InteractiveMarker& marker)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation.w = 1.0;
  control.markers.push_back(makeBox(marker.scale));
  control.markers.push_back(makeArrow(marker.scale));
  marker.controls.push_back(control);
  marker.controls.back().interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  marker.controls.back().name = "submit_button";
}

}

#endif //RVIZ2_CINEMATOGRAPHER_GUI_UTILS_H
