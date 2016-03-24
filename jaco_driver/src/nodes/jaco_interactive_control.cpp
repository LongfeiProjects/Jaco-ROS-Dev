
// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>


#include "jaco_driver/jaco_api.h"
#include "jaco_driver/jaco_arm.h"

#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"
#include <actionlib/client/simple_action_client.h>
#include "jaco_driver/jaco_fingers_action.h"

#include <algorithm>

// Create actionlib client
typedef actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> FingerClient;


//void processFeedback();
void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
//  ROS_INFO_STREAM( feedback->marker_name << " is now at "
//      << feedback->pose.position.x << ", " << feedback->pose.position.y
//      << ", " << feedback->pose.position.z );

  FingerClient client("/mico_arm_driver/fingers/finger_positions", true);
  jaco_msgs::SetFingersPositionGoal goal;

  client.waitForServer();
  // limit the range of marker to [0 10] before mapping to finger position
  float markerPos;
  float maxMarkerPose = 2.0f;
  markerPos = std::min(maxMarkerPose, std::max(0.0f,float(feedback->pose.position.x)));
  // map marker position to gripper position
  goal.fingers.finger1 = markerPos/maxMarkerPose*5000;
  goal.fingers.finger2 = markerPos/maxMarkerPose*5000;
  goal.fingers.finger3 = 0.0;
  client.sendGoal(goal);

      ROS_INFO("client send goal: %f \n", goal.fingers.finger1);

//  client.waitForResult(ros::Duration(1.0));
//  if(client.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
//      printf("Current State: %s\n", client.getState().toString().c_str());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_interactive_control");
  ros::NodeHandle nh("~");
  boost::recursive_mutex api_mutex;

  bool is_first_init = true;




  // create an interactive marker server on the topic namespace jaco_interactive_control
  interactive_markers::InteractiveMarkerServer server("jaco_interactive_control");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.name = "move_x";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);


  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
 ros::spin();




//  while (ros::ok())
//  {
//      try
//      {
//          jaco::JacoComm comm(nh, api_mutex, is_first_init);
//          jaco::JacoArm jaco(comm, nh);

//          ros::spin();
//      }
//      catch(const std::exception& e)
//      {
//          ROS_ERROR_STREAM(e.what());
//          jaco::JacoAPI api;
//          boost::recursive_mutex::scoped_lock lock(api_mutex);
//          api.closeAPI();
//          ros::Duration(1.0).sleep();
//      }

//      is_first_init = false;
//  }

  return 0;



}



// %Tag(fullSource)%
