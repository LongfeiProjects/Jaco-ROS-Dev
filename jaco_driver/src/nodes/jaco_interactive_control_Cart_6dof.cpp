
// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include "jaco_driver/jaco_api.h"
#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"

#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"
#include <actionlib/client/simple_action_client.h>

#include <algorithm>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <interactive_markers/menu_handler.h>


#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

// Create actionlib client
typedef actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;
typedef actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ArmPose_actionlibClient;
typedef actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> Finger_actionlibClient;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interMark_server;

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(send actionlib goals)%
void sendFingerGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    Finger_actionlibClient client("/mico_arm_driver/fingers/finger_positions", true);
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

        ROS_INFO("client send goal to Finger actionlib: %f \n", goal.fingers.finger1);
}

void sendArmPoseGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ArmPose_actionlibClient client("/mico_arm_driver/arm_pose/arm_pose", true);
    client.waitForServer();

    // limit the translation range of marker to [0 maxMarkerPosition],and orientation to [0 maxMarkerRotation] before mapping to arm pose position
    // original pose of interactive marker
    geometry_msgs::Pose markerPose = feedback->pose;
    // pose of interactive marker with saturation
    geometry_msgs::Pose markerPose_limit;
    float maxMarkerPosition = 2.0f;
    float maxMarkerRotation = 1.0f;

    markerPose_limit.position.x = std::min(maxMarkerPosition, std::max(0.0f,float(markerPose.position.x)));
    markerPose_limit.position.y = std::min(maxMarkerPosition, std::max(0.0f,float(markerPose.position.y)));
    markerPose_limit.position.z = std::min(maxMarkerPosition, std::max(0.0f,float(markerPose.position.z)));

    // map marker position to the position of arm end-effector
    geometry_msgs::Pose HomePose;
    jaco_msgs::ArmPoseGoal goal;
    float position_scale = 0.1;

    HomePose.position.x = 0.557002842426;
    HomePose.position.y = -0.165318846703;
    HomePose.position.z = 0.337297201157;
    HomePose.orientation.x = 0.469718859406;
    HomePose.orientation.y = -0.482891449239;
    HomePose.orientation.z = 0.723616758119;
    HomePose.orientation.w = -0.150195967788;

    goal.pose.header.frame_id = "mico_api_origin";
    goal.pose.pose.position.x = HomePose.position.x + markerPose_limit.position.x*position_scale;
    goal.pose.pose.position.y = HomePose.position.y + markerPose_limit.position.y*position_scale;
    goal.pose.pose.position.z = HomePose.position.z + markerPose_limit.position.z*position_scale;
    goal.pose.pose.orientation.x = HomePose.orientation.x;
    goal.pose.pose.orientation.y = HomePose.orientation.y;
    goal.pose.pose.orientation.z = HomePose.orientation.z;
    goal.pose.pose.orientation.w = HomePose.orientation.w;

    client.sendGoal(goal);

    ROS_INFO("client send goal to arm pose actionlib x: %f\n", goal.pose.pose.position.x);
}

void sendArmJointGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ArmJoint_actionlibClient client("/mico_arm_driver/joiint_angles/arm_joint_angles", true);
    client.waitForServer();

    jaco_msgs::JointAngles currentJointAngles;
    jaco_msgs::JointAngles homeJointAngles;

    // limit the translation range of marker to [0 maxMarkerPosition],and orientation to [0 maxMarkerRotation] before mapping to arm pose position
    // original pose of interactive marker

    // pose of interactive marker with saturation


    // map marker position to the position of arm end-effector

}


// %EndTag(send actionlib goals)%


// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose update" <<  "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
//      ROS_INFO_STREAM( s.str() << ": mouse up"  << "." );
      ROS_INFO_STREAM( s.str() << ": updated pose is: "
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;
  }

  // sendFingerGoal(feedback);
  // sendArmPoseGoal(feedback);
  // sendArmJointGoal(feedback);
  // interMark_server->applyChanges();

}
// %EndTag(processFeedback)%

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "mico_api_origin";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.1;
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  interMark_server->insert(int_marker);
  interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%


////////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
///

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jaco_interactive_control_Cart_6dof");
    ros::NodeHandle nh("~");

    interMark_server.reset( new interactive_markers::InteractiveMarkerServer("jaco_interactive_control_Cart_6dof","",false) );

    ros::Duration(0.1).sleep();

    tf::Vector3 position;
    position = tf::Vector3(0.1, 0.1, 0); // Simple 6-DOF Control
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    interMark_server->applyChanges();

    ros::spin();

    interMark_server.reset();
    return 0;

}



// %Tag(fullSource)%
