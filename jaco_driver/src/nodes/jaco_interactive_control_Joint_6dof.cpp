
// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"
#include "jaco_driver/jaco_api.h"
#include "jaco_driver/jaco_arm.h"

#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"
#include <actionlib/client/simple_action_client.h>


#include <algorithm>
#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

// Create actionlib client
typedef actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;
typedef actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ArmPose_actionlibClient;
typedef actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> Finger_actionlibClient;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armPose_interMark_server;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armJoint_interMark_server;
interactive_markers::MenuHandler menu_handler;
sensor_msgs::JointState current_joint_state;
// %EndTag(vars)%

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.25;
  marker.scale.y = msg.scale * 0.25;
  marker.scale.z = msg.scale * 0.25;
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

void sendArmPoseGoal(geometry_msgs::PoseStamped &endeffector_pose)
{
    ArmPose_actionlibClient client("/mico_arm_driver/arm_pose/arm_pose", true);
    jaco_msgs::ArmPoseGoal goal;
    client.waitForServer();

    goal.pose = endeffector_pose;

    ROS_INFO_STREAM("Goal parent frame is : " << goal.pose.header.frame_id);

    ROS_INFO_STREAM("Goal to arm pose actionlib: \n"
                    << "  x: " << goal.pose.pose.position.x
                    << ", y: " << goal.pose.pose.position.y
                    << ", z: " << goal.pose.pose.position.z
                    << ", qx: " << goal.pose.pose.orientation.x
                    << ", qy: " << goal.pose.pose.orientation.y
                    << ", qz: " << goal.pose.pose.orientation.z
                    << ", qw: " << goal.pose.pose.orientation.w);

    // Please be noted that a bug for cartesian position control exists in lower lever control. Therefore, though command (goal) is correctly sent, robot is not able to reach desired position yet. The command(goal) can be verified by: rostopic echo -n 1 (...)/out/tool_position.
    client.sendGoal(goal);

}

void sendArmJointGoal(const std::string marker_name, double joint_offset)
{
    ArmJoint_actionlibClient client("/mico_arm_driver/joint_angles/arm_joint_angles", true);
    jaco_msgs::ArmJointAnglesGoal goal;
    goal.angles.joint1 = current_joint_state.position[0];
    goal.angles.joint2 = current_joint_state.position[1];
    goal.angles.joint3 = current_joint_state.position[2];
    goal.angles.joint4 = current_joint_state.position[3];
    goal.angles.joint5 = current_joint_state.position[4];
    goal.angles.joint6 = current_joint_state.position[5];

    client.waitForServer();

    // map marker position to the position of arm joint.
    switch(*marker_name.rbegin()-'0')
    {
    case 1:
        goal.angles.joint1 -= joint_offset;
        break;
    case 2:
        goal.angles.joint2 += joint_offset;
        break;
    case 3:
        goal.angles.joint3 -= joint_offset;
        break;
    case 4:
        goal.angles.joint4 -= joint_offset;
        break;
    case 5:
        goal.angles.joint5 -= joint_offset;
        break;
    case 6:
        goal.angles.joint6 -= joint_offset;
        break;
    }

    ROS_INFO( " current joint state is as : %f, %f, %f, %f, %f, %f\n",  current_joint_state.position[0], current_joint_state.position[1], current_joint_state.position[2], current_joint_state.position[3], current_joint_state.position[4], current_joint_state.position[5]);

    ROS_INFO( " joint goal is set as : %f, %f, %f, %f, %f, %f\n",  goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);

    client.sendGoal(goal);

}


// %EndTag(send actionlib goals)%


// %Tag(processFeedback)%
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "mico_link_base";

    tf::Quaternion quaternion_mousedown, quaternion_mouseup;

    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";


    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        ROS_INFO_STREAM("mouse click now...");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        double roll_mousedown, pitch_mousedown, yaw_mousedown;
        tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mousedown);
        tf::Matrix3x3(quaternion_mousedown).getRPY(roll_mousedown, pitch_mousedown, yaw_mousedown);

        if (feedback->marker_name == "cartesian_6dof")
        {
            ROS_INFO_STREAM("cartesian_6dof control mode.");
            ROS_INFO_STREAM( s.str() << ": mouse DOWN (refers to current status): "
                             << "\nposition = "
                             << feedback->pose.position.x
                             << ", " << feedback->pose.position.y
                             << ", " << feedback->pose.position.z
                             << "\norientation = "
                             << "w: " << feedback->pose.orientation.w
                             << ", x: " << feedback->pose.orientation.x
                             << ", y: " << feedback->pose.orientation.y
                             << ", z: " << feedback->pose.orientation.z
                             << "\nRollPitchYaw = "
                             << ": Roll: " << roll_mousedown
                             << ", Pitch: " << pitch_mousedown
                             << ", Yaw: " << yaw_mousedown
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec" );
        }
        else
        {
            ROS_INFO_STREAM("Joint control is mode.");
            ROS_INFO_STREAM( s.str() << ": mouse DOWN (refers to current status): "
                             << feedback->marker_name
                             << ": " << yaw_mousedown
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec" );
        }

        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        double roll_mouseup, pitch_mouseup, yaw_mouseup;
        tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mouseup);
        tf::Matrix3x3(quaternion_mouseup).getRPY(roll_mouseup, pitch_mouseup, yaw_mouseup);

        if (feedback->marker_name == "cartesian_6dof")
        {
            tf::TransformListener transform_listener;
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped pose_endeffector;
            pose_endeffector.header.frame_id = "mico_link_base";
            pose_endeffector.header.stamp = ros::Time::now();

            try{
                ros::Time now = ros::Time::now();
                transform_listener.waitForTransform("mico_link_base","mico_end_effector", now, ros::Duration(3.0));
                transform_listener.lookupTransform("mico_link_base","mico_end_effector", ros::Time(0), transform);
            }
            catch(tf::TransformException exception)
            {
                ROS_ERROR("Error occured during transformation from mico_end_effector to mico_link_base: %s", exception.what());
                ros::Duration(1.0).sleep();
            }
            pose_endeffector.pose.position.x = transform.getOrigin().x();
            pose_endeffector.pose.position.y = transform.getOrigin().y();
            pose_endeffector.pose.position.z = transform.getOrigin().z();
            tf::quaternionTFToMsg(transform.getRotation().normalize(), pose_endeffector.pose.orientation);

            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                             << "\nposition = "
                             << feedback->pose.position.x
                             << ", " << feedback->pose.position.y
                             << ", " << feedback->pose.position.z
                             << "\norientation = "
                             << "w: " << feedback->pose.orientation.w
                             << ", x: " << feedback->pose.orientation.x
                             << ", y: " << feedback->pose.orientation.y
                             << ", z: " << feedback->pose.orientation.z
                             << "\nRollPitchYaw = "
                             << ": Roll: " << roll_mouseup
                             << ", Pitch: " << pitch_mouseup
                             << ", Yaw: " << yaw_mouseup
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec" );


            ROS_INFO_STREAM("POSE_endeffector parent frame is : " << pose_endeffector.header.frame_id);
            sendArmPoseGoal(pose_endeffector);
            // armPose_interMark_server->applyChanges();
        }
        else
        {
            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                             << feedback->marker_name
                             << ": " << yaw_mouseup
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec");
            sendArmJointGoal(feedback->marker_name, yaw_mouseup-yaw_mousedown);
            //   armJoint_interMark_server->applyChanges();
        }

        // sendFingerGoal(feedback);

        break;
    }


}
// %EndTag(processFeedback)%

// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "mico_end_effector";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 0.1;
  int_marker.name = "cartesian_6dof";
  int_marker.description = "6-DOF Cartesian Control";

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

  armPose_interMark_server->insert(int_marker);
//  armPose_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
  armPose_interMark_server->setCallback(int_marker.name, &processFeedback);
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%


// %Tag(1DOF)%
void make1DofMarker(const std::string& frame_id, const std::string& axis, unsigned int interaction_mode, const tf::Vector3& position, const std::string& description, const std::string& name)
{

    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.08;
    int_marker.name = name;
    int_marker.description = description;

    // insert a box
    makeBoxControl(int_marker);
    InteractiveMarkerControl control;

    if (interaction_mode == InteractiveMarkerControl::ROTATE_AXIS)
        control.name = "rotate";
    else if (interaction_mode == InteractiveMarkerControl::MOVE_AXIS)
        control.name = "move";
    else
        ROS_INFO("interactive mode should be eigher ROTATE_AXIS or MOVE_AXIS");

    if (axis == "x")
    {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name += "_x";
    }
    else if (axis == "y")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name += "_y";
    }
    else if (axis == "z")
    {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name += "_z";
    }
    else
        ROS_INFO("\n The rotation axis must be x, y or z. \n");

    control.interaction_mode = interaction_mode;
    int_marker.controls.push_back(control);

    armJoint_interMark_server->insert(int_marker);
//    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback);
    //  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    //    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%position


// %Tag(CurrentJoint)%
void currentJointsFeedback(const sensor_msgs::JointStateConstPtr joint_state)
{
    std::vector<std::string> joint_names_;
    joint_names_.resize(jaco::JACO_JOINTS_COUNT);
    joint_names_[0] =  "joint_1";
    joint_names_[1] =  "joint_2";
    joint_names_[2] =  "joint_3";
    joint_names_[3] =  "joint_4";
    joint_names_[4] =  "joint_5";
    joint_names_[5] =  "joint_6";
    joint_names_[6] =  "joint_finger_1";
    joint_names_[7] =  "joint_finger_2";
    joint_names_[8] =  "joint_finger_3";
    current_joint_state.name = joint_names_;

    current_joint_state.header.stamp = ros::Time::now();
    current_joint_state.position.resize(9);
    current_joint_state.position = joint_state->position;
    current_joint_state.velocity = joint_state->velocity;
    current_joint_state.effort = joint_state->effort;

}
// %EndTag(CurrentJoint)%

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

    ros::Subscriber armJoint_sub = nh.subscribe("/mico_arm_driver/out/joint_state", 1, &currentJointsFeedback);

    armPose_interMark_server.reset( new interactive_markers::InteractiveMarkerServer("jaco_interactive_control_Cart_6dof","",false) );
    armJoint_interMark_server.reset( new interactive_markers::InteractiveMarkerServer("jaco_interactive_control_Joint_6dof","",false) );

    ros::Duration(0.1).sleep();


    // %Tag(CreatInteractiveMarkers)%
    tf::Vector3 position;
    position = tf::Vector3(0, 0, 0); // Simple 6-DOF Control
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );

    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_1", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "1st Axis", "marker_joint1");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_2", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "2nd Axis", "marker_joint2");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_3", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "3rd Axis", "marker_joint3");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_4", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "4th Axis", "marker_joint4");
    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_5", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "5th Axis", "marker_joint5");    position = tf::Vector3(0, 0, 0);
    make1DofMarker("mico_link_hand", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "6th Axis", "marker_joint6");
    // %EndTag(CreatInteractiveMarkers)%



    armPose_interMark_server->applyChanges();
    armJoint_interMark_server->applyChanges();

    ros::spin();

    armPose_interMark_server.reset();
    armJoint_interMark_server.reset();

    return 0;

}


