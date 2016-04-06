//============================================================================
// Name        : jaco_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Jaco robotic manipulator arm
//============================================================================

#include "jaco_driver/jaco_api.h"
#include "jaco_driver/jaco_arm.h"
#include "jaco_driver/jaco_pose_action.h"
#include "jaco_driver/jaco_angles_action.h"
#include "jaco_driver/jaco_fingers_action.h"


int main(int argc, char **argv)
{
int count_error;

    ros::init(argc, argv, "jaco_arm_driver");
ROS_WARN("jaco_arm_driver.cpp: 1, %d", ++count_error);
    ros::NodeHandle nh("~");
ROS_WARN("jaco_arm_driver.cpp: 2, %d", ++count_error);
    boost::recursive_mutex api_mutex;
ROS_WARN("jaco_arm_driver.cpp: 3, %d", ++count_error);

    bool is_first_init = true;
    while (ros::ok())
    {
ROS_WARN("jaco_arm_driver.cpp: 4, %d", ++count_error);
        try
        {
ROS_WARN("jaco_arm_driver.cpp: 5, %d", ++count_error);
            jaco::JacoComm comm(nh, api_mutex, is_first_init);
ROS_WARN("jaco_arm_driver.cpp: 6, %d", ++count_error);
            jaco::JacoArm jaco(comm, nh);
ROS_WARN("jaco_arm_driver.cpp: 7, %d", ++count_error);
            jaco::JacoPoseActionServer pose_server(comm, nh);
ROS_WARN("jaco_arm_driver.cpp: 8, %d", ++count_error);
            jaco::JacoAnglesActionServer angles_server(comm, nh);
ROS_WARN("jaco_arm_driver.cpp: 9, %d", ++count_error);
            jaco::JacoFingersActionServer fingers_server(comm, nh);
ROS_WARN("jaco_arm_driver.cpp: 10, %d", ++count_error);
            ros::spin();
        }
        catch(const std::exception& e)
        {
ROS_WARN("jaco_arm_driver.cpp: 11, %d", ++count_error);
            ROS_ERROR_STREAM(e.what());
ROS_WARN("jaco_arm_driver.cpp: 12, %d", ++count_error);
            jaco::JacoAPI api;
ROS_WARN("jaco_arm_driver.cpp: 13, %d", ++count_error);
            boost::recursive_mutex::scoped_lock lock(api_mutex);
ROS_WARN("jaco_arm_driver.cpp: 14, %d", ++count_error);
            api.closeAPI();
ROS_WARN("jaco_arm_driver.cpp: 15, %d", ++count_error);
            ros::Duration(1.0).sleep();
        }

        is_first_init = false;
    }
    return 0;
}
