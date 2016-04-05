#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jaco_msgs/HomeArm.h>
#include <jaco_msgs/JointAngles.h>
#include <jaco_msgs/ArmJointAnglesAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_home_test");
    ros::NodeHandle nh;

    // send request to home arm
    ros::ServiceClient home_arm_client = nh.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");

    // send request to move away from home
    ArmJoint_actionlibClient move_arm_client("/mico_arm_driver/joint_angles/arm_joint_angles", true);
    jaco_msgs::ArmJointAnglesGoal goal;
//    jaco_msgs::JointAngles homeAngle;
//    goal.angles = homeAngle + angle_offset;


    jaco_msgs::HomeArm homeArm_srv;
    int count = 0;
    int countMax = 250;


    ROS_INFO(" Homming arm counting: ");
    while(ros::ok() && count<countMax)
    {
        ros::Duration(1).sleep();

        // add some test offset to drive away from home, to save time and space, offset is small.
        goal.angles.joint1 = -1.66 + rand()/RAND_MAX*M_PI/180;
        goal.angles.joint2 = -1.65 + rand()/RAND_MAX*M_PI/180;
        goal.angles.joint3 = 0.19 + rand()/RAND_MAX*M_PI/180;
        goal.angles.joint4 = -1.11 + rand()%10*M_PI/180;
        goal.angles.joint5 = 1.67 + rand()%10*M_PI/180;
        goal.angles.joint6 = 3.22 + rand()%10*M_PI/180;
        move_arm_client.waitForServer();

        move_arm_client.sendGoal(goal);
        move_arm_client.waitForResult();

        if (home_arm_client.call(homeArm_srv))
        {
            ROS_INFO(" %d, ", ++count);
        }
        else
        {
            ROS_WARN("\nCalling Home Arm Service failed at attemp: %d \n", ++count);
            break;

        }
    }

    return 0;

}


