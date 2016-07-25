/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: jaco_types.cpp
 *  Desc: Wrappers around Kinova structs to facilitate easier conversion to ROS
 *		  types.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#include <math.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <jaco_driver/jaco_types.h>
#include <string>


namespace jaco
{

// A few helper functions
// ----------------------

float normalizeInRads(float rads)
{
    return static_cast<float>(angles::normalize_angle_positive(rads));
}


float normalizePositiveInDegrees(float degrees)
{
    return angles::to_degrees(angles::normalize_angle_positive(angles::from_degrees(degrees)));
}


float normalizeInDegrees(float degrees)
{
    return angles::to_degrees(angles::normalize_angle(angles::from_degrees(degrees)));
}


bool areValuesClose(float first, float second, float tolerance)
{
    return ((first <= second + tolerance) && (first >= second - tolerance));
}


/**
 * @brief EulerXYZ2Quaternion
 * @param tx input Euler angle tx
 * @param ty input Euler angle ty
 * @param tz input Euler angle tz
 * @return output Quaternion
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
tf::Quaternion EulerXYZ2Quaternion(float tx, float ty, float tz)
{
    float sx = sin(0.5*tx);
    float cx = cos(0.5*tx);
    float sy = sin(0.5*ty);
    float cy = cos(0.5*ty);
    float sz = sin(0.5*tz);
    float cz = cos(0.5*tz);

    float qx, qy, qz, qw;
    qx =  sx*cy*cz + cx*sy*sz;
    qy = -sx*cy*sz + cx*sy*cz;
    qz =  sx*sy*cz + cx*cy*sz;
    qw = -sx*sy*sz + cx*cy*cz;

    tf::Quaternion q;
    q.setX(qx);
    q.setY(qy);
    q.setZ(qz);
    q.setW(qw);
    return q;
}

/**
 * @brief getEulerXYZ get Euler-XYZ convention for KinovaPose orientation from Quaternion convention.
 * @param q input Quaternion
 * @param tx output Euler angle tx
 * @param ty output Euler angle ty
 * @param tz output Euler angle tz
 * @warning DSP(KinovaPose) use Euler-XYZ convention, while ROS use Euler-ZYX by default.
 */
void getEulerXYZ(tf::Quaternion &q, float &tx, float &ty, float &tz)
{
    float qx = q.getX();
    float qy = q.getY();
    float qz = q.getZ();
    float qw = q.getW();

    tx = atan2((2*qw*qx-2*qy*qz),(qw*qw-qx*qx-qy*qy+qz*qz));
    ty = asin(2*qw*qy+2*qx*qz);
    tz = atan2((2*qw*qz-2*qx*qy),(qw*qw+qx*qx-qy*qy-qz*qz));
}


// Exceptions
// ----------
JacoCommException::JacoCommException(const std::string& message, const int error_code)
{
    std::stringstream ss;
        ss << "JacoCommException: " << message << " (return code: " << error_code << ")" << std::endl;
    desc_ = ss.str();
}


const char* JacoCommException::what() const throw()
{
    return desc_.c_str();
}


// Class definitions
// -----------------

JacoPose::JacoPose(const geometry_msgs::Pose &pose)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);

    X = static_cast<float>(pose.position.x);
    Y = static_cast<float>(pose.position.y);
    Z = static_cast<float>(pose.position.z);

    getEulerXYZ(q,ThetaX, ThetaY,ThetaZ);
}


JacoPose::JacoPose(const CartesianInfo &pose)
{
    X = pose.X;
    Y = pose.Y;
    Z = pose.Z;

    ThetaX = normalizeInRads(pose.ThetaX);
    ThetaY = normalizeInRads(pose.ThetaY);
    ThetaZ = normalizeInRads(pose.ThetaZ);
}


/**
 * @brief construct geometric::Pose message from KinovaPose
 * @return geometry_msgs::Pose[x,y,z,qx,qy,qz,qw] position in meters, orientation is in Quaternion.
 */
geometry_msgs::Pose JacoPose::constructPoseMsg()
{
    geometry_msgs::Pose pose;
    tf::Quaternion position_quaternion;

    // However, DSP using Euler-XYZ, while ROS using Euler-ZYX.
    JacoPose::getQuaternion(position_quaternion);
    tf::quaternionTFToMsg(position_quaternion, pose.orientation);

    pose.position.x = X;
    pose.position.y = Y;
    pose.position.z = Z;

    return pose;
}


tf::Quaternion JacoPose::getQuaternion(tf::Quaternion &q)
{
    q = EulerXYZ2Quaternion(ThetaX, ThetaY, ThetaZ);
}


geometry_msgs::Wrench JacoPose::constructWrenchMsg()
{
    geometry_msgs::Wrench wrench;

    wrench.force.x  = X;
    wrench.force.y  = Y;
    wrench.force.z  = Z;
    wrench.torque.x = ThetaX;
    wrench.torque.y = ThetaY;
    wrench.torque.z = ThetaZ;

    return wrench;
}

bool JacoPose::isCloseToOther(const JacoPose &other, float tolerance) const
{
    bool status = true;
    status = status && areValuesClose(X, other.X, tolerance);
    status = status && areValuesClose(Y, other.Y, tolerance);
    status = status && areValuesClose(Z, other.Z, tolerance);
    status = status && areValuesClose(ThetaX, other.ThetaX, tolerance);
    status = status && areValuesClose(ThetaY, other.ThetaY, tolerance);
    status = status && areValuesClose(ThetaZ, other.ThetaZ, tolerance);
    return status;
}


JacoAngles::JacoAngles(const jaco_msgs::JointAngles &angles)
{
    Actuator1 = normalizePositiveInDegrees(180.0 - (angles.joint1 * (180.0 / M_PI)));
    Actuator2 = normalizePositiveInDegrees((angles.joint2 * (180.0 / M_PI)) + 270.0);
    Actuator3 = normalizePositiveInDegrees(90.0 - (angles.joint3 * (180.0 / M_PI)));
    Actuator4 = normalizePositiveInDegrees(180.0 - (angles.joint4 * (180.0 / M_PI)));
    Actuator5 = normalizePositiveInDegrees(180.0 - (angles.joint5 * (180.0 / M_PI)));
    Actuator6 = normalizePositiveInDegrees(270.0 - (angles.joint6 * (180.0 / M_PI)));
}


JacoAngles::JacoAngles(const AngularInfo &angles)
{
    Actuator1 = normalizePositiveInDegrees(angles.Actuator1);
    Actuator2 = normalizePositiveInDegrees(angles.Actuator2);
    Actuator3 = normalizePositiveInDegrees(angles.Actuator3);
    Actuator4 = normalizePositiveInDegrees(angles.Actuator4);
    Actuator5 = normalizePositiveInDegrees(angles.Actuator5);
    Actuator6 = normalizePositiveInDegrees(angles.Actuator6);
}


jaco_msgs::JointAngles JacoAngles::constructAnglesMsg()
{
    jaco_msgs::JointAngles angles;
    angles.joint1 = (180.0 - Actuator1) / (180.0 / M_PI);
    angles.joint2 = (Actuator2 - 270.0) / (180.0 / M_PI);
    angles.joint3 = (90.0 - Actuator3) / (180.0 / M_PI);
    angles.joint4 = (180.0 - Actuator4) / (180.0 / M_PI);
    angles.joint5 = (180.0 - Actuator5) / (180.0 / M_PI);
    angles.joint6 = (270.0 - Actuator6) / (180.0 / M_PI);
    return angles;
}


bool JacoAngles::isCloseToOther(const JacoAngles &other, float tolerance) const
{
    bool status = true;
    status = status && areValuesClose(Actuator1, other.Actuator1, tolerance);
    status = status && areValuesClose(Actuator2, other.Actuator2, tolerance);
    status = status && areValuesClose(Actuator3, other.Actuator3, tolerance);
    status = status && areValuesClose(Actuator4, other.Actuator4, tolerance);
    status = status && areValuesClose(Actuator5, other.Actuator5, tolerance);
    status = status && areValuesClose(Actuator6, other.Actuator6, tolerance);
    return status;
}


FingerAngles::FingerAngles(const jaco_msgs::FingerPosition &position)
{
    Finger1 = position.finger1;
    Finger2 = position.finger2;
    Finger3 = position.finger3;
}


FingerAngles::FingerAngles(const FingersPosition &angle)
{
    Finger1 = angle.Finger1;
    Finger2 = angle.Finger2;
    Finger3 = angle.Finger3;
}


jaco_msgs::FingerPosition FingerAngles::constructFingersMsg()
{
    jaco_msgs::FingerPosition angles;
    angles.finger1 = Finger1;
    angles.finger2 = Finger2;
    angles.finger3 = Finger3;
    return angles;
}


bool FingerAngles::isCloseToOther(const FingerAngles &other, float tolerance) const
{
    bool status = true;
    status = status && areValuesClose(Finger1, other.Finger1, tolerance);
    status = status && areValuesClose(Finger2, other.Finger2, tolerance);
    status = status && areValuesClose(Finger3, other.Finger3, tolerance);
    return status;
}

}  // namespace jaco
