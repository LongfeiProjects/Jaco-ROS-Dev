/*
 * jacolib.cpp
 *
 *  Created on: Mar 10, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#include <jaco_driver/jaco_api.h>
#include <vector>


namespace jaco
{

void* checkApiInit(void * usbLib, const char* name)
{
    void * function_pointer = dlsym(usbLib, name);
    assert(function_pointer != NULL);
    return function_pointer;
}


JacoAPI::JacoAPI(void)
{
    // // // ROS_WARN__STREAM("Class name is: " << typeid(*this).name() << "; function name is : " << __FUNCTION__<<std::endl);
    count_error_comm = 0;
    // ROS_WARN_("jaco_api.cpp: 1, %d", ++count_error_comm);

    void *usbLib = dlopen(JACO_USB_LIBRARY, RTLD_NOW | RTLD_GLOBAL);
    if (usbLib == NULL)
    {
        // ROS_WARN_("%s", dlerror());
    }
// ROS_WARN_("jaco_api.cpp: 2, %d", ++count_error_comm);
    initAPI = (int (*)())checkApiInit(usbLib, "InitAPI");
// ROS_WARN_("jaco_api.cpp: 3, %d", ++count_error_comm);
    closeAPI = (int (*)())checkApiInit(usbLib, "CloseAPI");
// ROS_WARN_("jaco_api.cpp: 4, %d", ++count_error_comm);
    getAPIVersion = (int (*)(int[API_VERSION_COUNT]))checkApiInit(usbLib, "GetAPIVersion");
// ROS_WARN_("jaco_api.cpp: 5, %d", ++count_error_comm);
    getDevices = (int (*)(KinovaDevice[MAX_KINOVA_DEVICE], int &))checkApiInit(usbLib, "GetDevices");
// ROS_WARN_("jaco_api.cpp: 6, %d", ++count_error_comm);
    setActiveDevice = (int (*)(KinovaDevice))checkApiInit(usbLib, "SetActiveDevice");
// ROS_WARN_("jaco_api.cpp: 7, %d", ++count_error_comm);
    getCodeVersion = (int (*)(int[CODE_VERSION_COUNT]))checkApiInit(usbLib, "GetCodeVersion");
// ROS_WARN_("jaco_api.cpp: 8, %d", ++count_error_comm);
    getGeneralInformations = (int (*)(GeneralInformations &))checkApiInit(usbLib, "GetGeneralInformations");
// ROS_WARN_("jaco_api.cpp: 9, %d", ++count_error_comm);
    getCartesianPosition = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianPosition");
// ROS_WARN_("jaco_api.cpp: 10, %d", ++count_error_comm);
    getAngularPosition = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularPosition");
// ROS_WARN_("jaco_api.cpp: 11, %d", ++count_error_comm);
    getAngularVelocity = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularVelocity");
// ROS_WARN_("jaco_api.cpp: 12, %d", ++count_error_comm);
    getCartesianForce = (int (*)(CartesianPosition &))checkApiInit(usbLib, "GetCartesianForce");
// ROS_WARN_("jaco_api.cpp: 13, %d", ++count_error_comm);
    setCartesianForceMinMax = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianForceMinMax");
// ROS_WARN_("jaco_api.cpp: 14, %d", ++count_error_comm);
    setCartesianInertiaDamping = (int (*)(CartesianInfo, CartesianInfo))checkApiInit(usbLib, "SetCartesianInertiaDamping");
// ROS_WARN_("jaco_api.cpp: 15, %d", ++count_error_comm);
    startForceControl = (int (*)())checkApiInit(usbLib, "StartForceControl");
// ROS_WARN_("jaco_api.cpp: 16, %d", ++count_error_comm);
    stopForceControl = (int (*)())checkApiInit(usbLib, "StopForceControl");
// ROS_WARN_("jaco_api.cpp: 17, %d", ++count_error_comm);
    getAngularForce = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularForce");
// ROS_WARN_("jaco_api.cpp: 18, %d", ++count_error_comm);
    getAngularCurrent = (int (*)(AngularPosition &))checkApiInit(usbLib, "GetAngularCurrent");
// ROS_WARN_("jaco_api.cpp: 19, %d", ++count_error_comm);
    getActualTrajectoryInfo = (int (*)(TrajectoryPoint &))checkApiInit(usbLib, "GetActualTrajectoryInfo");
// ROS_WARN_("jaco_api.cpp: 20, %d", ++count_error_comm);
    getGlobalTrajectoryInfo = (int (*)(TrajectoryFIFO &))checkApiInit(usbLib, "GetGlobalTrajectoryInfo");
// ROS_WARN_("jaco_api.cpp: 21, %d", ++count_error_comm);
    getSensorsInfo = (int (*)(SensorsInfo &))checkApiInit(usbLib, "GetSensorsInfo");
// ROS_WARN_("jaco_api.cpp: 22, %d", ++count_error_comm);
    setAngularControl = (int (*)())checkApiInit(usbLib, "SetAngularControl");
// ROS_WARN_("jaco_api.cpp: 23, %d", ++count_error_comm);
    setCartesianControl = (int (*)())checkApiInit(usbLib, "SetCartesianControl");
// ROS_WARN_("jaco_api.cpp: 24, %d", ++count_error_comm);
    startControlAPI = (int (*)())checkApiInit(usbLib, "StartControlAPI");
// ROS_WARN_("jaco_api.cpp: 25, %d", ++count_error_comm);
    stopControlAPI = (int (*)())checkApiInit(usbLib, "StopControlAPI");
// ROS_WARN_("jaco_api.cpp: 26, %d", ++count_error_comm);
    moveHome = (int (*)())checkApiInit(usbLib, "MoveHome");
// ROS_WARN_("jaco_api.cpp: 27, %d", ++count_error_comm);
    initFingers = (int (*)())checkApiInit(usbLib, "InitFingers");
// ROS_WARN_("jaco_api.cpp: 28, %d", ++count_error_comm);
    restoreFactoryDefault = (int (*)())checkApiInit(usbLib, "RestoreFactoryDefault");
// ROS_WARN_("jaco_api.cpp: 29, %d", ++count_error_comm);
    sendJoystickCommand = (int (*)(JoystickCommand))checkApiInit(usbLib, "SendJoystickCommand");
// ROS_WARN_("jaco_api.cpp: 30, %d", ++count_error_comm);
    sendAdvanceTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendAdvanceTrajectory");
// ROS_WARN_("jaco_api.cpp: 31, %d", ++count_error_comm);
    sendBasicTrajectory = (int (*)(TrajectoryPoint))checkApiInit(usbLib, "SendBasicTrajectory");
// ROS_WARN_("jaco_api.cpp: 32, %d", ++count_error_comm);
    getControlType = (int (*)(int &)) checkApiInit(usbLib, "GetControlType");
// ROS_WARN_("jaco_api.cpp: 33, %d", ++count_error_comm);
    getQuickStatus = (int (*)(QuickStatus &))checkApiInit(usbLib, "GetQuickStatus");
// ROS_WARN_("jaco_api.cpp: 34, %d", ++count_error_comm);
    getClientConfigurations = (int (*)(ClientConfigurations &))checkApiInit(usbLib, "GetClientConfigurations");
// ROS_WARN_("jaco_api.cpp: 35, %d", ++count_error_comm);
    setClientConfigurations = (int (*)( ClientConfigurations))checkApiInit(usbLib, "SetClientConfigurations");
// ROS_WARN_("jaco_api.cpp: 36, %d", ++count_error_comm);
    eraseAllTrajectories = (int (*)())checkApiInit(usbLib, "EraseAllTrajectories");
// ROS_WARN_("jaco_api.cpp: 37, %d", ++count_error_comm);
    getPositionCurrentActuators = (int (*)(float[POSITION_CURRENT_COUNT]))checkApiInit(usbLib, "GetPositionCurrentActuators");
// ROS_WARN_("jaco_api.cpp: 38, %d", ++count_error_comm);
    setActuatorPID = (int (*)(unsigned int, float, float, float))checkApiInit(usbLib, "SetActuatorPID");
// ROS_WARN_("jaco_api.cpp: 39, %d", ++count_error_comm);
}

}  // namespace jaco
