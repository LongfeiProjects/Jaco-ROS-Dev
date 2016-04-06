/*
 * jacolib.h
 *
 *  Created on: Feb 16, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 */

#ifndef JACO_DRIVER_JACO_API_H
#define JACO_DRIVER_JACO_API_H

#include <dlfcn.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>

#include "kinova/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova/KinovaTypes.h"


namespace jaco
{

#define JACO_USB_LIBRARY "Kinova.API.USBCommandLayerUbuntu.so"

class JacoAPI
{
 public:
    JacoAPI(void);

    int (*initAPI)(void);
    int (*closeAPI)(void);
    int (*getAPIVersion)(int Response[API_VERSION_COUNT]);
    int (*getDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &);
    int (*setActiveDevice)(KinovaDevice);

    int (*getGeneralInformations)(GeneralInformations &);
    int (*getQuickStatus)(QuickStatus &);
    // int (*GetForcesInfo)(ForcesInfo &);

    int (*getCodeVersion)(int Response[CODE_VERSION_COUNT]);
    int (*startControlAPI)();
    int (*stopControlAPI)();
    int (*initFingers)();

    int (*moveHome)();

    int (*getCartesianPosition)(CartesianPosition &);
    int (*getAngularPosition)(AngularPosition &);
    int (*getAngularVelocity)(AngularPosition &);
    int (*getCartesianForce)(CartesianPosition &);
    int (*setCartesianForceMinMax)(CartesianInfo, CartesianInfo);
    int (*setCartesianInertiaDamping)(CartesianInfo, CartesianInfo);
    int (*startForceControl)();
    int (*stopForceControl)();
    int (*getAngularForce)(AngularPosition &);
    int (*getAngularCurrent)(AngularPosition &);
    int (*getControlType)(int &);
    int (*getActualTrajectoryInfo)(TrajectoryPoint &);
    int (*getGlobalTrajectoryInfo)(TrajectoryFIFO &);
    int (*getSensorsInfo)(SensorsInfo &);
    int (*setAngularControl)();
    int (*setCartesianControl)();
    int (*restoreFactoryDefault)();
    int (*sendJoystickCommand)(JoystickCommand);
    int (*sendAdvanceTrajectory)(TrajectoryPoint);
    int (*sendBasicTrajectory)(TrajectoryPoint);
    int (*getClientConfigurations)(ClientConfigurations &);
    int (*setClientConfigurations)(ClientConfigurations);
    int (*eraseAllTrajectories)();
    int (*getPositionCurrentActuators)(float Response[POSITION_CURRENT_COUNT]);
    int (*setActuatorPID)(unsigned int, float, float, float);

};

}  // namespace jaco
#endif  // JACO_DRIVER_JACO_API_H
