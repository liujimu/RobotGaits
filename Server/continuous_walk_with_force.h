#ifndef CONTINUOUS_WALK_WITH_FORCE_H
#define CONTINUOUS_WALK_WITH_FORCE_H

#endif // CONTINUOUS_WALK_WITH_FORCE_H


#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_Control.h>
#include <Aris_DynKer.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include <Robot_Type_I.h>

#ifndef PI
#define PI 3.141592653589793
#endif

enum WALK_DIRECTION
{
    STOP,
    FORWARD,
    BACKWARD,
    RIGHTWARD,
    LEFTWARD,
    TURNLEFT,
    TURNRIGHT,
    FAST_TURNLEFT,
    FAST_TURNRIGHT
};

/*parse function*/
Aris::Core::MSG parseCWF(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parseCWFStop(const std::string &cmd, const std::map<std::string, std::string> &params);

/*operation function*/
int continuousWalk(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
int continuousWalkWithForce(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);
WALK_DIRECTION forceJudge(const double *force, const double *threshold);
