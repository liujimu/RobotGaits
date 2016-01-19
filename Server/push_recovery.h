#ifndef PUSH_RECOVERY_H
#define PUSH_RECOVERY_H

#endif // PUSH_RECOVERY_H

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

/*gait parameters*/
struct PR_PARAM final:public Robots::GAIT_PARAM_BASE
{
    std::int32_t pushCount{1500};
    std::int32_t recoverCount{4000};
    std::int32_t totalCount{5000};
    std::int32_t firstStepCount{2000};
    double d{0.4};//步长
    double h{0.05};//步高
    double angle{5};//
    double descend{0.04};//
};

/*operation function*/
int PushRecovery(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam);

/*parse function*/
Aris::Core::MSG parsePushRecovery(const std::string &cmd, const std::map<std::string, std::string> &params);
Aris::Core::MSG parsePushRecoveryStop(const std::string &cmd, const std::map<std::string, std::string> &params);

/*calculation function*/
double Hermite3(double x, double x1, double x2, double y1, double y2, double m1, double m2);
