/* 步态功能：持续检测力传感器信号，根据主要受力方向调整身体位姿
 * by chenzhijun, 2016-3-30
 */

/*将以下注释代码添加到xml文件*/
/*
      <cmb default="cmb_param">
        <cmb_param type="group">
          <u abbreviation="u" type="double" default="0"/>
          <v abbreviation="v" type="double" default="0"/>
          <w abbreviation="w" type="double" default="0"/>
          <roll abbreviation="r" type="double" default="0"/>
          <pitch abbreviation="p" type="double" default="0"/>
          <yaw abbreviation="y" type="double" default="0"/>
        </cmb_param>
      </cmb>
      <cmj default="cmj_param">
        <cmj_param type="group">
          <isStop abbreviation="s" type="int" default="0"/>
          <isForce abbreviation="f" type="int" default="0"/>
          <u abbreviation="u" type="int" default="0"/>
          <v abbreviation="v" type="int" default="0"/>
          <w abbreviation="w" type="int" default="0"/>
          <roll abbreviation="r" type="int" default="0"/>
          <pitch abbreviation="p" type="int" default="0"/>
          <yaw abbreviation="y" type="int" default="0"/>
        </cmj_param>
      </cmj>
*/

#ifndef CONTINUOUS_MOVE_H
#define CONTINUOUS_MOVE_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct ContinuousMoveParam final :public aris::server::GaitParamBase
{
    std::int32_t move_direction;
};

struct CM_RecordParam
{
    double bodyPE_last[6];
    double bodyVel_last[6];

    double forceSum[6]{0,0,0,0,0,0};
    double forceAvg[6]{0,0,0,0,0,0};
    double force[6];
};

/*parse function*/
void parseContinuousMoveBegin(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
void parseContinuousMoveJudge(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);

/*operation function*/
int continuousMove(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);


#endif // CONTINUOUS_MOVE_H
