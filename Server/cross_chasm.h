/* 步态功能：跨沟
* by liujimu, 2016-7-7
*/

/*将以下注释代码添加到xml文件*/
/*
            <cc default="cc_param">
                <cc_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <chasmWidth abbreviation="d" type="double" default="1"/>
                    <stepHeight abbreviation="h" type="double" default="0.05"/>
                </cc_param>
            </cc>
*/

#ifndef CROSS_CHASM_H
#define CROSS_CHASM_H

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

struct ccParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 2000 };
	double chasmWidth{ 1 };
	double stepHeight{ 0.05 };
};

auto crossChasmParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto crossChasmGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

struct wvParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 2000 };
	int legSequence{ 0 };//迈腿次序，0、1、2；-1表示不迈腿
	double stepHeight{ 0.05 };
	double stepLength{ 0 };
	double stepWidth{ 0 };
	double targetPeb[6]{ 0 };
};

auto waveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CROSS_CHASM_H
