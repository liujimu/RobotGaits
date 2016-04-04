#ifndef SAY_HELLO_H
#define SAY_HELLO_H

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

struct shParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 3000 };
	double stepLength{ 0.2 };
	double height{ 0.05 };
	double pitchMax{ PI / 12 };
	int helloTimes{ 1 };
};

auto sayHelloParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto sayHelloGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // SAY_HELLO_H
