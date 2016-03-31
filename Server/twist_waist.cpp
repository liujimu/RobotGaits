#include "twist_waist.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

/*将以下注释代码添加到xml文件*/
/*

*/

auto twistWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
    twParam param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "pitchMax")
        {
            param.pitchMax = stod(i.second) / 180 * PI;
        }
        else if (i.first == "rollMax")
        {
            param.rollMax = stod(i.second) / 180 * PI;
        }
    }

    msg.copyStruct(param);
}

auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const twParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];

    if (param.count%param.totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

    const double s = -PI * cos(PI * (param.count + 1) / param.totalCount) + PI;//s 从0到2*PI.

    Peb[3] = param.rollMax * sin(s);
    Peb[4] = param.pitchMax * (1 - cos(s)) / 2;

    robot.SetPeb(Peb, beginMak);
    robot.SetPee(Pee, beginMak);

    return param.totalCount - param.count - 1;
}
