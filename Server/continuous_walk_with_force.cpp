#include "continuous_walk_with_force.h"

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
      <cwf default="cwf_param">
        <cwf_param type="group">
          <totalCount abbreviation="t" type="int" default="3000"/>
          <distance abbreviation="d" type="double" default="0.5"/>
          <height abbreviation="h" type="double" default="0.05"/>
        </cwf_param>
      </cwf>
      <cwfs/>
*/

auto CWFParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)->void
{
    Robots::WalkParam  param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.d = std::stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = std::stod(i.second);
        }
    }

    state::getState().isStopping() = false;

    msg.copyStruct(param);
}

auto CWFStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)->void
{
    state::getState().isStopping() = true;
}

auto CWFGait(Aris::Dynamic::Model &model, const Aris::Dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const Robots::WalkParam &>(param_in);

    static bool isWalking{ false };
    static int walkBeginCount{ 0 };
    static double forceOffsetSum[6]{ 0 };

    double forceOffsetAvg[6]{ 0 };
    double realForceData[6]{ 0 };
    double forceInBody[6];
    const double forceThreshold[6]{ 40, 40, 40, 40, 40, 40 };//力传感器的触发阈值,单位N或Nm
    const double forceAMFactor{ 1 };//传感器数值与实际力大小的转化系数
    const double sensorPE[6]{ 0, 0, 0, 0, -PI/2, -PI/2 };

    //力传感器手动清零
    if (param.count < 100)
    {
        if(param.count == 0)
        {
            std::fill(forceOffsetSum, forceOffsetSum + 6, 0);
        }
        for(int i = 0; i < 6; i++)
        {
            forceOffsetSum[i] += param.force_data->at(0).fce[i];
        }
    }
    else
    {
        for(int i = 0; i < 6; i++)
        {
            forceOffsetAvg[i] = forceOffsetSum[i] / 100;
            realForceData[i]=(param.force_data->at(0).fce[i] - forceOffsetAvg[i]) / forceAMFactor;
            //转换到机器人身体坐标系
            double sensorPM[16]{ 0 };
            Aris::Dynamic::s_pe2pm(sensorPE, sensorPM);
            Aris::Dynamic::s_pm_dot_v3(sensorPM, realForceData, forceInBody);
            Aris::Dynamic::s_pm_dot_v3(sensorPM, realForceData + 3, forceInBody + 3);
        }
        //用于显示力的初始值
        if(param.count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[5]);
        }

        static Robots::WalkParam realParam = param;

        if(!isWalking)
        {
            WALK_DIRECTION walkDir=forceJudge(forceInBody, forceThreshold);
            if(walkDir!=STOP)
            {
                switch (walkDir)
                {
                case FORWARD:
                    realParam.d=param.d;
                    realParam.alpha=0;
                    realParam.beta=0;
                    rt_printf("Walking Forward\n");
                    break;
                case BACKWARD:
                    realParam.d=-1*param.d;
                    realParam.alpha=0;
                    realParam.beta=0;
                    rt_printf("Walking Backward\n");
                    break;
                case LEFTWARD:
                    realParam.d=param.d/2;
                    realParam.alpha=PI/2;
                    realParam.beta=0;
                    rt_printf("Walking Leftward\n");
                    break;
                case RIGHTWARD:
                    realParam.d=param.d/2;
                    realParam.alpha=-PI/2;
                    realParam.beta=0;
                    rt_printf("Walking Rightward\n");
                    break;
                case TURNLEFT:
                    realParam.d=0;
                    realParam.alpha=0;
                    realParam.beta=PI/12;
                    rt_printf("Turning Left\n");
                    break;
                case TURNRIGHT:
                    realParam.d=0;
                    realParam.alpha=0;
                    realParam.beta=-PI/12;
                    rt_printf("Turning Right\n");
                    break;
                case FAST_TURNLEFT:
                    realParam.d=0;
                    realParam.alpha=0;
                    realParam.beta=PI/6;
                    rt_printf("Fast Turning Left\n");
                    break;
                case FAST_TURNRIGHT:
                    realParam.d=0;
                    realParam.alpha=0;
                    realParam.beta=-PI/6;
                    rt_printf("Fast Turning Right\n");
                    break;
                default:
                    break;
                }
                isWalking=true;
                walkBeginCount=param.count;

                rt_printf("realForceData: %f %f %f\n",forceInBody[0],forceInBody[1],forceInBody[5]);
            }
        }
        else
        {
            realParam.count = param.count - walkBeginCount;
            int ret = Robots::walkGait(robot, realParam);
            if(ret == 0)
            {
                rt_printf("Finish One Walking Step\n");
                isWalking=false;
            }
        }
    }

    if(state::getState().isStopping() && (!isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

WALK_DIRECTION forceJudge(const double *force, const double *threshold)
{
    WALK_DIRECTION walkDir{STOP};
    if(std::fabs(force[5]) > threshold[5])
    {
        if(force[5] < -2*threshold[5])
            walkDir=FAST_TURNRIGHT;
        else if(force[5] < -threshold[5])
            walkDir=TURNRIGHT;
        else if(force[5] > 2*threshold[5])
            walkDir=FAST_TURNLEFT;
        else if(force[5] > threshold[5])
            walkDir=TURNLEFT;
    }
    else if(std::fabs(std::fabs(force[0]) - threshold[0]) > std::fabs(std::fabs(force[1]) - threshold[1]))
    {
        if(force[0] < -threshold[0])
            walkDir=FORWARD;
        else if(force[0] > threshold[0])
            walkDir=BACKWARD;
    }
    else
    {
        if(force[1] < -threshold[1])
            walkDir=LEFTWARD;
        else if(force[1] > threshold[1])
            walkDir=RIGHTWARD;
    }
    return walkDir;
}
