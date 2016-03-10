#include "force_guided_walk.h"

using namespace Aris::DynKer;

std::atomic_bool g_isStoppingFGW;

int ForceGuidedWalk(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const FGW_PARAM *pFGWP = static_cast<const FGW_PARAM *>(pParam);

    static bool s_isWalking{false};
    static bool s_isLastStep{false};

    static int s_walkBeginCount{0};
    static double s_forceOffsetSum[3]{0};
    static int s_forceIndex{0};

    static double s_beginPm[16];
    static double s_beginPee[18];
    static double s_prevBodyDspl;//上一毫秒身体的位移
    static double s_prevBodyVel;//上一毫秒身体的速度
    static double s_prevFootDspl;//上一毫秒足尖的位移
    static double s_prevFootVel;//上一毫秒足尖的速度

    const double kClockPeriod{0.001};
    const double M{1};
    const double C{3};
    const double kForceMax{1};
    const double kForceAmpliFactor{1000};
    const double kForceNormaliFactor{100};
    const double kForceThreshold[3]{40,40,40};//力传感器的触发阈值,单位N或Nm
    const int S2B[3]{2, 0, 1};//将力传感器的坐标系映射到机器人身体坐标系

    double forceOffsetAvg[2]{0};
    double realForce[2]{0};

    /*记录前100ms力传感器的数值，用于手动清零*/
    if (pFGWP->count < 100)
    {
        if(pFGWP->count == 0)
        {
            std::memset(s_forceOffsetSum, 0, sizeof(s_forceOffsetSum));
        }
        s_forceOffsetSum[0] += pFGWP->pForceData->at(0).Fx;
        s_forceOffsetSum[1] += pFGWP->pForceData->at(0).Fy;
    }
    /*步态主体*/
    else
    {
        /*对力信号进行清零*/
        for(int i = 0; i < 2; i++)
        {
            forceOffsetAvg[i] = s_forceOffsetSum[i] / 100;
        }
        if(pFGWP->count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[2]);
        }
        realForce[0] = (pFGWP->pForceData->at(0).Fx - forceOffsetAvg[0]) / kForceAmpliFactor;
        realForce[1] = (pFGWP->pForceData->at(0).Fy - forceOffsetAvg[1]) / kForceAmpliFactor;
        /*检测力的方向，确定运动参数*/
        if(!s_isWalking)
        {
            double forceMax{0};
            for(int i = 0; i < 3; i++)
            {
                double forceTmp = std::fabs(realForce[i]) - kForceThreshold[i];
                if(forceTmp > forceMax)
                {
                    s_forceIndex = i;
                    forceMax = forceTmp;
                }
            }

            if(s_forceIndex >= 0)
            {
                s_isWalking = true;
                s_walkBeginCount = pFGWP->count + 1;

                s_prevBodyDspl = 0;
                s_prevBodyVel = 0;
            }
        }

        /*运动规划*/
        else
        {
            int count = (pFGWP->count - s_walkBeginCount) % pFGWP->totalCount;
            double curTime = (count + 1) * kClockPeriod;
            int parity = ((pFGWP->count - s_walkBeginCount) / pFGWP->totalCount) % 2; //计算迈腿顺序，parity=0为奇数步
            double pEE[18];
            double pBodyPE[6];
            std::copy_n(s_beginPee, 18, pEE);
            int fAxis = S2B[s_forceIndex];

            if(!s_isLastStep)
            {
                double F = realForce[s_forceIndex] / kForceNormaliFactor * kForceMax;
                if(F > kForceMax)
                {
                    F = kForceMax;
                }
                else if(std::fabs(F) < kForceThreshold[s_forceIndex] / kForceNormaliFactor * kForceMax)
                {
                    F = 0;
                }

                /*设置身体*/
                /*用阻抗模型计算身体位姿变化*/
                double bodyAcc = (F - C * s_prevBodyVel) / M;
                s_prevBodyVel += bodyAcc * kClockPeriod;
                s_prevBodyDspl += s_prevBodyVel * kClockPeriod;
                pBodyPE[fAxis] = s_prevBodyDspl;
                /*预估身体在totalCount时的位置、速度*/
                double remianTime = (pFGWP->totalCount - count - 1) * kClockPeriod;
                double finalBodyDspl = M / C * (kClockPeriod - F / C) * (1 - std::exp(-C / M * remianTime)) + F / C * remianTime + s_prevBodyDspl;
                double finalBodyVel =(s_prevBodyVel - F / C) * std::exp(-C / M * remianTime) + F / C;

                /*设置足尖位置*/
                /*设置移动腿*/
                /*设置足尖高度*/
                if(count < pFGWP->totalCount / 2)
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3(curTime, 0, pFGWP->totalCount / 2 * kClockPeriod, 0, pFGWP->stepHeight, 0, 0);//高度变化
                    }
                }
                else
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3(curTime, 0, pFGWP->totalCount / 2 * kClockPeriod, pFGWP->stepHeight, 0, 0, 0);//高度变化
                    }
                }
                /*设置足尖水平位移*/
                if(count < pFGWP->totalCount - 1)
                {
                    double curFootDspl = Hermite3(curTime, curTime - kClockPeriod, pFGWP->totalCount * kClockPeriod, s_prevFootDspl, 2 * finalBodyDspl, s_prevFootVel, 0);
                    double nextFootDspl = Hermite3(curTime + kClockPeriod, curTime - kClockPeriod, pFGWP->totalCount * kClockPeriod, s_prevFootDspl, 2 * finalBodyDspl, s_prevFootVel, 0);
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + fAxis] = s_beginPee[i + fAxis] + curFootDspl;//水平位置变化
                    }
                    s_prevFootVel = (nextFootDspl - s_prevFootDspl) / 2 / kClockPeriod;
                    s_prevFootDspl = curFootDspl;
                }
                else
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + fAxis] = s_beginPee[i + fAxis] + 2 * finalBodyDspl;
                    }
                    s_prevFootDspl = 0;
                    s_prevFootVel = 0;
                    /*判断下一步是否是最后一步*/
                    if(F == 0 && std::fabs(s_prevBodyVel) < 0.01)
                    {
                        s_isLastStep = true;
                    }
                }
                /*设置支撑腿*/
                for (int i = 3 * (1 - parity); i < 18; i += 6)
                {
                    pEE[i + 1] = s_beginPee[i + 1];
                    pEE[i + fAxis] = s_beginPee[i + fAxis];
                }
            }

            /*规划最后一步*/
            else
            {
                /*设置身体*/
                double finalBodyDspl = s_beginPee[9 * parity + fAxis];
                pBodyPE[fAxis] = Hermite3(curTime, 0, pFGWP->totalCount * kClockPeriod, 0, finalBodyDspl, s_prevBodyVel, 0);

                /*设置足尖位置*/
                /*设置足尖高度*/
                if(count < pFGWP->totalCount / 2)
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3(curTime, 0, pFGWP->totalCount / 2 * kClockPeriod, 0, pFGWP->stepHeight, 0, 0);//高度变化
                    }
                }
                else
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3(curTime, 0, pFGWP->totalCount / 2 * kClockPeriod, pFGWP->stepHeight, 0, 0, 0);//高度变化
                    }
                }
                /*设置足尖水平位移*/
                double finalFootDspl = s_beginPee[9 * parity + fAxis] - s_beginPee[fAxis];;
                double curFootDspl = Hermite3(curTime, 0, pFGWP->totalCount * kClockPeriod, 0, finalFootDspl, s_prevBodyVel, 0);
                for (int i = parity; i < 18; i += 6)
                {
                    pEE[i + fAxis] = s_beginPee[i + fAxis] + curFootDspl;//水平位置变化
                }

                /*设置支撑腿*/
                for (int i = 3 * (1 - parity); i < 18; i += 6)
                {
                    pEE[i + 1] = s_beginPee[i + 1];
                    pEE[i + fAxis] = s_beginPee[i + fAxis];
                }

                //判断动作结束
                if(count == pFGWP->totalCount - 1)
                {
                    s_isWalking = false;
                    s_isLastStep = false;
                }
            }

            double pEE2G[18];
            double pBodyPE2G[6];
            //将身体位资转换到地面坐标系
            char order[4] = "313";
            double relativePm[16], absolutePm[16];
            Aris::DynKer::s_pe2pm(pBodyPE, relativePm, order);
            Aris::DynKer::s_pm_dot_pm(s_beginPm, relativePm, absolutePm);
            Aris::DynKer::s_pm2pe(absolutePm, pBodyPE2G);

            //将足尖坐标转换到地面坐标系
            for (int i = 0; i < 18; i += 3)
            {
                Aris::DynKer::s_pm_dot_pnt(s_beginPm, pEE + i, pEE2G + i);
            }

            /*计算完毕，更新pRobot*/
            pRobot->SetPee(pEE2G, pBodyPE, "G");
        }
    }

    if(g_isStoppingFGW && (!s_isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

Aris::Core::MSG parseForceGuidedWalk(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    FGW_PARAM param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.stepLengthLimit = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.stepHeight = stod(i.second);
        }
    }

    g_isStoppingFGW = false;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    return msg;
}


Aris::Core::MSG parseForceGuidedWalkStop(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    g_isStoppingFGW = true;

    Aris::Core::MSG msg;

    return msg;
}
