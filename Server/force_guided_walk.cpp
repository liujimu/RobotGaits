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
    if (pCWFP->count < 100)
    {
        if(pCWFP->count == 0)
        {
            std::memset(forceOffsetSum, 0, sizeof(forceOffsetSum));
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
            forceOffsetAvg[i] = forceOffsetSum[i] / 100;
        }
        if(pCWFP->count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[2]);
        }
        realForce[0] = (pFGWP->pForceData->at(0).Fx - forceOffsetAvg[0]) / kForceAmpliFactor;
        realForce[1] = (pFGWP->pForceData->at(0).Fy - forceOffsetAvg[1]) / kForceAmpliFactor;
        /*检测力的方向，确定运动参数*/
        if(!isWalking)
        {
            forceIndex{-1};
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
            int count = (pFGWP->count - s_beginCount) % pFGWP->totalCount;
            double pEE[18];
            double pBodyPE[6];
            double h = pFGWP->stepHeight;

            double F = realForce[s_forceIndex] / kForceNormaliFactor * kForceMax;
            if(F > kForceMax)
            {
                F = kForceMax;
            }
            else if(std::fabs(F) < kForceThreshold[s_forceIndex] / kForceNormaliFactor * kForceMax)
            {
                F = 0;
            }

            if(!s_isLastStep)
            {
                /*设置身体*/
                //用阻抗模型计算身体位姿变化
                double bodyAcc = (F - C * s_prevBodyVel) / M;
                s_prevBodyVel += bodyAcc * kClockPeriod;
                s_prevBodyDspl += s_prevBodyVel * kClockPeriod;

                //预估身体在totalCount时的位置、速度
                double remianTime = (pFGWP->totalCount - count - 1) * kClockPeriod;
                double bodyPeFinal = M / C * (kClockPeriod - F / C) * (1 - std::exp(-C / M * remianTime)) + F / C * remianTime + s_prevBodyDspl;
                double bodyVelFinal =(s_prevBodyVel - F / C) * std::exp(-C / M * remianTime) + F / C;

                /*设置足尖位置*/
                std::copy_n(s_beginPee, 18, pEE);
                int fAxis = S2B[s_forceIndex];
                int parity = ((pFGWP->count - s_beginCount) / pFGWP->totalCount) % 2; //计算迈腿顺序，parity=0为奇数步
                /*设置移动腿*/
                /*设置足尖高度*/
                if(count < (pFGWP->totalCount / 2))
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3((count + 1) * kClockPeriod, 0, pFGWP->totalCount / 2 * kClockPeriod, 0, pFGWP->stepHeight, 0, 0);//高度变化
                    }
                }
                else
                {
                    for (int i = parity; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1] + Hermite3((count + 1) * kClockPeriod, 0, pFGWP->totalCount / 2 * kClockPeriod, pFGWP->stepHeight, 0, 0, 0);//高度变化
                    }
                }

                pEE[i + fAxis] = d / 2 * (1 - cos(s)) + s_beginPee[i + fAxis];//水平位置变化

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

            }

            double pEE2G[18];
            double pBodyPE2G[6];
            //将身体位资转换到地面坐标系
            pBodyPE[s_forceIndex] = s_prevBodyDspl;
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

            //判断动作结束
            if(s_isLastStep && count == (totalCount - 1))
            {
                s_isWalking = false;
                s_fSign = 0;
            }
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

int WalkLastStep(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    /*初始化参数*/
    const WALK_PARAM *pRealParam = static_cast<const Robots::WALK_PARAM *>(pParam);

    int wAxis = std::abs(pRealParam->walkDirection) - 1;
    int uAxis = std::abs(pRealParam->upDirection) - 1;
    int lAxis = 3 - wAxis - uAxis;
    int wSign = pRealParam->walkDirection / std::abs(pRealParam->walkDirection);
    int uSign = pRealParam->upDirection / std::abs(pRealParam->upDirection);
    int lSign = ((3 + wAxis - uAxis) % 3 == 1) ? wSign* uSign : -wSign* uSign;

    double pm[4][4], pe[6];
    s_pe2pm(pParam->beginBodyPE, *pm, "313");
    char order[4]{0};
    order[0] = '1' + uAxis;
    order[1] = '1' + (1 + uAxis) % 3;
    order[2] = '1' + (2 + uAxis) % 3;
    s_pm2pe(*pm, pe, order);

    int totalCount = pRealParam->totalCount;
    double h = pRealParam->h;
    double d = pRealParam->d;
    double a = pRealParam->alpha + uSign*pe[3];
    double b = pRealParam->beta;

    const double *beginPee = pRealParam->beginPee;
    const double *beginBodyPE = pRealParam->beginBodyPE;

    double pEE[18];
    double pBodyPE[6];

    /*初始化完毕，开始计算*/
    int count = pRealParam->count;
    double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
    /*设置移动腿*/
    for (int i = 3; i < 18; i += 6)
    {
        pEE[i + wAxis] = wSign*(0.5*d / cos(b / 2)*cos(a + b / 2)*(1 - cos(s)) / 2
            + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * cos((1 - cos(s)) / 4 * b)
            - lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * sin((1 - cos(s)) / 4 * b))
            + beginBodyPE[wAxis] - wSign*0.25*d / cos(b / 2)*cos(a+b/2);
        pEE[i + uAxis] = uSign*h*sin(s)
            + beginPee[i + uAxis];
        pEE[i + lAxis] = lSign*(0.5*d / cos(b / 2)*sin(a + b / 2)*(1 - cos(s)) / 2
            + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis] + wSign*0.25*d / cos(b / 2)*cos(a + b / 2)) * sin((1 - cos(s)) / 4 * b)
            + lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis] + lSign*0.25*d / cos(b / 2)*sin(a + b / 2)) * cos((1 - cos(s)) / 4 * b))
            + beginBodyPE[lAxis] - lSign*0.25*d / cos(b / 2)*sin(a + b / 2);
    }



    /*设置支撑腿*/
    for (int i = 0; i < 18; i += 6)
    {
        pEE[i + wAxis] = beginPee[i + wAxis];
        pEE[i + uAxis] = beginPee[i + uAxis];
        pEE[i + lAxis] = beginPee[i + lAxis];
    }

    /*设置身体*/
    double t = count + 1;
    double T = totalCount * 2;

    /*以下计算角度，需要在313和321的欧拉角中间转来转去*/
    pe[3] += uSign*b / 4 * dec_even(totalCount, count + 1);
    s_pe2pm(pe, *pm, order);
    s_pm2pe(*pm, pBodyPE);

    /*以下计算位置*/
    pBodyPE[wAxis] += wSign*0.25*d / cos(b / 2)*cos(a + b / 2)*(dec_even(totalCount, count + 1));
    pBodyPE[uAxis] += 0;
    pBodyPE[lAxis] += lSign*0.25*d / cos(b / 2)*sin(a + b / 2)*(dec_even(totalCount, count + 1));

    /*计算完毕，更新pRobot*/
    pRobot->SetPee(pEE, pBodyPE, "G");
    return totalCount - count - 1;    
}

Aris::Core::MSG parseForceGuidedWalk(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    isStoppingCWF = false;

    Robots::GAIT_PARAM_BASE param;
    Aris::Core::MSG msg;
    msg.CopyStruct(param);
    return msg;
}


Aris::Core::MSG parseForceGuidedWalkStop(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    isStoppingCWF = true;

    Aris::Core::MSG msg;

    return msg;
}

double Hermite3(double x, double x1, double x2, double y1, double y2, double m1, double m2)
{
    double alpha1=(1+2*(x-x1)/(x2-x1))*std::pow((x-x2)/(x1-x2),2);
    double alpha2=(1+2*(x-x2)/(x1-x2))*std::pow((x-x1)/(x2-x1),2);
    double beta1=(x-x1)*std::pow((x-x2)/(x1-x2),2);
    double beta2=(x-x2)*std::pow((x-x1)/(x2-x1),2);
    double y=alpha1*y1+alpha2*y2+beta1*m1+beta2*m2;
    return y;
}
