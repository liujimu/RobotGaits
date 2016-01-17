#include "force_guided_walk.h"

using namespace Aris::DynKer;

std::atomic_bool g_isStoppingFGW;

int ForceGuidedWalk(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const FGW_PARAM *pFGWP = static_cast<const Robots::WALK_PARAM *>(pParam);

    static bool s_isWalking{false};
    static bool s_isLastStep{false};

    static int s_walkBeginCount{0};
    static double s_forceOffsetSum[3]{0};

    static double s_prevBodyPE[6]{0};
    static double s_prevBodyVel[6]{0};
    static double s_prevPee[18]{0};
    static double s_prevVee[18]{0};

    const double CLOCK_PERIOD{0.001};
    const double FORCE_MAX{1};
    const double MASS{1};
    const double DAMP{4};
    const double FORCE_FACTOR{0.001};
    const double FORCE_THRESHOLD[3]{40,40,40};//力传感器的触发阈值,单位N或Nm

    double forceOffsetAvg[3]{0};
    double realForce[3]{0};
    double pBodyPE[6];
    double pEE[18];

    //力传感器手动清零
    if (pCWFP->count < 100)
    {
        if(pCWFP->count == 0)
        {
            for(int i = 0; i < 3; i++)
            {
                forceOffsetSum[i] = 0;
            }
        }
        s_forceOffsetSum[0] += pFGWP->pForceData->at(0).Fx;
        s_forceOffsetSum[1] += pFGWP->pForceData->at(0).Fy;
        s_forceOffsetSum[2] += pFGWP->pForceData->at(0).Mz;
    }
    //步态主体
    else
    {
        //对力信号进行标准化处理
        for(int i = 0; i < 3; i++)
        {
            forceOffsetAvg[i] = forceOffsetSum[i] / 100;
        }
        if(pCWFP->count == 100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[2]);
        }
        realForce[0] = (pFGWP->pForceData->at(0).Fx - forceOffsetAvg[0]) * FORCE_FACTOR;
        realForce[1] = (pFGWP->pForceData->at(0).Fy - forceOffsetAvg[1]) * FORCE_FACTOR;
        realForce[2] = (pFGWP->pForceData->at(0).Mz - forceOffsetAvg[2]) * FORCE_FACTOR;

        //暂时只考虑前后方向上的行走
        if(std::fabs(realForce[0]) > FORCE_MAX)
        {
            realForce[0] = realForce[0] / std::fabs(realForce[0]) * FORCE_MAX;
        }
        else if(std::fabs(realForce[0]) < FORCE_THRESHOLD)
        {
            realForce[0] = 0;
        }

        double pEE[18];
        double pBodyPE[6];

        if(!s_isWalking)
        {
            if(realForce[0] != 0)
            {
                s_isWalking = true;
                s_walkBeginCount=pFGWP->count;
                pRobot->GetPee(realParam.beginPee);
                pRobot->GetBodyPe(realParam.beginBodyPE);
                std::copy_n(realParam.beginPee,18,s_prevPee);
                std::copy_n(realParam.beginBodyPE,6,s_prevBodyPE);
                std::memset(s_prevVee,0,sizeof(s_prevVee));
                std::memset(s_prevBodyVel,0,sizeof(s_prevBodyVel));
            }
        }
        else
        {
            realParam.count = pFGWP->count - s_walkBeginCount;
            if (pRealParam->count < pRealParam->totalCount)
            {
                int count = pRealParam->count;
                double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
                /*设置移动腿*/
                for (int i = 3; i < 18; i += 6)
                {
                    pEE[i + wAxis] = wSign*(d *cos(a + b)*(1 - cos(s)) / 2
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 2 * b)
                        - lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 2 * b))
                        + beginBodyPE[wAxis];
                    pEE[i + uAxis] = uSign*h*sin(s)
                        + beginPee[i + uAxis];
                    pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 2 * b)
                        + lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 2 * b))
                        + beginBodyPE[lAxis];
                }
                /*设置支撑腿*/
                for (int i = 0; i < 18; i += 6)
                {
                    pEE[i + wAxis] = beginPee[i + wAxis];
                    pEE[i + uAxis] = beginPee[i + uAxis];
                    pEE[i + lAxis] = beginPee[i + lAxis];
                }
            }
            else
            {
                int count = pRealParam->count - pRealParam->totalCount;
                double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
                /*设置移动腿*/
                for (int i = 0; i < 18; i += 6)
                {
                    pEE[i + wAxis] = wSign*(d *cos(a + b)*(1 - cos(s)) / 2
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos((1 - cos(s)) / 2 * b)
                        - lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin((1 - cos(s)) / 2 * b))
                        + beginBodyPE[wAxis];
                    pEE[i + uAxis] = uSign*h*sin(s)
                        + beginPee[i + uAxis];
                    pEE[i + lAxis] = lSign*(d *sin(a + b)*(1 - cos(s)) / 2
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin((1 - cos(s)) / 2 * b)
                        + lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos((1 - cos(s)) / 2 * b))
                        + beginBodyPE[lAxis];
                }
                /*设置支撑腿*/
                for (int i = 3; i < 18; i += 6)
                {
                    pEE[i + wAxis] = wSign*(d *cos(a + b)
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * cos(b)
                        - lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * sin(b))
                        + beginBodyPE[wAxis];
                    pEE[i + uAxis] = 0
                        + beginPee[i + uAxis];
                    pEE[i + lAxis] = lSign*(d *sin(a + b)
                        + wSign*(beginPee[i + wAxis] - beginBodyPE[wAxis]) * sin(b)
                        + lSign*(beginPee[i + lAxis] - beginBodyPE[lAxis]) * cos(b))
                        + beginBodyPE[lAxis];
                }
            }
            
            int ret=Robots::walk(pRobot, &realParam);
            if(ret==0)
            {
                if(s_prevBodyVel[2] == 0 && realForce[0] == 0)
                s_isWalking=false;
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

}


Aris::Core::MSG parseForceGuidedWalkStop(const std::string &cmd, const std::map<std::string, std::string> &params)
{

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
