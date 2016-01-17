#include "push_recovery.h"

using namespace Aris::DynKer;

std::atomic_bool g_isStoppingPR;

int PushRecovery(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const Robots::WALK_PARAM *pCWFP = static_cast<const Robots::WALK_PARAM *>(pParam);

    static bool isWalking=false;
    static int walkBeginCount{0};
    static double forceOffsetSum[3]{0};
    static double beginPm[16];
    static double beginBodyPE[6];
    static double beginPee[18];
    static double bodyPE[6];
    static double bodyVel[6];

    double forceOffsetAvg[3]{0};
    double realForce[3]{0};
    const double clockPeriod{0.001};
    const double forceThreshold[3]{40, 40, 40};//力传感器的触发阈值,单位N或Nm
    const double forceAMFactor=1;
    const int s2r[3]{2, 0, 1};//将力传感器的坐标系映射到机器人身体坐标系
    int wAxis{2};
    char order[4]="313";

    //下列数组的顺序是相对于机器人身体的xyz轴
    double M{1};
    double K[6]{0};
    double C[6]{2,2,2,2,2,2};
    double F[6]{0,0.5,0,0.5,0,0};
    const double Fmax{1};

    double d=0.5;
    double h=0.05;

    //力传感器手动清零
    if (pCWFP->count<100)
    {
        if(pCWFP->count==0)
        {
            for(int i=0;i<3;i++)
            {
                forceOffsetSum[i]=0;
            }
        }
        forceOffsetSum[0]+=pCWFP->pForceData->at(0).Fx;
        forceOffsetSum[1]+=pCWFP->pForceData->at(0).Fy;
        forceOffsetSum[2]+=pCWFP->pForceData->at(0).Fz;
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            forceOffsetAvg[i]=forceOffsetSum[i]/100;
        }
        if(pCWFP->count==100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[2]);
        }
        realForce[0]=(pCWFP->pForceData->at(0).Fx-forceOffsetAvg[0])/forceAMFactor;
        realForce[1]=(pCWFP->pForceData->at(0).Fy-forceOffsetAvg[1])/forceAMFactor;
        realForce[2]=(pCWFP->pForceData->at(0).Fz-forceOffsetAvg[2])/forceAMFactor;

        if(!isWalking)
        {
            int forceIndex{-1};
            double forceMax{0};
            for(int i = 0; i < 3; i++)
            {
                double forceTmp = std::fabs(realForce[i]) - forceThreshold[i];
                if(forceTmp > forceMax)
                {
                    forceIndex = i;
                    forceMax = forceTmp;
                }
            }

            if(forceIndex >= 0)
            {
                wAxis = s2r[forceIndex];
                F[wAxis] += Fmax;
                K[3] = 1.5;
                K[1] = 1.5;
                if(wAxis == 0)
                {
                    std::strcpy(order, "123");
                }

                isWalking=true;
                walkBeginCount = pCWFP->count + 1;

                s_pe2pm(pCWFP->beginBodyPE, beginPm);

                pRobot->GetBodyPe(beginBodyPE);
                pRobot->GetPee(beginPee);

                std::memset(bodyPE, 0 ,sizeof(bodyPE));
                std::memset(bodyVel, 0, sizeof(bodyVel));

                rt_printf("realForceData: %f %f %f\n",realForce[0],realForce[1],realForce[2]);
                rt_printf("beginBodyPE: %f %f %f\n",beginBodyPE[0],beginBodyPE[1],beginBodyPE[2]);
                rt_printf("PushDirection: %c\n", s2r[forceIndex]+'x');
            }
        }
        else
        {
            double pEE[18];
            double pBodyPE[6];

            /*初始化完毕，开始计算*/
            int count = pCWFP->count - walkBeginCount;
            int totalCount = 3000;

            if(count < totalCount)
            {
                double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;
                /*设置移动腿*/
                for (int i = 0; i < 18; i += 6)
                {
                    pEE[i + 1] = h*sin(s) + beginPee[i + 1];
                    pEE[i + wAxis] = d / 2 * (1 - cos(s)) + beginPee[i + wAxis];
                    pEE[i + 2 - wAxis] = beginPee[i + 2 - wAxis];
                }
                /*设置支撑腿*/
                for (int i = 3; i < 18; i += 6)
                {
                    pEE[i + 1] = beginPee[i + 1];
                    pEE[i + wAxis] = beginPee[i + wAxis];
                    pEE[i + 2 - wAxis] = beginPee[i + 2 - wAxis];
                }
            }
            else
            {
                double s = -(PI / 2)*cos(PI * (count - totalCount + 1) / totalCount) + PI / 2;
                /*设置移动腿*/
                for (int i = 3; i < 18; i += 6)
                {
                    pEE[i + 1] = h*sin(s) + beginPee[i + 1];
                    pEE[i + wAxis] = d / 2 * (1 - cos(s)) + beginPee[i + wAxis];
                    pEE[i + 2 - wAxis] = beginPee[i + 2 - wAxis];
                }
                /*设置支撑腿*/
                for (int i = 1; i < 18; i += 6)
                {
                    pEE[i + 1] = beginPee[i + 1];
                    pEE[i + wAxis] = beginPee[i + wAxis] + d;
                    pEE[i + 2 - wAxis] = beginPee[i + 2 - wAxis];
                }
            }

            /*设置身体*/
            if(count >= 2000)
            {
                std::memset(F, 0, sizeof(F));
            }
            double bodyAcc[6]{0};
            for (int i=0;i<6;i++)
            {
                bodyAcc[i] = (F[i] - C[i] * bodyVel[i] - K[i] * bodyPE[i]) / M;
                bodyVel[i] += bodyAcc[i] * clockPeriod;
                bodyPE[i] += bodyVel[i] * clockPeriod;
                if(bodyPE[1] < 0)
                {
                    bodyPE[1] = 0;
                }
                if(bodyPE[3] < 0)
                {
                    bodyPE[3] = 0;
                }
            }

            double relativePm[16], absolutePm[16];
            Aris::DynKer::s_pe2pm(bodyPE, relativePm, order);
            Aris::DynKer::s_pm_dot_pm(beginPm, relativePm, absolutePm);
            Aris::DynKer::s_pm2pe(absolutePm, pBodyPE,"313");

            /*计算完毕，更新pRobot*/
            pRobot->SetPee(pEE, pBodyPE, "G");

            //判断动作结束
            if(count == (2*totalCount - 1))
            {
                rt_printf("Finish One Walking Step\n");
                rt_printf("pEE: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n"
                    , pEE[0], pEE[1], pEE[2], pEE[3], pEE[4], pEE[5], pEE[6], pEE[7], pEE[8]
                    , pEE[9], pEE[10], pEE[11], pEE[12], pEE[13], pEE[14], pEE[15], pEE[16], pEE[17]);
                rt_printf("pBodyPE: %f %f %f %f %f %f\n"
                    , pBodyPE[0], pBodyPE[1], pBodyPE[2], pBodyPE[3], pBodyPE[4], pBodyPE[5]);

                isWalking=false;
            }
        }
    }

    if(g_isStoppingPR && (!isWalking))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

Aris::Core::MSG parsePushRecovery(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    g_isStoppingPR = false;

    Robots::GAIT_PARAM_BASE param;
    Aris::Core::MSG msg;
    msg.CopyStruct(param);
    return msg;
}


Aris::Core::MSG parsePushRecoveryStop(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    g_isStoppingPR = true;

    Aris::Core::MSG msg;
    return msg;
}
