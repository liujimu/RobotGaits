#include "push_recovery.h"

using namespace Aris::DynKer;

std::atomic_bool g_isStoppingPR;

int PushRecovery(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const PR_PARAM *pPRP = static_cast<const PR_PARAM *>(pParam);

    static bool isWalking{false};
    static int s_beginCount{0};
    static double forceOffsetSum[3]{0};
    static double s_beginPm[16];
    static double s_beginPee[18];
    static double s_bodyPE[6];
    static double s_bodyVel[6];
    static double s_recoverBodyPE[6];//插值起点
    static double s_recoverBodyVel[6];//插值起点
    static double s_F[6]{0};
    static int s_fAxis;
    static int s_fSign;

    const double clockPeriod{0.001};
    const double forceThreshold[3]{40, 40, 80};//力传感器的触发阈值,单位N或Nm
    const double forceAMFactor=1;//力传感器输出数值与实际作用力的比值，1或1000
    double forceOffsetAvg[3]{0};
    double realForce[3]{0};
    const int s2b[3]{2, 0, 1};//将力传感器的坐标系映射到机器人身体坐标系

    double M{1};
    double C[6]{1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    double K[6]{0, 1.5, 0, 1.5, 0, 0};
    const double Fv{0.1};
    const double Fh{0.5};
    const double Fr{0.3};

    //力传感器手动清零
    if (pPRP->count<100)
    {
        if(pPRP->count==0)
        {
            for(int i=0;i<3;i++)
            {
                forceOffsetSum[i]=0;
            }
        }
        forceOffsetSum[0]+=pPRP->pForceData->at(0).Fx;
        forceOffsetSum[1]+=pPRP->pForceData->at(0).Fy;
        forceOffsetSum[2]+=pPRP->pForceData->at(0).Fz;
    }

    else
    {
        for(int i=0;i<3;i++)
        {
            forceOffsetAvg[i]=forceOffsetSum[i]/100;
        }
        if(pPRP->count==100)
        {
            rt_printf("forceOffsetAvg: %f %f %f\n",forceOffsetAvg[0],forceOffsetAvg[1],forceOffsetAvg[2]);
        }
        realForce[0]=(pPRP->pForceData->at(0).Fx-forceOffsetAvg[0])/forceAMFactor;
        realForce[1]=(pPRP->pForceData->at(0).Fy-forceOffsetAvg[1])/forceAMFactor;
        realForce[2]=(pPRP->pForceData->at(0).Fz-forceOffsetAvg[2])/forceAMFactor;

        /*检测力的方向，确定运动参数*/
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
                s_fAxis = s2b[forceIndex];
                s_fSign = realForce[forceIndex] / std::fabs(realForce[forceIndex]);
                std::memset(s_F, 0, sizeof(s_F));
                if(s_fAxis == 1)
                {
                    s_F[s_fAxis] = 2 * s_fSign * Fv;
                }
                else
                {
                    s_F[s_fAxis] = s_fSign * Fh;
                    s_F[1] = -1 * Fv;
                    s_F[3] = (1 - s_fAxis) * s_fSign * Fr;
                }

                pRobot->GetPee(s_beginPee);
                pRobot->GetBodyPm(s_beginPm);
                std::memset(s_bodyPE, 0 ,sizeof(s_bodyPE));
                std::memset(s_bodyVel, 0, sizeof(s_bodyVel));

                isWalking=true;
                s_beginCount = pPRP->count + 1;

                //For testing
                double beginBodyPE[6];
                pRobot->GetBodyPe(beginBodyPE);
                rt_printf("beginBodyPE: %f %f %f %f %f %f\n",beginBodyPE[0],beginBodyPE[1],beginBodyPE[2]);
                rt_printf("realForceData: %f %f %f\n",realForce[0],realForce[1],realForce[2]);
                rt_printf("PushDirection: %c%c\n", 44 - s_fSign, s2b[forceIndex] + 'x');
            }
        }

        /*运动规划*/
        else
        {
            double pEE[18];
            double pBodyPE[6];
            double d = pPRP->d;
            double h = pPRP->h;
            int count = pPRP->count - s_beginCount;
            int totalCount = pPRP->totalCount;

            /*设置身体*/
            if(count == pPRP->pushCount)
            {
                std::memset(s_F, 0, sizeof(s_F));
            }
            //用阻抗模型计算身体位姿变化
            double bodyAcc[6]{0};
            if(count < pPRP->recoverCount)
            {
                for (int i = 0; i < 4; i++)
                {
                    bodyAcc[i] = (s_F[i] - C[i] * s_bodyVel[i] - K[i] * s_bodyPE[i]) / M;
                    s_bodyVel[i] += bodyAcc[i] * clockPeriod;
                    s_bodyPE[i] += s_bodyVel[i] * clockPeriod;
                }
                if(count == (pPRP->recoverCount - 1))
                {
                    std::copy_n(s_bodyPE, 6, s_recoverBodyPE);
                }
            }
            //超过recoverCount后，用两点Hermite插值调整身体至目标位姿
            else if(count < totalCount)
            {
                double finalBodyPE[6]{0};
                if(s_fAxis != 1)
                {
                    finalBodyPE[s_fAxis] = s_fSign * d;
                }
                for (int i = 0; i < 4; i++)
                {
                    s_bodyPE[i]=Hermite3(count * clockPeriod, (pPRP->recoverCount - 1) * clockPeriod, (pPRP->totalCount - 1) * clockPeriod,
                                         s_recoverBodyPE[i], finalBodyPE[i], s_recoverBodyVel[i], 0);
                }
            }
            //转换到地面坐标系
            char order[4] = "313";
            if(s_fAxis == 2)
            {
                std::strcpy(order, "131");
            }
            double relativePm[16], absolutePm[16];
            Aris::DynKer::s_pe2pm(s_bodyPE, relativePm, order);
            Aris::DynKer::s_pm_dot_pm(s_beginPm, relativePm, absolutePm);
            Aris::DynKer::s_pm2pe(absolutePm, pBodyPE, order);


            std::copy_n(s_beginPee, 6, pEE);
            if(s_fAxis != 1)
            {
                if(count < totalCount / 3)
                {
                    double s = -(PI / 2) * cos(PI * (count + 1) / (totalCount / 3)) + PI / 2;
                    /*设置移动腿*/
                    for (int i = 0; i < 18; i += 6)
                    {
                        pEE[i + 1] = h*sin(s) + s_beginPee[i + 1];
                        pEE[i + s_fAxis] = d / 2 * (1 - cos(s)) + s_beginPee[i + s_fAxis];
                    }
                    /*设置支撑腿*/
                    for (int i = 3; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1];
                        pEE[i + s_fAxis] = s_beginPee[i + s_fAxis];
                    }
                }
                else if(count < totalCount)
                {
                    double s = -(PI / 2) * cos(PI * (count - totalCount / 3 + 1) / (totalCount - totalCount / 3)) + PI / 2;
                    /*设置移动腿*/
                    for (int i = 3; i < 18; i += 6)
                    {
                        pEE[i + 1] = h*sin(s) + s_beginPee[i + 1];
                        pEE[i + s_fAxis] = d / 2 * (1 - cos(s)) + s_beginPee[i + s_fAxis];
                    }
                    /*设置支撑腿*/
                    for (int i = 1; i < 18; i += 6)
                    {
                        pEE[i + 1] = s_beginPee[i + 1];
                        pEE[i + s_fAxis] = s_beginPee[i + s_fAxis] + d;
                    }
                }
            }

            //将足尖坐标转换到地面坐标系
            double pEE2G[18];
            for (int i = 0; i < 18; i += 3)
            {
                Aris::DynKer::s_pm_dot_pnt(s_beginPm, pEE + i, pEE2G + i);
            }

            /*计算完毕，更新pRobot*/
            pRobot->SetPee(pEE, pBodyPE, "G");

            //判断动作结束
            if(count == (totalCount - 1))
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
    PR_PARAM  param;

    for (auto &i : params)
    {
        if (i.first == "pushCount")
        {
            param.pushCount = std::stoi(i.second);
        }
        if (i.first == "recoverCount")
        {
            param.recoverCount = std::stoi(i.second);
        }
        else if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
    }

    g_isStoppingPR = false;

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

double Hermite3(double x, double x1, double x2, double y1, double y2, double m1, double m2)
{
    double alpha1=(1+2*(x-x1)/(x2-x1))*std::pow((x-x2)/(x1-x2),2);
    double alpha2=(1+2*(x-x2)/(x1-x2))*std::pow((x-x1)/(x2-x1),2);
    double beta1=(x-x1)*std::pow((x-x2)/(x1-x2),2);
    double beta2=(x-x2)*std::pow((x-x1)/(x2-x1),2);
    double y=alpha1*y1+alpha2*y2+beta1*m1+beta2*m2;
    return y;
}
