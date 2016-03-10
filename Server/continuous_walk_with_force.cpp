#include "continuous_walk_with_force.h"

/*将以下注释代码添加到xml文件*/
/*
      <cwf default="cwf_param">
        <cwf_param type="group">
          <totalCount abbreviation="t" type="int" default="3000"/>
          <walkDirection abbreviation="w" type="int" default="-3"/>
          <upDirection abbreviation="u" type="int" default="2"/>
          <distance abbreviation="d" type="double" default="0.5"/>
          <height abbreviation="h" type="double" default="0.05"/>
        </cwf_param>
      </cwf>
      <cwfs/>
*/

std::atomic_bool isStoppingCWF;

Aris::Core::MSG parseCWF(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    Robots::WALK_PARAM  param;

    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "walkDirection")
        {
            param.walkDirection = std::stoi(i.second);
        }
        else if (i.first == "upDirection")
        {
            param.upDirection = std::stoi(i.second);
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

    isStoppingCWF=false;

    Aris::Core::MSG msg;

    msg.CopyStruct(param);

    return msg;
}

Aris::Core::MSG parseCWFStop(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    isStoppingCWF = true;

    Aris::Core::MSG msg;

    return msg;
}

int continuousWalkWithForce(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const Robots::WALK_PARAM *pCWFP = static_cast<const Robots::WALK_PARAM *>(pParam);

    static bool isWalking=false;
    static int walkBeginCount{0};
    //static double walkBeginPee[18]{0};
    //static double walkBeginBodyPE[6]{0};
    static double forceOffsetSum[3]{0};

    double forceOffsetAvg[3]{0};
    double realForceData[3]{0};
    const double forceThreshold[3]{40,40,40};//力传感器的触发阈值,单位N或Nm
    const double forceAMFactor=1;

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
        forceOffsetSum[2]+=pCWFP->pForceData->at(0).Mz;
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
        realForceData[0]=(pCWFP->pForceData->at(0).Fx-forceOffsetAvg[0])/forceAMFactor;
        realForceData[1]=(pCWFP->pForceData->at(0).Fy-forceOffsetAvg[1])/forceAMFactor;
        realForceData[2]=(pCWFP->pForceData->at(0).Mz-forceOffsetAvg[2])/forceAMFactor;

        static Robots::WALK_PARAM realParam = *pCWFP;

        if(!isWalking)
        {
            WALK_DIRECTION walkDir=forceJudge(realForceData, forceThreshold);
            if(walkDir!=STOP)
            {
                switch (walkDir)
                {
                case FORWARD:
                    realParam.d=pCWFP->d;
                    realParam.alpha=0;
                    realParam.beta=0;
                    rt_printf("Walking Forward\n");
                    break;
                case BACKWARD:
                    realParam.d=-1*pCWFP->d;
                    realParam.alpha=0;
                    realParam.beta=0;
                    rt_printf("Walking Backward\n");
                    break;
                case LEFTWARD:
                    realParam.d=pCWFP->d/2;
                    realParam.alpha=PI/2;
                    realParam.beta=0;
                    rt_printf("Walking Leftward\n");
                    break;
                case RIGHTWARD:
                    realParam.d=pCWFP->d/2;
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
                walkBeginCount=pCWFP->count;
                pRobot->GetPee(realParam.beginPee);
                pRobot->GetBodyPe(realParam.beginBodyPE);

                rt_printf("realForceData: %f %f %f\n",realForceData[0],realForceData[1],realForceData[2]);
                rt_printf("beginBodyPE: %f %f %f\n",realParam.beginBodyPE[0],realParam.beginBodyPE[1],realParam.beginBodyPE[2]);
            }
        }
        else
        {
            realParam.count=pCWFP->count-walkBeginCount;
            int ret=Robots::walk(pRobot, &realParam);
            if(ret==0)
            {
                rt_printf("Finish One Walking Step\n");
                isWalking=false;
            }
        }
    }

    if(isStoppingCWF && (!isWalking))
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
    if(std::fabs(force[2]) > threshold[2])
    {
        if(force[2] < -2*threshold[2])
            walkDir=FAST_TURNRIGHT;
        else if(force[2] < -threshold[2])
            walkDir=TURNRIGHT;
        else if(force[2] > 2*threshold[2])
            walkDir=FAST_TURNLEFT;
        else if(force[2] > threshold[2])
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
