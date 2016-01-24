#include "move2.h"

/*将以下注释代码添加到xml文件*/
/*
       <move2>
        <move2_param type="group">
          <component abbreviation="c" type="string" default="bd"/>
          <move_pos type="unique" default="relative">
            <relative type="group">
              <u abbreviation="u" type="double" default="0"/>
              <v abbreviation="v" type="double" default="0"/>
              <w abbreviation="w" type="double" default="0"/>
            </relative>
            <absolute type="group">
              <x abbreviation="x" type="double" default="0"/>
              <y abbreviation="y" type="double" default="0"/>
              <z abbreviation="z" type="double" default="0"/>
            </absolute>
          </move_pos>
        </move2_param>
      </move2>
*/

Aris::Core::MSG parseMove2(const std::string &cmd, const std::map<std::string, std::string> &params)
{
    MOVES_PARAM  param;

    double targetPos[3]; //移动目标位置

    for(auto &i:params)
    {
        if(i.first=="component")
        {
            if(i.second=="lf")
            {
                param.comID=0;
            }
            else if(i.second=="lm")
            {
                param.comID=1;
            }
            else if(i.second=="lr")
            {
                param.comID=2;
            }
            else if(i.second=="rf")
            {
                param.comID=3;
            }
            else if(i.second=="rm")
            {
                param.comID=4;
            }
            else if(i.second=="rr")
            {
                param.comID=5;
            }
            else if(i.second=="bd")
            {
                param.comID=6;
            }
            else
            {
                std::cout<<"parse failed"<<std::endl;
                return MSG{};
            }
        }
        //绝对坐标移动
        else if(i.first=="x")
        {
            targetPos[0]=std::stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="y")
        {
            targetPos[1]=std::stod(i.second);
            param.isAbsolute=true;
        }
        else if(i.first=="z")
        {
            targetPos[2]=std::stod(i.second);
            param.isAbsolute=true;
        }
        //相对坐标移动
        else if(i.first=="u")
        {
            targetPos[0]=std::stod(i.second);
        }
        else if(i.first=="v")
        {
            targetPos[1]=std::stod(i.second);
        }
        else if(i.first=="w")
        {
            targetPos[2]=std::stod(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
            return MSG{};
        }
    }

    if(param.comID==6)
    {
        std::copy_n(targetPos, 3, param.targetBodyPE);
    }
    else
    {
        std::copy_n(targetPos, 3, &param.targetPee[3*param.comID]);
    }

    param.periodCount=3000;

    Aris::Core::MSG msg;
    msg.CopyStruct(param);

    std::cout<<"finished parse"<<std::endl;

    return msg;
}

int move2(Robots::ROBOT_BASE * pRobot, const Robots::GAIT_PARAM_BASE * pParam)
{
    const MOVES_PARAM *pMP = static_cast<const MOVES_PARAM *>(pParam);

    double realTargetPee[18];
    double realTargetPbody[6];
    std::copy_n(pMP->beginPee, 18, realTargetPee);
    std::copy_n(pMP->beginBodyPE, 6, realTargetPbody);

    //绝对坐标
    if (pMP->isAbsolute)
    {
        if(pMP->comID==6)
        {
            std::copy_n(pMP->targetBodyPE, 3, realTargetPbody);
        }
        else
        {
            std::copy_n(&(pMP->targetPee[pMP->comID*3]), 3, &realTargetPee[pMP->comID*3]);
        }
    }
    //相对坐标
    else
    {
        if(pMP->comID==6)
        {
            for(int i=0;i<3;i++)
            {
                realTargetPbody[i]+=pMP->targetBodyPE[i];
            }
        }
        else
        {
            for(int i=0;i<18;i++)
            {
                realTargetPee[i]+=pMP->targetPee[i];
            }
        }
    }

    double s = -(PI / 2)*cos(PI * (pMP->count  + 1) / pMP->periodCount ) + PI / 2;

    /*插值当前的末端和身体位置*/
    double pEE[18], pBody[6];

    for (int i = 0; i < 18; ++i)
    {
        pEE[i] = pMP->beginPee[i] * (cos(s) + 1) / 2 + realTargetPee[i] * (1 - cos(s)) / 2;
    }
    for (int i = 0; i < 6; ++i)
    {
        pBody[i] = pMP->beginBodyPE[i] * (cos(s) + 1) / 2 + realTargetPbody[i] * (1 - cos(s)) / 2;
    }

    pRobot->SetPee(pEE, pBody);

    /*test*/
//    if(pMP->count%500==0)
//    {
//        rt_printf("rr: %f %f %f\n"
//                        , pEE[15], pEE[16], pEE[17]);
//    }

    /*返回剩余的count数*/

    return pMP->periodCount - pMP->count - 1;

}

