#include <Platform.h>

#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;


#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#endif


#include <Aris_Pipe.h>
#include <Aris_Core.h>
#include <Aris_Message.h>
#include <Aris_IMU.h>
#include <Robot_Server.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "push_recovery.h"

using namespace Aris::Core;

void startRecordGaitData();//记录步态执行过程中的数据，用于分析

int main()
{
    startRecordGaitData();

	auto rs = Robots::ROBOT_SERVER::GetInstance();
	rs->CreateRobot<Robots::ROBOT_TYPE_I>();
#ifdef PLATFORM_IS_LINUX
    rs->LoadXml("/home/hex/Desktop/RobotGaits/resource/Robot_III/Robot_III.xml");
#endif
#ifdef PLATFORM_IS_WINDOWS
	rs->LoadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
	rs->AddGait("wk", Robots::walk, Robots::parseWalk);
	rs->AddGait("ad", Robots::adjust, Robots::parseAdjust);
	rs->AddGait("fw", Robots::fastWalk, Robots::parseFastWalk);
	rs->AddGait("ro", Robots::resetOrigin, Robots::parseResetOrigin);
    //My gaits
    rs->AddGait("pr", PushRecovery, parsePushRecovery);
    rs->AddGait("prs", PushRecovery, parsePushRecoveryStop);

	rs->Start();
	std::cout<<"started"<<std::endl;

	
	
	Aris::Core::RunMsgLoop();

	return 0;
}

void startRecordGaitData()
{
    pushRecoveryThread = std::thread([&]()
    {
        struct PR_PIPE_PARAM param;
        static std::fstream fileGait;
        std::string name = Aris::Core::logFileName();
        name.replace(name.rfind("log.txt"), std::strlen("gait.txt"), "gait.txt");
        fileGait.open(name.c_str(), std::ios::out | std::ios::trunc);

        while (1)
        {
            pushRecoveryPipe.RecvInNRT(param);

            if(param.count > 0)
            {
                fileGait << param.count;
                for(int i = 0; i < 18; i++)
                {
                    fileGait << "\t" << param.pIn[i];
                }
                for(int i = 0; i < 18; i++)
                {
                    fileGait << "\t" << param.pEE[i];
                }
                for(int i = 0; i < 6; i++)
                {
                    fileGait << "\t" << param.bodyPE[i];
                }
                fileGait << endl;
            }
        }

        fileGait.close();
    });

}
