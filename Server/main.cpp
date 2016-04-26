#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "continuous_walk_with_force.h"
#include "push_recovery.h"
#include "peg_in_hole.h"
#include "continuous_move.h"
#include "twist_waist.h"
#include "say_hello.h"

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        xml_address = "/home/hex/Desktop/RobotGaits/resource/Robot_III/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        xml_address = "/home/hex/Desktop/RobotGaits/resource/Robot_III/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();


    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    //my gaits
    rs.addCmd("cwf", CWFParse, CWFGait);
    rs.addCmd("cwfs", CWFStopParse, CWFGait);
    rs.addCmd("pr", pushRecoveryParse, pushRecoveryGait);
    rs.addCmd("prs", pushRecoveryStopParse, pushRecoveryGait);
    rs.addCmd("ph", pegInHoleParse, pegInHoleGait);
    rs.addCmd("phs", pegInHoleContinueParse, pegInHoleGait);
    rs.addCmd("cmb", parseContinuousMoveBegin, continuousMove);
    rs.addCmd("cmj", parseContinuousMoveJudge, continuousMove);
    rs.addCmd("tw", twistWaistParse, twistWaistGait);
    rs.addCmd("sh", sayHelloParse, sayHelloGait);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();



    return 0;
}
