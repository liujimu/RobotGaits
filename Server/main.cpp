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

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

#include "continuous_walk_with_force.h"

using namespace Aris::Core;

int main(int argc, char *argv[])
{
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        xml_address = "/home/hex/Desktop/RobotGaits/resource/Robot_III/Robot_III_Single_Motor.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        xml_address = "/home/hex/Desktop/RobotGaits/resource/Robot_III/Robot_III_Single_Motor.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "/home/hex/Desktop/RobotGaits/resource/Robot_VIII/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = Aris::Server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);

    //my gaits
    rs.addCmd("cwf", CWFParse, CWFGait);
    rs.addCmd("cwfs", CWFStopParse, CWFGait);

    rs.open();

    rs.setOnExit([&]()
    {
        Aris::Core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        Aris::Core::stopMsgLoop();
    });
    Aris::Core::runMsgLoop();

    return 0;
}
