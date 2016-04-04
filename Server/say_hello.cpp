#include "say_hello.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

/*������ע�ʹ������ӵ�xml�ļ�*/
/*
            <sh default="sh_param">
                <sh_param type="group">
                    <totalCount abbreviation="t" type="int" default="3000"/>
                    <stepLength abbreviation="d" type="double" default="0.2"/>
                    <height abbreviation="h" type="double" default="0.05"/>
                    <rollMax abbreviation="r" type="double" default="5"/>
                    <helloTimes abbreviation="n" type="int" default="1"/>
                </sh_param>
            </sh>
*/

auto sayHelloParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	shParam param;

	for (auto &i : params)
	{
		if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "stepLength")
		{
			param.stepLength = std::stod(i.second);
		}
		else if (i.first == "height")
		{
			param.height = std::stod(i.second);
		}
		else if (i.first == "pitchMax")
		{
			param.pitchMax = std::stod(i.second) / 180 * PI;
		}
		else if (i.first == "helloTimes")
		{
			param.helloTimes = std::stoi(i.second);
		}
	}

	msg.copyStruct(param);
}

auto sayHelloGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
	auto &param = static_cast<const shParam &>(param_in);

	//��ʼ��
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	int totalCount = param.totalCount;
	int n = param.helloTimes;
	const double stepHeight{ 0.05 };
    const double helloAmplitude{ 0.04 };

	if (param.count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
	}

	double footDist = std::sqrt(std::pow(beginPee[1], 2) + std::pow(beginPee[2], 2));
	double theta = std::atan2(std::fabs(beginPee[2]), std::fabs(beginPee[1]));

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);

	double targetPeb[6];
	std::fill(targetPeb, targetPeb + 6, 0);
	targetPeb[1] = param.height;
	targetPeb[4] = param.pitchMax;

	double state1Pee[18];
	std::copy(beginPee, beginPee + 18, state1Pee);
	state1Pee[5] -= param.stepLength;
	state1Pee[14] -= param.stepLength;

	double state2Pee[18];
	std::copy(state1Pee, state1Pee + 18, state2Pee);
	for (int i = 0; i < 2; ++i)
	{
		state2Pee[9 * i + 1] += (footDist * param.pitchMax * std::sin(theta) + param.height);
		state2Pee[9 * i + 2] -= footDist * param.pitchMax * std::cos(theta);
	}
    //for test
    if(param.count == 0)
    {
        rt_printf("state2Pee:%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f\n\n",
                  state2Pee[0], state2Pee[1], state2Pee[2], state2Pee[3], state2Pee[4], state2Pee[5],
                state2Pee[6], state2Pee[7], state2Pee[8], state2Pee[9], state2Pee[10], state2Pee[11],
                state2Pee[12], state2Pee[13], state2Pee[14], state2Pee[15], state2Pee[16], state2Pee[17]);
    }

	//��һ�������м���
	if (param.count < param.totalCount)
	{
		const double s = -PI/2 * cos(PI * (param.count + 1) / param.totalCount) + PI/2; //s��0��PI.
        std::copy(beginPee, beginPee + 18, Pee);
        for (int i = 0; i < 2; ++i)
		{
			Pee[9 * i + 4] += stepHeight * sin(s);
			Pee[9 * i + 5] += param.stepLength / 2 * (cos(s) - 1);
		}
	}
	//�ڶ�����̧ͷ��̧ǰ��
	else if (param.count < 2 * param.totalCount)
	{
		const double s = -0.5 * cos(PI * (param.count + 1 - param.totalCount) / param.totalCount) + 0.5; //s��0��1.
		for (int i = 0; i < 6; ++i)
		{
			Peb[i] = targetPeb[i] * s;
		}
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 0; i < 2; ++i)
		{
			Pee[9 * i + 1] = state1Pee[9 * i + 1] * (1 - s) + state2Pee[9 * i + 1] * s;
			Pee[9 * i + 2] = state1Pee[9 * i + 2] * (1 - s) + state2Pee[9 * i + 2] * s;
		}
	}
    //����������ǰ�ȴ��к�
    else if (param.count < (2 + n) * param.totalCount)
    {
        const double s = 2 * PI * (param.count + 1 - 2 * param.totalCount) / param.totalCount; //s��0��2*n*PI.
        std::copy(targetPeb, targetPeb + 6, Peb);
        std::copy(state2Pee, state2Pee + 18, Pee);
        Pee[1] = state2Pee[1] + helloAmplitude * sin(s);
        Pee[10] = state2Pee[10] - helloAmplitude * sin(s);
        //for test
        if (param.count % 100 == 99)
        {
            rt_printf("count:%d\n", param.count);
            rt_printf("s:%f\n", s);
            rt_printf("Peb:%f %f %f %f %f %f\n", Peb[0], Peb[1], Peb[2], Peb[3], Peb[4], Peb[5]);
            rt_printf("Peb:%f %f %f %f\n\n", Pee[1], Pee[10], Pee[2], Pee[11]);
        }
    }
    //���Ĳ�����ǰ�ȣ��ָ�������̬
    else if (param.count < (3 + n) * param.totalCount)
    {
        const double s = -0.5 * cos(PI * (param.count + 1 - (2 + n) * param.totalCount) / param.totalCount) + 0.5; //s��0��1.
        for (int i = 0; i < 6; ++i)
        {
            Peb[i] = targetPeb[i] * (1 - s);
        }
        std::copy(state2Pee, state2Pee + 18, Pee);
        for (int i = 0; i < 2; ++i)
        {
            Pee[9 * i + 1] = state2Pee[9 * i + 1] * (1 - s) + state1Pee[9 * i + 1] * s;
            Pee[9 * i + 2] = state2Pee[9 * i + 2] * (1 - s) + state1Pee[9 * i + 2] * s;
        }
        //for test
        if (param.count % 100 == 0)
        {
            rt_printf("step 4\n");
            rt_printf("count:%d\n", param.count);
            rt_printf("s:%f\n", s);
            rt_printf("Peb:%f %f %f %f %f %f\n", Peb[0], Peb[1], Peb[2], Peb[3], Peb[4], Peb[5]);
            rt_printf("Peb:%f %f %f %f\n\n", Pee[1], Pee[10], Pee[2], Pee[11]);
        }
    }
    //���岽�����м���
    else
    {
        const double s = -PI / 2 * cos(PI * (param.count + 1 - (3 + n) * param.totalCount) / param.totalCount) + PI / 2; //s��0��PI.
        std::copy(state1Pee, state1Pee + 18, Pee);
        for (int i = 0; i < 2; ++i)
        {
            Pee[9 * i + 4] += stepHeight * sin(s);
            Pee[9 * i + 5] -= param.stepLength / 2 * (cos(s) - 1);
        }
    }

	robot.SetPeb(Peb, beginMak);
	robot.SetPee(Pee, beginMak);

    return (4 + n) * param.totalCount - param.count - 1;
}