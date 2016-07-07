#include "cross_chasm.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto crossChasmParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	ccParam param;

	for (auto &i : params)
	{
		if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "chasmWidth")
		{
			param.chasmWidth = std::stod(i.second);
		}
		else if (i.first == "stepHeight")
		{
			param.stepHeight = std::stod(i.second);
		}
	}

	msg.copyStruct(param);
}

auto crossChasmGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
	auto &param = static_cast<const ccParam &>(param_in);

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	if (param.count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
	}

	double targetPeb_Y[13]{ 0, 0, 0, -0.1, 0, 0.1, 0, -0.1, 0, 0.1, 0, 0, 0 };
	double targetPeb_Z[13]{ -0.1, -0.175, 0, -0.275, -0.25, -0.37, 0, -0.37, -0.25, -0.275, 0, -0.175, -0.1 };
	int legSequence[13]{ -1, 2, 1, 0, -1, 2, 1, 0, -1, 2, 1, 0, -1 };
	double stepLength[13]{ 0, 0.55, 0.65, 1.04, 0, 0.75, 1.04, 0.75, 0, 1.04, 0.65, 0.55, 0 };
	double stepWidth[13]{ 0, 0.1, 0, 0.1, 0, 0, 0, 0, 0, -0.1, 0, -0.1, 0 };

	static wvParam wv_param;
	int i = param.count / param.totalCount;
	wv_param.totalCount = param.totalCount;
	wv_param.count = param.count % param.totalCount;
	wv_param.legSequence = legSequence[i];
	wv_param.stepLength = stepLength[i] * param.chasmWidth;
	wv_param.stepWidth = stepWidth[i];
	wv_param.stepHeight = param.stepHeight;
	wv_param.targetPeb[1] = targetPeb_Y[i];
	wv_param.targetPeb[2] = targetPeb_Z[i] * param.chasmWidth;
	int ret = waveGait(robot, wv_param);

	//test
	if (param.count % 100 == 0)
	{
		double testPee[18], testPeb[6];
		robot.GetPee(testPee, beginMak);
		robot.GetPeb(testPeb, beginMak);
		rt_printf("count: %d\n", param.count);
		rt_printf("i: %d\n", i);
		rt_printf("wv_param.targetPeb: %f %f %f %f %f %f\n\n", wv_param.targetPeb[0], wv_param.targetPeb[1], wv_param.targetPeb[2], wv_param.targetPeb[3], wv_param.targetPeb[4], wv_param.targetPeb[5]);
		rt_printf("testPeb: %f %f %f %f %f %f\n", testPeb[0], testPeb[1], testPeb[2], testPeb[3], testPeb[4], testPeb[5]);
		rt_printf("testPee_Y: %f %f %f %f %f %f\n", testPee[1], testPee[4], testPee[7], testPee[10], testPee[13], testPee[16]);
		rt_printf("testPee_Z: %f %f %f %f %f %f\n\n", testPee[2], testPee[5], testPee[8], testPee[11], testPee[14], testPee[17]);
	}

    return 13 * param.totalCount - param.count - 1;
}

auto waveGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotBase &>(model);
	auto &param = static_cast<const wvParam &>(param_in);

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	if (param.count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
		//rt_printf("d:%f\n", param.d);
		//rt_printf("h:%f\n", param.stepHeight);
		rt_printf("i:%d\n", param.legSequence);
		rt_printf("beginPee: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n\n", 
			beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5], 
			beginPee[6], beginPee[7], beginPee[8], beginPee[9], beginPee[10], beginPee[11], 
			beginPee[12], beginPee[13], beginPee[14], beginPee[15], beginPee[16], beginPee[17]);
	}

	double Peb[6]{ 0 }, Pee[18]{ 0 };
	const double s = -0.5 * std::cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1
	//身体插值
	for (int i = 0; i < 3; ++i)
	{
		Peb[i] = param.targetPeb[i] * s;
	}
	//足尖插值
	int j = param.legSequence;
	double d = param.stepLength;
	double h = param.stepHeight;
	double w = param.stepWidth;
	std::copy(beginPee, beginPee + 18, Pee);
	if (j >= 0)
	{
		Pee[3 * j] += w * s;
		Pee[3 * j + 1] += h * std::sin(PI * s);
		Pee[3 * j + 2] += d / 2 * (std::cos(PI * s) - 1);
		Pee[3 * j + 9] += -w * s;
		Pee[3 * j + 10] += h * std::sin(PI * s);
		Pee[3 * j + 11] += d / 2 * (std::cos(PI * s) - 1);
		//test
		//if (param.count % 100 == 0)
		//{
		//	rt_printf("\ncount:%d\n", param.count);
		//	rt_printf("j:%d\n", j);
		//	rt_printf("s:%f\n", s);
		//	rt_printf("d:%f\n", d);
		//	rt_printf("h:%f\n", h);
		//	rt_printf("Pee_Y: %f %f %f %f %f %f\n", Pee[1], Pee[4], Pee[7], Pee[10], Pee[13], Pee[16]);
		//	rt_printf("Pee_Z: %f %f %f %f %f %f\n", Pee[2], Pee[5], Pee[8], Pee[11], Pee[14], Pee[17]);
		//	rt_printf("Peb: %f %f %f %f %f %f\n\n", Peb[0], Peb[1], Peb[2], Peb[3], Peb[4], Peb[5]);
		//}
	}

	robot.SetPeb(Peb, beginMak);
	robot.SetPee(Pee, beginMak);

	return param.totalCount - param.count - 1;
}