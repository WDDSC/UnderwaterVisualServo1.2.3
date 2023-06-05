#ifndef MYROBOTICKINEMATICS_H
#define MYROBOTICKINEMATICS_H

#include <iostream>
#include <vector>
#include "MyMath.h"

/*RobotLink类*/
//封装了机械臂关节的D-H参数以及其他参数
class RobotLink
{
private:

public:
	double q;//D-H参数q，关节转角
	double a;//D-H参数a，连杆长度
	double d;//D-H参数d，连杆在转轴方向的偏移
	double alpha;//D-H参数alpha，与下一个关节之间的扭转角
	bool canRotate;//标示该关节是否可以转动（虚拟关节不可转动，实际的4个关节可以转动）
	double upperLimit;//关节转角上限
	double lowerLimit;//关节转角下限
	PoseMatrix CalcPoseMat(double jointAngle);//计算指定转角时，该关节的位姿变换矩阵
	PoseMatrix CalcPoseMat();//计算默认转角下的位姿变换矩阵
	RobotLink();
	~RobotLink();
	void setLinkParam(double q, double a, double d, double alpha);//用D-H参数设置关节
	void setLinkLimit(double lower = -DBL_MAX, double upper = DBL_MAX);//设置该关节的转动角度上下限
};

/*RoboticKinematics类*/
//提供多关节机械臂的实例化以及运动学相关算法接口
//enum class IK_TYPE { POSITION, TOWARD };
enum class enum_RobotType {ELECTRIC_PREDFINED, HYDRUALIC_PREDFINED, NON_PREDFINED};
class RoboticKinematics
{
private:
	std::vector<RobotLink> linkSerial;//按顺序存储机械臂的各个关节（RobotLink）
	//目前使用的机械臂D-H参数（从左到右为q,a,d,alpha，从上到下为不同关节）
	std::vector<std::vector<double>> const defaultDHParams = \
	{ {0, 0, 129.5, 90},//1关节，参数d=122.5（1关节到2关节的连杆长度122.5mm）可以根据实际模型的改动
		{ 0,414,-119.5,0 },//2关节，参数a=414（2关节到3关节的连杆长度414mm）可以根据实际模型的改动
		{ 0,0,119.5,-90 },//3关节，可以根据实际模型的改动
		{ 90,0,0,90 },
		{ 0,0,264,0 }};//4关节，参数d=264（4关节到末端长度254mm,10月15日加长10mm）可以根据实际模型的改动
	std::vector<double> const defaultLinkLengths = { defaultDHParams[0][2], defaultDHParams[1][1], defaultDHParams[4][2] };
	//std::vector<std::vector<double>> const defaultThetaLimit = { {-180,180},{-180,180},{-180,180},{-180,180} };//目前的1~4关节角度上下限（可改动）
	std::vector<std::vector<double>> const defaultThetaLimit = { {-DBL_MAX,DBL_MAX},{-DBL_MAX,DBL_MAX},{-DBL_MAX,DBL_MAX},{-DBL_MAX,DBL_MAX} };//目前的1~4关节角度上下限（可改动）
	//double handCoordinateResetMatVal[4][4] = { {0,0,1,0},{-1,0,0,0},{0,-1,0,0},{0,0,0,1} };
	//PoseMatrix handCoordinateResetMat = inv(PoseMatrix(handCoordinateResetMatVal));
	double handCoordinateResetMatVal[4][4] = { {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1} };
	PoseMatrix handCoordinateResetMat = PoseMatrix(handCoordinateResetMatVal);
	double baseCoordinateMatVal[4][4] = { {1,0,0,0},{0,-1,0,0},{0,0,-1,0},{0,0,0,1} };
	PoseMatrix baseCoordinateMat = PoseMatrix(baseCoordinateMatVal);

public:
	RoboticKinematics();
	~RoboticKinematics();
	void AddLink(RobotLink link);//为机械臂对象增加连杆
	void AddLink(double a, double d, double alpha);//根据D-H参数增加连杆
	void AddLink(double q, double a, double d, double alpha);//根据D-H参数增加连杆
	std::vector<RobotLink> setRobot(std::vector<std::vector<double>> DHParams);//设置机械臂的D-H参数
	std::vector<RobotLink> setDefaultRobot();//设置为默认机械臂
	std::vector<RobotLink> setPredefinedRobot(enum_RobotType robotType);//设置为预设机械臂（目前有电动1和液压2）
	void setJointAngleLimit(std::vector<std::vector<double>> thetaLimit);//设置机械臂各关节角度上下限
	bool IsValidAngles(std::vector<double> jointAngles);//判断关节角度是否超出限位
	PoseMatrix ForwardKinematics(std::vector<double>& jointAngles, std::vector<double>& endPose);//计算正运动学（根据关节角度jointAngles得出末端位姿endPose）
	PoseMatrix InverseKinematics3DofPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, double relaDepthDesired = 0);//计算逆运动学（根据末端位置endPosition得出关节角度jointAngles）
	PoseMatrix InverseKinematics3DofToward(std::vector<double>& jointAngles, std::vector<double>& towardPosition, double q23 = 0);;//计算逆运动学（根据朝向点位置towardPosition得出关节角度jointAngles）
	//PoseMatrix InverseKinematics_4DOF(std::vector<double>& jointAngles, std::vector<double>& endPosition);//计算逆运动学（根据末端位置endPosition得出关节角度jointAngles）
	double InverseKinematicsGDPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, std::vector<double> jointAnglesInit);//基于雅克比矩阵的通用3自由度逆运动学
	double InverseKinematicsGNPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, std::vector<double> jointAnglesInit);//基于雅克比矩阵的通用3自由度逆运动学
	std::vector<std::vector<double>> calcJacobianNumerical(std::vector<double> jointAngles);
};

/*运动学Demo*/
int demoKinematics();

#endif