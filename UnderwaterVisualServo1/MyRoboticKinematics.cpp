#include "MyRoboticKinematics.h"
using namespace std;

/*RobotLink类*/
//封装了机械臂关节的D-H参数以及其他参数
PoseMatrix RobotLink::CalcPoseMat(double jointAngle)
{
	double mat[4][4] = { \
		cos(jointAngle), -sin(jointAngle) * cos(alpha), sin(jointAngle) * sin(alpha), a * cos(jointAngle),\
		sin(jointAngle), cos(jointAngle) * cos(alpha), -cos(jointAngle) * sin(alpha), a * sin(jointAngle),\
		0, sin(alpha), cos(alpha), d,\
		0,0,0,1
	};
	return PoseMatrix(mat);
}
PoseMatrix RobotLink::CalcPoseMat()
{
	return CalcPoseMat(this->q);
}
RobotLink::RobotLink()
{
	q = 0;
	a = 0;
	d = 0;
	alpha = 0;
	canRotate = true;
	upperLimit = 3.1415926535;
	lowerLimit = -3.1415926535;
}
RobotLink::~RobotLink() {}
void RobotLink::setLinkParam(double q, double a, double d, double alpha)
{
	if (q != 0) {
		this->canRotate = false;
		setLinkLimit(q, q);
	}
	else {
		this->canRotate = true;
		setLinkLimit();
	}
	this->q = q / 180 * 3.1415926535;
	this->a = a;
	this->d = d;
	this->alpha = alpha / 180 * 3.1415926535;
}
void RobotLink::setLinkLimit(double lower, double upper)
{
	this->lowerLimit = lower / 180.0 * 3.1415926535;
	this->upperLimit = upper / 180.0 * 3.1415926535;
}

/*RoboticKinematics类*/
//提供多关节机械臂的实例化以及运动学相关算法接口
RoboticKinematics::RoboticKinematics() {}
RoboticKinematics::~RoboticKinematics() {}
void RoboticKinematics::AddLink(RobotLink link)
{
	linkSerial.push_back(link);
}
void RoboticKinematics::AddLink(double a, double d, double alpha)
{
	RobotLink link = RobotLink();
	link.setLinkParam(0, a, d, alpha);
	linkSerial.push_back(link);
}
void RoboticKinematics::AddLink(double q, double a, double d, double alpha)
{
	RobotLink link = RobotLink();
	link.setLinkParam(q, a, d, alpha);
	linkSerial.push_back(link);
}
std::vector<RobotLink>  RoboticKinematics::setRobot(std::vector<std::vector<double>> DHParams)
{
	RobotLink link = RobotLink();
	std::vector<RobotLink> robot;
	for (int iLink = 0; iLink < DHParams.size(); iLink++) {
		if (DHParams[iLink].size() == 3) {
			link.setLinkParam(0, DHParams[iLink][0], DHParams[iLink][1], DHParams[iLink][2]);
			link.setLinkLimit();
			robot.push_back(link);
		}
		else if (DHParams[iLink].size() == 4) {
			link.setLinkParam(DHParams[iLink][0], DHParams[iLink][1], DHParams[iLink][2], DHParams[iLink][3]);
			link.setLinkLimit();
			robot.push_back(link);
		}
		else { throw; }
	}
	linkSerial = robot;
	return robot;
}
std::vector<RobotLink>  RoboticKinematics::setDefaultRobot()
{
	std::vector<RobotLink> defaultRobot = setRobot(defaultDHParams);
	setJointAngleLimit(defaultThetaLimit);
	return defaultRobot;
}
std::vector<RobotLink> RoboticKinematics::setPredefinedRobot(enum_RobotType robotType)
{
	switch (robotType) {
	case enum_RobotType::ELECTRIC_PREDFINED: {
		return setDefaultRobot();
		break;
	}
	case enum_RobotType::HYDRUALIC_PREDFINED: {
		//目前使用的机械臂D-H参数（从左到右为q,a,d,alpha，从上到下为不同关节）
		std::vector<std::vector<double>> const hydralicDHParams = \
		{	{ 0,	101,	0,	90},
			{ 0,	605,	0,	0 },
			{ 0,	284.4,	0,	0 },
			{ 0,	484.4,	0,	90 }};
		//std::vector<double> const hydralicLinkLengths = { hydralicDHParams[0][1], hydralicDHParams[1][1], hydralicDHParams[2][1], hydralicDHParams[3][1] };
		std::vector<std::vector<double>> const hydralicThetaLimit = { {-180,180},{-180,180},{-180,180},{-180,180} };//目前的1~4关节角度上下限（可改动）
		std::vector<RobotLink> hydralicRobot = setRobot(hydralicDHParams);
		//setJointAngleLimit(hydralicThetaLimit);
		return hydralicRobot;
		break;
	}
	default: {
		return setDefaultRobot();
		break;
	}
	}
}
;
void RoboticKinematics::setJointAngleLimit(std::vector<std::vector<double>> thetaLimit)
{
	int iLimit = 0;
	for (int iLink = 0; iLink < linkSerial.size(); iLink++) {
		if (!linkSerial[iLink].canRotate) {
			continue;
		}
		else {
			linkSerial[iLink].setLinkLimit(thetaLimit[iLimit][0], thetaLimit[iLimit][1]);
			iLimit++;
		}
	}
}
bool RoboticKinematics::IsValidAngles(std::vector<double> jointAngles)
{
	int iAngle = 0;
	for (int iLink = 0; iLink < linkSerial.size(); iLink++) {
		if (linkSerial[iLink].canRotate) {
			if (jointAngles[iAngle] / 180 * 3.1415926535 < linkSerial[iLink].lowerLimit || jointAngles[iAngle] / 180 * 3.1415926535 > linkSerial[iLink].upperLimit)
				return false;
			if (isnan(jointAngles[iAngle]))
				return false;
			iAngle++;
		}
		else continue;
	}
	if (iAngle != jointAngles.size())return false;
	return true;
}
PoseMatrix RoboticKinematics::ForwardKinematics(std::vector<double>& jointAngles, std::vector<double>& endPose)
{
	if (!IsValidAngles(jointAngles)) {
		std::cout << "[ERROR] Invalid joint angles are input!!!" << std::endl;
		throw -1;
	}
	PoseMatrix endTransMat = PoseMatrix();
	//endTransMat.IdentityMat();
	endTransMat = baseCoordinateMat;
	int iAngle = 0;
	for (int iLink = 0; iLink < linkSerial.size(); iLink++) {
		if (linkSerial[iLink].canRotate)
			endTransMat = endTransMat * linkSerial[iLink].CalcPoseMat(jointAngles[iAngle++] / 180.0 * 3.1415926535);
		else
			endTransMat = endTransMat * linkSerial[iLink].CalcPoseMat();
	}
	endTransMat.EliminateSmallValue();
	endTransMat = endTransMat * handCoordinateResetMat;
	if (!endTransMat.IsValid()) {
		std::cout << "[ERROR] Solved pose matrix is invalid!!!" << std::endl;
		throw -1;
	}
	endPose.clear();
	endPose.resize(6);
	endPose[0] = endTransMat.elements[0][3];
	endPose[1] = endTransMat.elements[1][3];
	endPose[2] = endTransMat.elements[2][3];
	endPose[3] = 0 + atan2(endTransMat.elements[2][1], endTransMat.elements[2][2]) * 180.0 / 3.1415926535;
	endPose[4] = 0 + atan2(-endTransMat.elements[2][0], sqrt(endTransMat.elements[2][1] * endTransMat.elements[2][1] + endTransMat.elements[3][3] * endTransMat.elements[3][3])) * 180.0 / 3.1415926535;
	endPose[5] = 0 + atan2(endTransMat.elements[1][0], endTransMat.elements[0][0]) * 180.0 / 3.1415926535;
	if (endPose[3] > 180) endPose[3] -= 360;
	if (endPose[4] > 180) endPose[4] -= 360;
	if (endPose[5] > 180) endPose[5] -= 360;
	return endTransMat;
}
PoseMatrix RoboticKinematics::InverseKinematics3DofPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, double relaDepthDesired)
{
	PoseMatrix poseMatOnBase = inv(baseCoordinateMat) * TransMat(endPosition[0], endPosition[1], endPosition[2]);
	double L1, L2, L3;
	L1 = defaultLinkLengths[0];
	L2 = defaultLinkLengths[1];
	L3 = defaultLinkLengths[2] + relaDepthDesired;//！！！！！！！！！！！！
	double x, y, z, d;
	x = poseMatOnBase.elements[0][3];
	y = poseMatOnBase.elements[1][3];
	z = poseMatOnBase.elements[2][3] - L1;
	d = sqrt(x * x + y * y);
	jointAngles.clear();
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	double q1, q2, q3, q4;
	try {
		q1 = atan2(y, x);
		//q3 = -abs(acos((d * d + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3)));
		//q2 = asin(L3 / sqrt(z * z + d * d) * sin(-q3)) + atan2(z, d);
		q3 = abs(acos((d * d + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3)));
		q2 = asin(L3 / sqrt(z * z + d * d) * sin(-q3)) + atan2(z, d);
		q4 = 0;
		jointAngles[0] = q1 * 180.0 / 3.1415926535;
		jointAngles[1] = q2 * 180.0 / 3.1415926535;
		jointAngles[2] = q3 * 180.0 / 3.1415926535;
		jointAngles[3] = q4 * 180.0 / 3.1415926535;
		if (!IsValidAngles(jointAngles)) throw - 1;
	}
	catch (...) {
		std::cout << "[ERROR] Invalid inverse kinematics input!!!" << std::endl;
		throw - 1;
	}
	std::vector<double> endPose;
	return ForwardKinematics(jointAngles, endPose);
}
PoseMatrix RoboticKinematics::InverseKinematics3DofToward(std::vector<double>& jointAngles, std::vector<double>& towardPosition, double q23)
{
	PoseMatrix poseMatOnBase = inv(baseCoordinateMat) * TransMat(towardPosition[0], towardPosition[1], towardPosition[2]);
	double L1, L2, L3;
	L1 = defaultLinkLengths[0];
	L2 = defaultLinkLengths[1];
	L3 = defaultLinkLengths[2];
	double x, y, z, d;
	x = poseMatOnBase.elements[0][3];
	y = poseMatOnBase.elements[1][3];
	z = poseMatOnBase.elements[2][3] - L1;
	d = sqrt(x * x + y * y);
	jointAngles.clear();
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	jointAngles.push_back(0);
	double q1, q2, q3, q4;
	try {
		q1 = atan2(y, x);
		double temp = (d * tan(q23) - z) / L2 / sqrt(1 + tan(q23) * tan(q23));
		if (temp > 1) q3 = 3.1415926535/2;
		else if (temp < -1) q3 = -3.1415926535 / 2;
		else
			q3 = asin((d * tan(q23) - z) / L2 / sqrt(1 + tan(q23) * tan(q23)));
		q2 = q23 - q3;
		q4 = 0;
		jointAngles[0] = q1 * 180.0 / 3.1415926535;
		jointAngles[1] = q2 * 180.0 / 3.1415926535;
		jointAngles[2] = q3 * 180.0 / 3.1415926535;
		jointAngles[3] = q4 * 180.0 / 3.1415926535;
		if (!IsValidAngles(jointAngles)) throw -1;
	}
	catch (...) {
		std::cout << "[ERROR] Invalid inverse kinematics input!!!" << std::endl;
		throw -2;
	}
	std::vector<double> endPose;
	return ForwardKinematics(jointAngles, endPose);
}
double RoboticKinematics::InverseKinematicsGDPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, std::vector<double> jointAnglesInit)
{
	vector<double> q = jointAnglesInit;
	vector<double> q_last = jointAnglesInit;
	vector<double> q_try, q_left, q_right;
	vector<double> Pd = endPosition;
	vector<double> P, E, P_try, E_try, P_left, E_left;
	double rate_init = pow(10, -2);
	double rate = rate_init;
	double gamma = 2.0;//因为要用二分法，必须2.0
	double err = DBL_MAX;
	double err_try = 0.0;
	double err_left, err_right;
	const double rate_thresh = pow(10, -5);
	const double rate_max = 1;
	const double err_thresh = pow(10, -3);
	const int iter_threshold = 1000;
	int iterations = 0;
	vector<vector<double>> J;
	vector<double> grad;
	while (iterations < iter_threshold)
	{
		//主循环，出发点不变
		iterations++;
		//计算出发点梯度
		J = calcJacobianNumerical(q);
		J = truncate(J, { 0,1,2 }, {});//截取3维位置的雅克比矩阵
		//计算出发点误差
		ForwardKinematics(q, P);
		P = truncate(P, { 0,1,2 });
		E = P - Pd;
		err = myNorm(E);
		if (err <= err_thresh)
			break;
		//进入内循环，从出发点向梯度方向延伸
		//找到凹陷
		q_left = q;
		err_left = err;
		rate = rate_init;
		while (true) {
			iterations++;
			rate *= gamma;
			if (rate > rate_max) {
				rate = rate_max;
			}
			q_try = q - rate * (transpose(J) * E);
			//q_try = q - rate * unify(transpose(J) * E);
			ForwardKinematics(q_try, P_try);
			P_try = truncate(P_try, { 0,1,2 });
			E_try = P_try - Pd;
			err_try = myNorm(E_try);
			if(err_try < err_left){
				q_left = q_try;
				err_left = err_try;
			}
			else {
				q_right = q_try;
				err_right = err_try;
				q_left = (q + q_left) * 0.5;
				ForwardKinematics(q_left, P_left);
				P_left = truncate(P_left, { 0,1,2 });
				E_left = P_left - Pd;
				err_left = myNorm(E_left);
				break;
			}
		}
		//二分查找
		while (myNorm(q_right - q_left) > 2*rate_thresh) {
			iterations++;
			q_try = (q_left + q_right) * 0.5;
			ForwardKinematics(q_try, P_try);
			P_try = truncate(P_try, { 0,1,2 });
			E_try = P_try - Pd;
			err_try = myNorm(E_try);
			if (err_left < err_right) {
				q_right = q_try;
				err_right = err_try;
			}
			else {
				q_left = q_try;
				err_left = err_try;
			}
		}
		q_last = q;
		q = (q_left + q_right) * 0.5;
		//cout << "residual: "<<(err_left + err_right) * 0.5 << endl;
		//cout << E << endl;
		//cout << "Jg: " << transpose(J)*transpose(arr2vec(grad)) << endl;
	}
	cout << "Iterations: " << iterations << endl;
	jointAngles = q;
	return err;
}
double RoboticKinematics::InverseKinematicsGNPosition(std::vector<double>& jointAngles, std::vector<double>& endPosition, std::vector<double> jointAnglesInit)
{
	vector<double> q = jointAnglesInit;
	vector<double> q_last = jointAnglesInit;
	vector<double> q_try;
	vector<double> Pd = endPosition;
	vector<double> P, E, P_try, E_try;
	double rate_init = pow(10, -2);
	double rate = rate_init;
	double gamma = 1/0.618;
	double err = DBL_MAX;
	double err_try;
	const double rate_thresh = pow(10, -5);
	const double rate_max = 1;
	const double err_thresh = pow(10, -3);
	const int iter_threshold = 1000;
	int iterations = 0;
	vector<vector<double>> J;
	while (iterations < iter_threshold)
	{
		try {
			iterations++;
			J = calcJacobianNumerical(q);
		}
		catch (...)
		{
			;//沿用上一迭代的J
		}
		J = truncate(J, { 0,1,2 }, {});//截取3维位置的雅克比矩阵
		ForwardKinematics(q, P);
		P = truncate(P, { 0,1,2 });
		E = P - Pd;
		err = myNorm(E);
		if (err <= err_thresh)
			break;
			q_try = q - rate * unify(pinv_damp(J, 0.001) * E);
		try {
			ForwardKinematics(q_try, P_try);
		}
		catch (...)
		{
			rate = rate / gamma;
			q = q_last;
			continue;
		}
		P_try = truncate(P_try, { 0,1,2 });
		E_try = P_try - Pd;
		err_try = myNorm(E_try);
		//学习率更新方法1
		if (err_try < err) {
			q = q_try;
			if (cosine(E - E_try, E) > 0) {
				rate = rate * gamma;
				if (rate > rate_max) {
					rate = rate_max;
				}
			}
		}
		else {
			//不能回退，否则更慢
			//if (cosine(E - E_try, E) < -0.707)
			rate = rate / gamma;
		}
		////学习率更新方法2
		//if (err_try < err) {
		//	q = q_try;
		//}
		//rate = rate * pow(gamma, (cosine(E - E_try, E)));
		if (rate <= rate_thresh)
			break;
	}
	q_last = q;
	jointAngles = q;
	cout << "Iterations: " << iterations << endl;
	return err;
}
vector<vector<double>> RoboticKinematics::calcJacobianNumerical(vector<double> jointAngles)
{
	double perturbation = pow(1.0, -6);
	vector<double> jointAngles1, jointAngles2;
	vector<vector<double>> J, Ji;
	vector<double> P1, P2;
	int iAngle = -1;
	for (int iLink = 0; iLink < linkSerial.size(); iLink++) {
		if (!linkSerial[iLink].canRotate)
			continue;
		else
			iAngle++;
		P1.clear();
		P2.clear();
		Ji.clear();
		jointAngles1 = jointAngles;
		jointAngles2 = jointAngles;
		jointAngles1[iAngle] += perturbation;
		jointAngles2[iAngle] -= perturbation;
		ForwardKinematics(jointAngles1, P1);
		ForwardKinematics(jointAngles2, P2);
		Ji =  arr2vec(P1 - P2) * (1.0 / 2.0 / perturbation);
		J.push_back(Ji[0]);
	}
	return transpose(J);
}