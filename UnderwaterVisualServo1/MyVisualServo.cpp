#include "MyVisualServo.h"

using namespace std;
using namespace cv;

/*轨迹规划器类TrajPlanner*/
TrajPlanner::TrajPlanner() { this->trajType = enum_TrajType::NONE; }
TrajPlanner::~TrajPlanner() {}
TrajPlanner::TrajPlanner(enum_TrajType trajType) { this->trajType = trajType; }


vector<double> ModelTrajPlanner::calcTimeNodeArr()
{
	if (pointArr.size() == 0)
	{
		ReportError("路径点序列不能为空");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	if (pointArr.size() == 1) return { 0 };
	vector<double> timeNodes = { 0 };
	switch (this->trajType)
	{
	case enum_TrajType::MODEL_PARABOLIC:
	{
		if (vel == NAN || acc == NAN) {
			ReportError("须设置轨迹速度及加速度");
			throw enum_ErrType::PARAM_UNEXPECTED;
		}
		for (int i = 1; i < pointArr.size(); i++) {
			if (myNorm(pointArr[i] - pointArr[i - 1]) <= vel * vel / acc)
				timeNodes.push_back(sqrt(myNorm(pointArr[i] - pointArr[i - 1]) / acc));
			else
				timeNodes.push_back(vel / acc + myNorm(pointArr[i] - pointArr[i - 1]) / vel);
			timeNodes.push_back(myNorm(pointArr[i] - pointArr[i - 1]) / vel);
		}
	}
	case enum_TrajType::MODEL_UNIFORM:
	{
		if (vel == NAN) {
			ReportError("须设置轨迹速度");
			throw enum_ErrType::PARAM_UNEXPECTED;
		}
		for (int i = 1; i < pointArr.size(); i++)
			timeNodes.push_back(timeNodeArr[timeNodeArr.size()-1] + myNorm(pointArr[i] - pointArr[i - 1]) / vel);
	}
	default:
	{
		ReportError("意料之外的轨迹类型");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	}
}
double ModelTrajPlanner::calcPeriod()
{
	if (timeNodeArr.empty()) {
		ReportError("时间节点序列为空");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	return timeNodeArr[timeNodeArr.size() - 1];
}
double ModelTrajPlanner::calcDuration()
{
	if (period == NAN || repeatTimes == NAN) {
		ReportError("轨迹周期尚未计算或轨迹重复次数尚未赋值");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	return period * repeatTimes;
}
MyPoint3D ModelTrajPlanner::ParabolicTraj(double time)
{
	if (time <= 0) return pointArr[0];
	else if (time >= duration) return pointArr[pointArr.size() - 1];
	time = fmod(time, period);
	int index = 0;
	while (timeNodeArr[index] < time) index++;
	MyPoint3D dirVec = unify(pointArr[index] - pointArr[index - 1]);
	double t1 = (time - timeNodeArr[index - 1]);
	double t2 = (timeNodeArr[index] - time);
	double distance;
	if (t1 <= vel / acc) distance = 0.5 * acc * t1 * t1;
	else if (t2 <= vel / acc) distance = myNorm(pointArr[index] - pointArr[index - 1]) - 0.5 * acc * t2 * t2;
	else distance = vel * vel / acc + (t1 - vel / acc) * vel;
	return pointArr[index - 1] + distance * dirVec;
}
MyPoint3D ModelTrajPlanner::UniformTraj(double time)
{
	if (time <= 0) return pointArr[0];
	else if (time >= duration) return pointArr[pointArr.size() - 1];
	time = fmod(time, period);
	int index = 0;
	while (timeNodeArr[index] < time) index++;
	MyPoint3D dirVec = unify(pointArr[index] - pointArr[index - 1]);
	double distance = vel * (time - timeNodeArr[index - 1]);
	return pointArr[index - 1] + distance * dirVec;
}
ModelTrajPlanner::ModelTrajPlanner()
{
	this->pointArr = {};
	this->timeNodeArr = {};
	this->vel = NAN;
	this->acc = NAN;
	this->repeatTimes = 1.0;
	this->period = NAN;
	this->duration = NAN;
}
ModelTrajPlanner::~ModelTrajPlanner() {}
ModelTrajPlanner::ModelTrajPlanner(enum_TrajType trajType, std::vector<double> trajParams, std::vector<MyPoint3D> pointArr, double repeatTimes)
{
	this->trajType = trajType;
	this->pointArr = pointArr;
	this->vel = trajParams.size() >= 1 ? trajParams[0] : NAN;
	this->acc = trajParams.size() >= 2 ? trajParams[1] : NAN;
	if ((trajType == enum_TrajType::MODEL_UNIFORM && trajParams.size() != 1)\
		|| (trajType == enum_TrajType::MODEL_PARABOLIC && trajParams.size() != 2)) {
		ReportWarning("输入参数trajParams的元素个数有误");
	}
	if (vel <= 0 || acc <= 0) {
		ReportError("轨迹的速度与加速度须设置为大于0的正值");
		throw enum_ErrType::INPUT_INVALID;
	}
	this->repeatTimes = repeatTimes;
	this->timeNodeArr = calcTimeNodeArr();
	this->period = calcPeriod();
	this->duration = period * repeatTimes;
}
void ModelTrajPlanner::setFixedPeriod(double newPeriod)
{
	if (period == NAN) {
		ReportError("轨迹尚未设置完成");
		throw enum_ErrType::PARAM_UNASSIGNED;
	}
	switch (trajType)
	{
	case enum_TrajType::MODEL_PARABOLIC:
	{
		vel = vel * period / newPeriod;
		acc = acc * (period / newPeriod) * (period / newPeriod);
	}
	case enum_TrajType::MODEL_UNIFORM:
	{
		vel = vel * period / newPeriod;
	}
	default:
	{
		ReportError("意料之外的轨迹类型");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	}
	timeNodeArr = calcTimeNodeArr();
	period = calcPeriod();
}
void ModelTrajPlanner::setFixedDuration(double newDuration)
{
	if (period == NAN || repeatTimes == NAN) {
		ReportError("轨迹尚未设置完成");
		throw enum_ErrType::PARAM_UNASSIGNED;
	}
	setFixedPeriod(newDuration / repeatTimes);
	duration = newDuration;
}
MyPoint3D ModelTrajPlanner::getCurrentTraj(double time)
{
	if (period == NAN) {
		ReportError("轨迹尚未设置完成");
		throw enum_ErrType::PARAM_UNASSIGNED;
	}
	switch (trajType)
	{
	case enum_TrajType::MODEL_PARABOLIC:
	{
		return ParabolicTraj(time);
	}
	case enum_TrajType::MODEL_UNIFORM:
	{
		return UniformTraj(time);
	}
	default:
	{
		ReportError("意料之外的轨迹类型");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	}
	return NONEPOINT3D;
}

MyPoint3D RealTimeTrajPlanner::AsymptoticTraj(double time)
{
	pointNow = (pointNow + vel * pointsArr[memorySize - 1]) / (1 + vel);
	return pointNow;
}
MyPoint3D RealTimeTrajPlanner::UniformTraj(double time)
{
	double timeFromLastPoint = time - timeNow;
	if (timeFromLastPoint * vel >= abs(pointsArr[memorySize - 1].x - pointNow.x)) {
		pointNow.x = pointsArr[memorySize - 1].x;
	}
	else {
		pointNow.x = pointNow.x + vel * signbit(pointsArr[memorySize - 1].x - pointNow.x);
	}
	if (timeFromLastPoint * vel >= abs(pointsArr[memorySize - 1].y - pointNow.y)) {
		pointNow.y = pointsArr[memorySize - 1].y;
	}
	else {
		pointNow.y = pointNow.y + vel * signbit(pointsArr[memorySize - 1].y - pointNow.y);
	}
	if (timeFromLastPoint * vel >= abs(pointsArr[memorySize - 1].z - pointNow.z)) {
		pointNow.z = pointsArr[memorySize - 1].z;
	}
	else {
		pointNow.z = pointNow.z + vel * signbit(pointsArr[memorySize - 1].z - pointNow.z);
	}
	timeNow = clock();
	return pointNow;
}
MyPoint3D RealTimeTrajPlanner::PolynomTraj(double time)
{
	const int order = 3;
	vector<double> polynomCoef;
	polynomCoef.resize(4);
	vector<vector<double>> T;
	//polynomCoef
	return MyPoint3D();
}
//std::vector<double> RealTimeTrajPlanner::ParabolicTraj(std::vector<double> pStart, std::vector<double> pEnd, double time);
//std::vector<double> RealTimeTrajPlanner::BSplineTraj(std::vector<double> pStart, std::vector<double> pEnd, double time);
RealTimeTrajPlanner::RealTimeTrajPlanner()
{
	this->memorySize = 1;
	this->pointsArr = {NONEPOINT3D};
	this->timeNodesPrevious = {};
	this->pointNow = NONEPOINT3D;
	this->timeNow = clock();
	this->vel = 0.0;
}
RealTimeTrajPlanner::~RealTimeTrajPlanner() {}
RealTimeTrajPlanner::RealTimeTrajPlanner(enum_TrajType trajType, int memorySize, MyPoint3D initP, double vel)
{
	this->trajType = trajType;
	this->memorySize = memorySize;
	this->pointsArr = {};
	for (int i = 0; i < this->memorySize; i++) {
		this->pointsArr.push_back({ initP });
	}
	this->pointNow = initP;
	this->timeNodesPrevious = {};
	for (int i = 0; i < this->memorySize; i++) {
		this->timeNodesPrevious.push_back({ clock() });
	}
	this->vel = vel;
}
void RealTimeTrajPlanner::NewPoint(MyPoint3D newP)
{
	for (int i = 0; i < memorySize - 1; i++) {
		pointsArr[i] = pointsArr[i + 1];
	}
	pointsArr[memorySize - 1] = newP;
	for (int i = 0; i < memorySize - 1; i++) {
		timeNodesPrevious[i] = timeNodesPrevious[i + 1];
	}
	timeNodesPrevious[memorySize - 1] = clock();
}
MyPoint3D RealTimeTrajPlanner::getCurrentTraj(double time)
{
	switch (trajType)
	{
	case enum_TrajType::REALTIME_ASYMPTOTIC:
	{
		return AsymptoticTraj(time);
	}
	default:
	{
		//ReportError("Unexpected trajectory type");
		ReportError("意料之外的轨迹类型");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	}
	return AsymptoticTraj(time);
}

AnalyticTrajPlanner::AnalyticTrajPlanner() {
	this->trajType = enum_TrajType::ANALYTIC_ROUND;
	this->trajParams = {};
	this->repeatTimes = 1.0;
}
AnalyticTrajPlanner::~AnalyticTrajPlanner() {}
AnalyticTrajPlanner::AnalyticTrajPlanner(enum_TrajType trajType, std::vector<double> trajParams, double repeatTimes)
{
	this->trajType = trajType;
	this->trajParams = trajParams;
	this->repeatTimes = repeatTimes;
}
MyPoint3D AnalyticTrajPlanner::getCurrentTraj(double time)
{
	switch (trajType)
	{
	case enum_TrajType::ANALYTIC_ROUND:
	{
		return MyPoint3D(roundTraj(time));
	}
	case enum_TrajType::ANALYTIC_RECTANGLE:
	{
		ReportError("尚未实现长方形轨迹功能");
		throw enum_ErrType::IN_DEVELOPMENT;
	}
	default:
	{
		//ReportError("Unexpected trajectory type");
		ReportError("意料之外的轨迹类型");
		throw enum_ErrType::PARAM_UNEXPECTED;
	}
	}
}
std::vector<double> AnalyticTrajPlanner::roundTraj(double time)
{
	std::vector<double> trajectory;
	double trajX, trajY, trajZ;
	double centerX = trajParams[0];
	double centerY = trajParams[1];
	double centerZ = trajParams[2];
	double nX = trajParams[3];
	double nY = trajParams[4];
	double nZ = trajParams[5];
	double radius = trajParams[6];
	double velocity = trajParams[7];
	nX = nX / sqrt(nX * nX + nY * nY + nZ * nZ);
	nY = nY / sqrt(nX * nX + nY * nY + nZ * nZ);
	nZ = nZ / sqrt(nX * nX + nY * nY + nZ * nZ);

	double period = 2 * 3.1415926535 * radius / velocity;
	double theta;
	if (time <= 0) theta = 0;
	else if (time / period <= repeatTimes) theta = fmod(time, period) / period * 2 * 3.1415926535;
	else theta = fmod(repeatTimes, 1.0) * 2 * 3.1415926535;

	//std::cout << "theta = " << theta/3.1415926535*180.0 << std::endl;
	if (nX == 0 && nY == 0) {
		trajZ = 0;
		trajX = radius * cos(theta);
		trajY = radius * sin(theta);
	}
	else {
		if (sin(theta) >= 0) trajZ = radius * sqrt((1 - cos(theta) * cos(theta)) / (1 + nZ * nZ / (nX * nX + nY * nY)));
		else trajZ = -radius * sqrt((1 - cos(theta) * cos(theta)) / (1 + nZ * nZ / (nX * nX + nY * nY)));
		trajX = -(nX * nZ) / (nX * nX + nY * nY) * trajZ + nY / sqrt(nX * nX + nY * nY) * radius * cos(theta);
		trajY = -(nY * nZ) / (nX * nX + nY * nY) * trajZ - nX / sqrt(nX * nX + nY * nY) * radius * cos(theta);
	}
	trajX += centerX;
	trajY += centerY;
	trajZ += centerZ;
	trajectory.push_back(trajX);
	trajectory.push_back(trajY);
	trajectory.push_back(trajZ);
	return trajectory;
}

/*IBVS类*/
IBVSController::IBVSController() {};
IBVSController::~IBVSController() {};
std::vector<double> IBVSController::output()
{
	control();
	return desiredEndVelocity;
}
int IBVSController::init(std::vector<cv::KeyPoint> desiredFeatureVector, double desiredDepth, double gain, Mat cameraMatrix)
{
	setDesiredFeatureVector(desiredFeatureVector);
	setDesiredDepth(desiredDepth);
	setLinearGain(gain);
	this->cameraMatrix = cameraMatrix;
	return 0;
}
int IBVSController::update(std::vector<cv::KeyPoint> actualFeatureVector, double featureDepth)
{
	setActualFeatureVector(actualFeatureVector);
	setFeatureDepth(featureDepth);
	return 0;
}
int IBVSController::update(std::vector<cv::KeyPoint> actualFeatureVector, std::vector<double> featureDepth)
{
	setActualFeatureVector(actualFeatureVector);
	if (featureDepth.empty()) {
		estFeatureDepth();
	}
	else {
		setFeatureDepth(featureDepth);
	}
	return 0;
}
int IBVSController::setActualFeatureVector(std::vector<cv::KeyPoint> actualFeatureVector)
{
	this->actualFeatureVector = actualFeatureVector;
	return 0;
}
int IBVSController::setDesiredFeatureVector(std::vector<cv::KeyPoint> desiredFeatureVector)
{
	this->desiredFeatureVector = desiredFeatureVector;
	return 0;
}
int IBVSController::setDesiredDepth(double z0)
{
	this->desiredDepth = z0;
	return 0;
}
int IBVSController::setFeatureDepth(double z)
{
	featureDepth.resize(desiredFeatureVector.size());
	for (int i = 0; i < desiredFeatureVector.size(); i++) {
		featureDepth[i] = desiredDepth + z;
	}
	return 0;
}
int IBVSController::setFeatureDepth(vector<double> Z)
{
	for (int i = 0; i < featureDepth.size(); i++) {
		featureDepth[i] = desiredDepth + Z[i];
	}
	return 0;
}
double IBVSController::estFeatureDepth()
{
	vector<Point2f> srcPoints, dstPoints;
	if (desiredFeatureVector.empty() || desiredFeatureVector.size() <= 4) {
		for (int i = 0; i < desiredFeatureVector.size(); i++) {
			this->featureDepth[i] = this->desiredDepth;
		}
		return desiredDepth;
	}
	for (int i = 0; i < this->desiredFeatureVector.size(); i++) {
		srcPoints.push_back(this->actualFeatureVector[i].pt);
		dstPoints.push_back(this->desiredFeatureVector[i].pt);
	}
	Mat H = findHomography(srcPoints, dstPoints);
	vector<Mat> rotMats, transMats, normalMats;
	decomposeHomographyMat(H, cameraMatrix, rotMats, transMats, normalMats);
	featureDepth.resize(desiredFeatureVector.size());
	for (int i = 0; i < desiredFeatureVector.size(); i++) {
		this->featureDepth[i] = mat2vec(transMats[0])[2][0];
	}
	if (isnan(mat2vec(transMats[0])[2][0])) {
		;
	}
	return mat2vec(transMats[0])[2][0];
}
int IBVSController::setLinearGain(double lambda)
{
	this->lambda = lambda;
	return 0;
}
int IBVSController::control(double u0, double v0)
{
	vector<double> U, V, E;
	for (int i = 0; i < actualFeatureVector.size(); i++) {
		U.push_back(actualFeatureVector[i].pt.x - u0);
		V.push_back(actualFeatureVector[i].pt.y - v0);
		E.push_back(actualFeatureVector[i].pt.x - desiredFeatureVector[i].pt.x);
		E.push_back(actualFeatureVector[i].pt.y - desiredFeatureVector[i].pt.y);
	}
	imgJacMat = ImgJacMatrix(U, V, featureDepth);
	desiredEndVelocity = -lambda * pinv(imgJacMat) * E;
	return 0;
}