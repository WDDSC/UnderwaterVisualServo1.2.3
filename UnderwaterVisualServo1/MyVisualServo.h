#ifndef MYVISUALSERVO_H
#define MYVISUALSERVO_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "MyMath.h"
#include "MyBasic.h"

/*轨迹规划器类TrajPlanner及其派生类*/
enum class enum_TrajType {
	NONE,
	MODEL_UNIFORM, MODEL_PARABOLIC,
	REALTIME_ASYMPTOTIC, REALTIME_UNIFORM, REALTIME_BSPLINE, REALTIME_POLYNOM,
	ANALYTIC_ROUND, ANALYTIC_RECTANGLE
};
class TrajPlanner
{
protected:
	enum_TrajType trajType;
public:
	TrajPlanner();
	~TrajPlanner();
	TrajPlanner(enum_TrajType trajType);
	virtual MyPoint3D getCurrentTraj(double time) = 0;
};

class ModelTrajPlanner : public TrajPlanner
{
private:
	std::vector<MyPoint3D> pointArr;
	std::vector<double> timeNodeArr;
	double vel;
	double acc;
	double repeatTimes;
	double period;
	double duration;
	std::vector<double> calcTimeNodeArr();
	double calcPeriod();
	double calcDuration();
	MyPoint3D ParabolicTraj(double time);
	MyPoint3D UniformTraj(double time);
public:
	ModelTrajPlanner();
	~ModelTrajPlanner();
	ModelTrajPlanner(enum_TrajType trajType, std::vector<double> trajParams, std::vector<MyPoint3D> pointArr, double repeatTimes = 1.0);
	//void setStarTraj(double size)
	void setFixedPeriod(double newPeriod);
	void setFixedDuration(double newDuration);
	MyPoint3D getCurrentTraj(double time);
};

class RealTimeTrajPlanner : public TrajPlanner
{
private:
	int memorySize;
	std::vector<MyPoint3D> pointsArr;
	std::vector<clock_t> timeNodesPrevious;
	MyPoint3D pointNow;
	clock_t timeNow; //单位ms
	double vel;
	MyPoint3D AsymptoticTraj(double time);
	MyPoint3D UniformTraj(double time);
	MyPoint3D PolynomTraj(double time);
	//std::vector<double> BSplineTraj(std::vector<double> pStart, std::vector<double> pEnd, double time);
	//void Resample();
public:
	RealTimeTrajPlanner();
	~RealTimeTrajPlanner();
	RealTimeTrajPlanner(enum_TrajType trajType, int memorySize, MyPoint3D initP = MyPoint3D(0,0,0), double vel = 0.0);
	void NewPoint(MyPoint3D newP);
	MyPoint3D getCurrentTraj(double time = NAN);
};

class AnalyticTrajPlanner : public TrajPlanner
{
private:
	std::vector<double> trajParams;
	double repeatTimes;
	std::vector<double> roundTraj(double time);
	//std::vector<double> rectangleTraj(double time);
public:
	AnalyticTrajPlanner();
	~AnalyticTrajPlanner();
	AnalyticTrajPlanner(enum_TrajType trajType, std::vector<double> trajParams, double repeatTimes = 1.0);
	MyPoint3D getCurrentTraj(double time);
};

/*视觉伺服类VisualServo及其派生类*/
//enum class enum_VSType {
//	PBVS_IK,
//	PBVS_JAC,
//	IBVS_JAC
//};
//
//class VisualServo
//{
//public:
//	VisualServo();
//	~VisualServo();
//	
//protected:
//	enum_VSType vsType;
//	virtual std::vector<double> process();
//};

class IBVSController
{
public:
	IBVSController();
	~IBVSController();
	std::vector<double> output();
	int init(std::vector<cv::KeyPoint> desiredFeatureVector, double desiredDepth, double gain, cv::Mat cameraMatrix);
	int update(std::vector<cv::KeyPoint> actualFeatureVector, double featureDepth);
	int update(std::vector<cv::KeyPoint> actualFeatureVector, std::vector<double> featureDepth = {});
private:
	cv::Mat cameraMatrix;
	std::vector<cv::KeyPoint> desiredFeatureVector;
	std::vector<cv::KeyPoint> actualFeatureVector;
	double desiredDepth;
	std::vector<double> featureDepth;
	ImgJacMatrix imgJacMat;
	double lambda;
	std::vector<double> desiredEndVelocity;
	int setActualFeatureVector(std::vector<cv::KeyPoint> actualFeatureVector);
	int setDesiredFeatureVector(std::vector<cv::KeyPoint> desiredFeatureVector);
	int setDesiredDepth(double z0);
	int setFeatureDepth(double z);
	double estFeatureDepth();
	int setFeatureDepth(std::vector<double> Z);
	int setLinearGain(double lambda);
	int control(double u0 = 320, double v0 = 240);
};

#endif