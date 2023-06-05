#ifndef MYTARGETTRACK_H
#define MYTARGETTRACK_H
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include "MyExceptionHandle.h"
#include "MyMath.h"
#include "MyBasic.h"

enum class enum_FeatureMatcherType{ORB};
enum class enum_MatchSifterType { MIN_DIST };

class MyOptFlowTrack
{
public:
	MyOptFlowTrack();
	~MyOptFlowTrack();
	int init(int maxCount = 500, double qLevel = 0.01, double minDist = 10.0);
	int track(cv::Mat &src);
	int drawResult(cv::Mat& dst);
	int getFeaturePoints(std::vector<cv::Point2f> &previousPoints, std::vector<cv::Point2f>& predictedPoints);
private:
	cv::Mat input,gray;   // 当前图片
	cv::Mat output,gray_prev;  // 前帧图片
	std::vector<cv::Point2f> points[2];  // point0为特征点的原来位置，point1为特征点的新位置
	std::vector<cv::Point2f> initial;    // 初始化跟踪点的位置
	std::vector<cv::Point2f> features;   // 检测的特征
	int maxCount; // 检测的最大特征数
	double qLevel;   // 特征检测的等级
	double minDist;  // 两特征点之间的最小距离
	std::vector<uchar> status;   // 跟踪特征的状态，特征的流发现为1，否则为0
	std::vector<float> err;
	int setParams(int maxCount = 500, double qLevel = 0.01, double minDist = 10.0);
};

class FeatureExtractAndMatch
{
public:
	FeatureExtractAndMatch();
	~FeatureExtractAndMatch();
	int init(cv::Mat refImg, int pointNum = 50, enum_FeatureMatcherType matcherType = enum_FeatureMatcherType::ORB,
		enum_MatchSifterType sifterType = enum_MatchSifterType::MIN_DIST);
	int init(int pointNum = 50, enum_FeatureMatcherType matcherType = enum_FeatureMatcherType::ORB,
		enum_MatchSifterType sifterType = enum_MatchSifterType::MIN_DIST);
	int Match(cv::Mat& input, cv::Mat& refer);
	int Match(cv::Mat& input);
	int Sift();
	int drawResult(cv::Mat& dst);
	//int getResult(std::vector<cv::KeyPoint>& keypointsRefer, std::vector<cv::KeyPoint>& keypointsInput, std::vector<cv::DMatch>& matches);
	int getResult(std::vector<cv::KeyPoint>& featurePointsRefer, std::vector<cv::KeyPoint>& featurePointsInput);
private:
	cv::Mat refImg;
	cv::Mat inputImg;
	cv::Mat descriptorRefer;
	std::vector<cv::KeyPoint> keypointsRefer;
	cv::Mat descriptorInput;
	std::vector<cv::KeyPoint> keypointsInput;
	std::vector<cv::DMatch> matches;
	cv::Ptr<cv::ORB> detector;
	enum_FeatureMatcherType matcherType;
	enum_MatchSifterType sifterType;
	int setParams(int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType);
	int setParams(cv::Mat refImg, int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType);
};

#endif