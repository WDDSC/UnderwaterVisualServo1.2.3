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
	cv::Mat input,gray;   // ��ǰͼƬ
	cv::Mat output,gray_prev;  // ǰ֡ͼƬ
	std::vector<cv::Point2f> points[2];  // point0Ϊ�������ԭ��λ�ã�point1Ϊ���������λ��
	std::vector<cv::Point2f> initial;    // ��ʼ�����ٵ��λ��
	std::vector<cv::Point2f> features;   // ��������
	int maxCount; // �������������
	double qLevel;   // �������ĵȼ�
	double minDist;  // ��������֮�����С����
	std::vector<uchar> status;   // ����������״̬��������������Ϊ1������Ϊ0
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