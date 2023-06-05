#ifndef MYINDENTIFY_H
#define MYINDENTIFY_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "MyExceptionHandle.h"
#include "MyBasic.h"
#include "MyMath.h"

//===========================================================================
/*特征提取算法类别*/
enum class enum_FeatureExtractor { SIFT, SURF, FREAK,  ORB, LKOF };
/*特征匹配算法类别*/
enum class enum_FeatureMatcher { BF, HAMMING, KNN };
/*单应性矩阵估计算法类别*/
enum class enum_HomographyEstimator { LS, KNN, RANSAC, MLESAC, UPDATE };
/*内点模型估计算法类别*/
enum class enum_InlierModelEstimator { LS, RANSAC, MLESAC, UPDATE };


//===========================================================================
//特征提取相关对象，包括特征点检测+描述符计算
/*各类特征检测器联合体*/
union MyUnionDetector
{
	cv::Ptr<cv::SIFT> detectorSift;
	cv::Ptr<cv::xfeatures2d::SURF> detectorSurf;
	cv::Ptr<cv::xfeatures2d::FREAK> detectorFreak;
	cv::Ptr<cv::ORB> detectorOrb;
	MyUnionDetector()
	{
		//detectorSift = cv::SIFT::create();
		//detectorSurf = cv::xfeatures2d::SURF::create();
		//detectorFreak = cv::xfeatures2d::FREAK::create();
		detectorOrb = cv::ORB::create();
	}
	~MyUnionDetector(){}
};
/*特征提取器*/
class MyFeatureExtractor
{
protected:
	enum_FeatureExtractor type;
	cv::Mat srcImg;
	MyUnionDetector detector;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	void nonMaxSupress();
public:
	MyFeatureExtractor(enum_FeatureExtractor type = enum_FeatureExtractor::ORB);
	~MyFeatureExtractor();
	//设置待处理图像并清空关键点和描述符
	void setImg(cv::Mat srcImg);
	//提取特征，包括特征点检测+描述符计算
	int extract(bool openNMS = true); 
	//获取特征点
	std::vector<cv::KeyPoint> getKeypoints();
	//获取描述符
	cv::Mat getDescriptors();
	//获取将特征点标出的图像
	cv::Mat getMarkedImg();
};

void shrink2fit(std::vector<cv::KeyPoint>& keypoints1, cv::Mat& descriptors1, std::vector<cv::KeyPoint>& keypoints2, cv::Mat& descriptors2);

//===========================================================================
//特征匹配相关对象
/*特征匹配器*/
class MyFeatureMatcher
{
protected:
	enum_FeatureMatcher type;
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	cv::Mat descriptors1, descriptors2;
	cv::Mat descriptors;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::DMatch> matches;
	std::vector<std::vector<cv::DMatch>> knnMatches;
public:
	MyFeatureMatcher(enum_FeatureMatcher type = enum_FeatureMatcher::BF);
	~MyFeatureMatcher();
	//设置模板特征点和描述符
	void setTemplate(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1);
	//设置实际特征点和描述符
	void setActual(std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2);
	//提取匹配
	int match();
	//获取匹配结果
	std::vector<cv::DMatch> getMatches();
	//获取将匹配对标出的图像
	cv::Mat getMatchedImg(cv::Mat src, cv::Mat dst);
};

//===========================================================================
//视觉伺服模型相关对象，包括内点模型估计、外点筛除等
/*模型类*/
class MyInlierModel
{
private:
public:
	ImgJacMatrix L;
	std::vector<double> dP;
	double depth;
	MyInlierModel();
	MyInlierModel(std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, std::vector<cv::DMatch> matches, double z = 1.0);
	~MyInlierModel();
	std::vector<double> predict(std::vector<cv::KeyPoint> keypoints);
};
/*特征点向量与double向量的转化*/
std::vector<double> keypoints2doubles(std::vector<cv::KeyPoint> keypoints);
std::vector<cv::KeyPoint> doubles2keypoints(std::vector<double> S);
/*内点模型估计器*/
class MyInlierModelEstimator
{
protected:
	enum_InlierModelEstimator type;
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	std::vector<cv::DMatch> matches;
	std::vector<cv::KeyPoint> inliers1, inliers2;
	std::vector<cv::DMatch> inlierMatches;
	std::vector<cv::KeyPoint> keypoints2_hat;
	std::vector<cv::DMatch> predictMatches;
	MyInlierModel estModel;
	//模型误差计算
	std::vector<double> modelPredErr(MyInlierModel& putativeModel);
	//模型对数似然值计算
	double nLogL(std::vector<double> modelPredErr, double mismatchRatio, double sigma, double mismatchRange);
	//筛除误匹配点
	void eliminateOutlier(MyInlierModel& bestPutativeModel, double sigma);
public:
	MyInlierModelEstimator(enum_InlierModelEstimator type = enum_InlierModelEstimator::LS);
	~MyInlierModelEstimator();
	//设置模板特征点、实际特征点、匹配情况
	void set(std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, std::vector<cv::DMatch> matches);
	//MLESAC估计内点模型
	MyInlierModel MLESAC(int P, double mismatchRatio = 0.20, double sigma = 1.0, double mismatchRange = 100.0, double confidence = 0.95);
	//绘制筛选后的匹配情况
	cv::Mat drawResult(cv::Mat& img1, cv::Mat& img2);
	//绘制筛选后使用的内点
	cv::Mat drawInliers(cv::Mat& img1, cv::Mat& img2);
};


//===========================================================================
//整体封装相关对象，包括上述步骤
/*匹配结果结构体*/
struct matchResult {
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	std::vector<cv::DMatch> matches;
	MyInlierModel model;
	void printResult(std::ostream& os);
	void printResult(std::fstream& fs);
};
/*优化匹配类*/
class MyRefinedMatcher : private MyFeatureExtractor, private MyFeatureMatcher, private MyInlierModelEstimator
{
private:
	//MyFeatureExtractor extractor;
	//MyFeatureMatcher matcher;
	//MyInlierModelEstimator estimator;
	cv::Mat imgTemplate;
	cv::Mat imgActual;
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	std::vector<cv::DMatch> rawMatches;
	int sampleSize;
	double mismatchRatio;
	double inlierVariance;
	double outlierRange;
	double confidence;
	std::vector<cv::KeyPoint> inliers1;
	std::vector<cv::KeyPoint> inliers2;
	std::vector<cv::DMatch> inlierMatches;
	std::vector<cv::KeyPoint> keypoints2_hat;
	std::vector<cv::DMatch> refinedMatches;
	MyInlierModel model;
public:
	//MyRefinedMatcher();
	MyRefinedMatcher(enum_FeatureExtractor extractorType = enum_FeatureExtractor::ORB, 
		enum_FeatureMatcher matcherType = enum_FeatureMatcher::HAMMING, enum_InlierModelEstimator estimatorType = enum_InlierModelEstimator::MLESAC);
	~MyRefinedMatcher();
	void setParams(int sampleSize = 3, double mismatchRatio = 0.80, double inlierVariance = 1.0, double outlierRange = 100.0, double confidence = 0.95);
	void setTemplateImg(cv::Mat img);
	void setActualImg(cv::Mat img);
	void match();
	matchResult getResult();
	cv::Mat getRawMatchImg();
	cv::Mat getRefinedMatchImg();
};


#endif