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
/*������ȡ�㷨���*/
enum class enum_FeatureExtractor { SIFT, SURF, FREAK,  ORB, LKOF };
/*����ƥ���㷨���*/
enum class enum_FeatureMatcher { BF, HAMMING, KNN };
/*��Ӧ�Ծ�������㷨���*/
enum class enum_HomographyEstimator { LS, KNN, RANSAC, MLESAC, UPDATE };
/*�ڵ�ģ�͹����㷨���*/
enum class enum_InlierModelEstimator { LS, RANSAC, MLESAC, UPDATE };


//===========================================================================
//������ȡ��ض��󣬰�����������+����������
/*�������������������*/
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
/*������ȡ��*/
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
	//���ô�����ͼ����չؼ����������
	void setImg(cv::Mat srcImg);
	//��ȡ������������������+����������
	int extract(bool openNMS = true); 
	//��ȡ������
	std::vector<cv::KeyPoint> getKeypoints();
	//��ȡ������
	cv::Mat getDescriptors();
	//��ȡ������������ͼ��
	cv::Mat getMarkedImg();
};

void shrink2fit(std::vector<cv::KeyPoint>& keypoints1, cv::Mat& descriptors1, std::vector<cv::KeyPoint>& keypoints2, cv::Mat& descriptors2);

//===========================================================================
//����ƥ����ض���
/*����ƥ����*/
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
	//����ģ���������������
	void setTemplate(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1);
	//����ʵ���������������
	void setActual(std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2);
	//��ȡƥ��
	int match();
	//��ȡƥ����
	std::vector<cv::DMatch> getMatches();
	//��ȡ��ƥ��Ա����ͼ��
	cv::Mat getMatchedImg(cv::Mat src, cv::Mat dst);
};

//===========================================================================
//�Ӿ��ŷ�ģ����ض��󣬰����ڵ�ģ�͹��ơ����ɸ����
/*ģ����*/
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
/*������������double������ת��*/
std::vector<double> keypoints2doubles(std::vector<cv::KeyPoint> keypoints);
std::vector<cv::KeyPoint> doubles2keypoints(std::vector<double> S);
/*�ڵ�ģ�͹�����*/
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
	//ģ��������
	std::vector<double> modelPredErr(MyInlierModel& putativeModel);
	//ģ�Ͷ�����Ȼֵ����
	double nLogL(std::vector<double> modelPredErr, double mismatchRatio, double sigma, double mismatchRange);
	//ɸ����ƥ���
	void eliminateOutlier(MyInlierModel& bestPutativeModel, double sigma);
public:
	MyInlierModelEstimator(enum_InlierModelEstimator type = enum_InlierModelEstimator::LS);
	~MyInlierModelEstimator();
	//����ģ�������㡢ʵ�������㡢ƥ�����
	void set(std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2, std::vector<cv::DMatch> matches);
	//MLESAC�����ڵ�ģ��
	MyInlierModel MLESAC(int P, double mismatchRatio = 0.20, double sigma = 1.0, double mismatchRange = 100.0, double confidence = 0.95);
	//����ɸѡ���ƥ�����
	cv::Mat drawResult(cv::Mat& img1, cv::Mat& img2);
	//����ɸѡ��ʹ�õ��ڵ�
	cv::Mat drawInliers(cv::Mat& img1, cv::Mat& img2);
};


//===========================================================================
//�����װ��ض��󣬰�����������
/*ƥ�����ṹ��*/
struct matchResult {
	std::vector<cv::KeyPoint> keypoints1;
	std::vector<cv::KeyPoint> keypoints2;
	std::vector<cv::DMatch> matches;
	MyInlierModel model;
	void printResult(std::ostream& os);
	void printResult(std::fstream& fs);
};
/*�Ż�ƥ����*/
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