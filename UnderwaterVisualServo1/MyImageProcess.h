#ifndef MYIMGPROCESS_H
#define MYIMGPROCESS_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "MyExceptionHandle.h"
#include "MyMath.h"

enum class enum_PreprocessorType { NONE, CVT_GRAY, INVERSE_COLOR, HIST_EQUAL, DARK_CHANNEL, FILTER, THRESH, HIST_MATCH };
enum class enum_HistEqualType { SINGLE_CHANNEL_GLOBAL, MULTI_CHANNEL_GLOBAL, SINGLE_CHANNEL_LOCAL, MULTI_CHANNEL_LOCAL };
enum class enum_FilterType { NONE_FILTER, MEAN_BLUR, MEDIAN_BLUR, MAXIMUM_BLUR, MINIMUM_BLUR, GUASSIAN_BLUR, SHARPEN_FILTER };
enum class enum_ProcessorType { NONE_PROCESS, ARUCO_SINGLE, ARUCO_BOARD, ARUCO_CUBE, OPTICAL_FLOW_GLOBAL, OPTICAL_FLOW_PARTIAL, UNDISTORT};
enum class enum_ImagingCondition { CAM4K_IN_AIR_640P, CAM4K_THROUGH_GLASS_640P, CAM4K_UNDERWATER_640P, CAM170_IN_AIR_640P, CAM170_SEALED_IN_AIR_640P, CAM170_SEALED_UNDERWATER_1024P, CAM170_SEALED_UNDERWATER_640P};

//������Ƶ��⵽��aruco��Ŀ����detectArUco�н��������
//histScale=12, kernel=3, ��⵽1117��
//histScale=12, kernel=1, ��⵽1023��
//histScale=8, kernel=1, ��⵽971��
//histScale=8, kernel=3, ��⵽1000��

//histScale=12, kernel=3+��, ��⵽800���
//histScale=12, kernel=5, ��⵽1151��
//histScale=12, kernel=7, ��⵽1127��
//histScale=12, kernel=7+��, ��⵽1047��
//histScale=12, kernel=9, ��⵽948��
//histScale=12, kernel=9+��, ��⵽889��

//histScale=10, kernel=5, ��⵽1110��
//histScale=12, kernel=5, ��⵽1151��
//histScale=14, kernel=5, ��⵽1173��
//histScale=16, kernel=5, ��⵽1197��
//histScale=16, kernel=7, ��⵽1145��

//histScale=12, kernel=1, ��⵽1023��
//histScale=12, kernel=5, ��⵽1151��
//histScale=12, ˫٤��2.0+kernel=1, ��⵽1073��
//histScale=12, ˫٤��2.0+kernel=5, ��⵽1110��
//histScale=12, ˫٤��1.5+kernel=1, ��⵽1120��
//histScale=12, ˫٤��1.25+kernel=1, ��⵽1133����������4������
//histScale=12, ˫٤��1.25+kernel=5, ��⵽1237��������������
//histScale=12, ˫٤��1.10+kernel=1, ��⵽1085����������3������

//histScale=16, ˫٤��1.25+kernel=1, ��⵽1073����������
//histScale=16, ˫٤��1.25+kernel=5, ��⵽1241��
static int arucoNum = 0;

class MyImagePreprocess
{
private:
	//Mat* framePtr;
	//Mat framePreprocessed;
	enum_PreprocessorType preprocessType;
	enum_HistEqualType histEqualType;
	enum_FilterType filterType;
	MyExceptionHandle exceptionHandler;
	cv::Mat kernel;
	double histScale;

	void inverseColor(cv::Mat& src, cv::Mat& dst);
	void applyFilter(cv::Mat& src, cv::Mat& dst);
	void histEqualDehaze(cv::Mat& src, cv::Mat& dst);
	void histMatch(cv::Mat& src, cv::Mat& dst);
	//int darkChannelDehaze(Mat& src, Mat& dst);

protected:

public:
	MyImagePreprocess();
	~MyImagePreprocess();
	void setParams(enum_PreprocessorType preprocessType = enum_PreprocessorType::NONE,
		enum_HistEqualType histEqualType = enum_HistEqualType::SINGLE_CHANNEL_GLOBAL,
		enum_FilterType filterType = enum_FilterType::NONE_FILTER);
	void setKernel(int kernelSize, cv::MorphShapes morphShape = cv::MORPH_RECT);
	//void setKernel(Mat kernel);
	void setHistScale(double histScale);
	int Preprocess(cv::Mat& src, cv::Mat& dst);
};


enum class enum_ObjType { ARUCO_BOARD, ARUCO_CUBE };
class MyPoseEstimator
{
private:
	enum_ObjType objType;
	cv::Vec3d estRvec;
	cv::Vec3d estTvec;
	cv::Vec3d EstimateCubeCenter(cv::Vec3d rvec, cv::Vec3d tvec);
public:
	MyPoseEstimator();
	MyPoseEstimator(enum_ObjType objType);
	~MyPoseEstimator();
	void EstimateObjectPose(std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs);
	void getEstimatedPose(cv::Vec3d& rvec, cv::Vec3d& tvec);
};


class MyImageProcess
{
private:
	enum_ProcessorType processType;
	enum_ImagingCondition imagingCondition;
	//MyImagePreprocess imgPreprocessor;
	MyPoseEstimator poseEstimator;

	cv::Ptr<cv::aruco::Dictionary> dictionary;
	cv::Ptr<cv::aruco::Board> arucoBoard;
	MyExceptionHandle exceptionHandler;
	cv::Ptr<cv::aruco::Board> GetCenterBoard();
	int DetectAruco(cv::Mat& src);
	void AssignCameraParams();
	void Init();

protected:

public:
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<int> arucoIds;
	std::vector<std::vector<cv::Point2f>> arucoCorners;
	cv::Vec3d objRvec;
	cv::Vec3d objTvec;
	std::vector<cv::Vec3d> arucoRvecs;
	std::vector<cv::Vec3d> arucoTvecs;
	MyImageProcess();
	~MyImageProcess();
	void setParams(enum_ProcessorType processType, enum_ImagingCondition imagingCondition);
	int Process(cv::Mat& src, cv::Mat& dst, cv::Mat& drawImg);
	int drawArucoSingle(cv::Mat& src);
	int drawArucoBoard(cv::Mat& src);
};


#endif