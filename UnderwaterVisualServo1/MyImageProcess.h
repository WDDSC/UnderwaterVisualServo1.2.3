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

//整个视频检测到的aruco数目，在detectArUco中进行了输出
//histScale=12, kernel=3, 检测到1117个
//histScale=12, kernel=1, 检测到1023个
//histScale=8, kernel=1, 检测到971个
//histScale=8, kernel=3, 检测到1000个

//histScale=12, kernel=3+锐化, 检测到800多个
//histScale=12, kernel=5, 检测到1151个
//histScale=12, kernel=7, 检测到1127个
//histScale=12, kernel=7+锐化, 检测到1047个
//histScale=12, kernel=9, 检测到948个
//histScale=12, kernel=9+锐化, 检测到889个

//histScale=10, kernel=5, 检测到1110个
//histScale=12, kernel=5, 检测到1151个
//histScale=14, kernel=5, 检测到1173个
//histScale=16, kernel=5, 检测到1197个
//histScale=16, kernel=7, 检测到1145个

//histScale=12, kernel=1, 检测到1023个
//histScale=12, kernel=5, 检测到1151个
//histScale=12, 双伽马2.0+kernel=1, 检测到1073个
//histScale=12, 双伽马2.0+kernel=5, 检测到1110个
//histScale=12, 双伽马1.5+kernel=1, 检测到1120个
//histScale=12, 双伽马1.25+kernel=1, 检测到1133个，误判有4次左右
//histScale=12, 双伽马1.25+kernel=5, 检测到1237个，几乎无误判
//histScale=12, 双伽马1.10+kernel=1, 检测到1085个，误判有3次左右

//histScale=16, 双伽马1.25+kernel=1, 检测到1073个，有误判
//histScale=16, 双伽马1.25+kernel=5, 检测到1241个
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