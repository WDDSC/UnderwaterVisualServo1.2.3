#include "MyImageProcess.h"

using namespace std;
using namespace cv;

MyImagePreprocess::MyImagePreprocess()
{
	this->preprocessType = enum_PreprocessorType::NONE;
	this->histEqualType = enum_HistEqualType::SINGLE_CHANNEL_GLOBAL;
	this->filterType = enum_FilterType::NONE_FILTER;
	this->exceptionHandler = MyExceptionHandle();
	this->kernel = getStructuringElement(MORPH_RECT, Size(3,3));
	this->histScale = 12.0;
}
MyImagePreprocess::~MyImagePreprocess()
{}
void MyImagePreprocess::setParams(enum_PreprocessorType preprocessType, enum_HistEqualType histEqualType, enum_FilterType filterType)
{
	this->preprocessType = preprocessType;
	this->histEqualType = histEqualType;
	this->filterType = filterType;
}
void MyImagePreprocess::setKernel(int kernelSize, MorphShapes morphShape)
{
	this->kernel = getStructuringElement(morphShape, Size(kernelSize, kernelSize));
}
void MyImagePreprocess::setHistScale(double HistScale)
{
	this->histScale = HistScale;
}
void MyImagePreprocess::inverseColor(Mat& src, Mat& dst)
{
	cv::Mat lookTable(1, 256, CV_8U);
	uchar* pdata = lookTable.data;
	for (int i = 0; i < 256; i++)
		pdata[i] = 255 - i;
	cv::LUT(src, lookTable, dst);
}
void MyImagePreprocess::applyFilter(Mat& src, Mat& dst)
{
	float kernelVec[3][3] = { 0,-1,0,-1,5,-1,0,-1,0 };
	float k = 0.5;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			kernelVec[i][j] *= k;
	kernelVec[1][1] += 1 - k;
	Mat sharpenKernel = Mat(Size(3, 3), CV_32FC1, kernelVec);
	switch (filterType) {
	case enum_FilterType::NONE_FILTER:
		src.copyTo(dst);
		break;
	case enum_FilterType::MEAN_BLUR:
		blur(src, dst, kernel.size());
		break;
	case enum_FilterType::MEDIAN_BLUR:
		medianBlur(src, dst, kernel.size().height);
		break;
	case enum_FilterType::GUASSIAN_BLUR:
		GaussianBlur(src, dst, kernel.size(), 0);
		break;
	case enum_FilterType::SHARPEN_FILTER:
		//vector<vector<int>> kernelVec = { {0,0,1} };
		filter2D(src, dst, src.depth(), sharpenKernel);
		break;
	default:
		exceptionHandler.ReportWarning(WARN_TYPE::FUNCTION_UNCOMPLETE_WARN, "MyImagePreprocess::applyFilter");
		src.copyTo(dst);
		break;
	}
}
void MyImagePreprocess::histEqualDehaze(Mat& src, Mat& dst)
{
	vector<Mat> channels;
	split(src, channels);
	Mat blue, green, red;
	Ptr<CLAHE> clahe = createCLAHE();
	switch (histEqualType) {
	case(enum_HistEqualType::SINGLE_CHANNEL_GLOBAL):
		if (channels.size() != 1)
			exceptionHandler.ReportWarning(WARN_TYPE::WRONG_PARAM_TYPE_WARN, "MyImagePreprocess::histEqualDehaze");
		equalizeHist(channels[0], dst);
		break;
	case(enum_HistEqualType::MULTI_CHANNEL_GLOBAL):
		if (channels.size() != 3) {
			exceptionHandler.ReportError(ERR_TYPE::IMG_PREPROCESS_ERR, "MyImagePreprocess::histEqualDehaze");
			throw ERR_TYPE::IMG_PREPROCESS_ERR;
		}
		blue = channels.at(0);
		green = channels.at(1);
		red = channels.at(2);
		equalizeHist(red, red);
		equalizeHist(green, green);
		equalizeHist(blue, blue);
		merge(channels, dst);
		break;
	case(enum_HistEqualType::SINGLE_CHANNEL_LOCAL):
		if (channels.size() != 1)
			exceptionHandler.ReportWarning(WARN_TYPE::WRONG_PARAM_TYPE_WARN, "MyImagePreprocess::histEqualDehaze");
		clahe->setClipLimit(histScale);    // (int)(4.*(8*8)/256)
		clahe->setTilesGridSize(Size(10, 10)); // 将图像分为8*8块
		clahe->apply(channels[0], dst);
		break;
	case(enum_HistEqualType::MULTI_CHANNEL_LOCAL):
		if (channels.size() != 3) {
			exceptionHandler.ReportError(ERR_TYPE::IMG_PREPROCESS_ERR, "MyImagePreprocess::histEqualDehaze");
			throw ERR_TYPE::IMG_PREPROCESS_ERR;
		}
		blue = channels.at(0);
		green = channels.at(1);
		red = channels.at(2);
		clahe->setClipLimit(histScale);    // (int)(4.*(8*8)/256)
		clahe->setTilesGridSize(Size(10, 10)); // 将图像分为8*8块
		clahe->apply(red, red);
		clahe->apply(green, green);
		clahe->apply(blue, blue);
		merge(channels, dst);
		break;
	}
}
//待完善
void MyImagePreprocess::histMatch(Mat& src, Mat& dst)
{
	if (src.channels() != 1) {
		exceptionHandler.ReportError(ERR_TYPE::IMG_PREPROCESS_ERR, "MyImagePreprocess::histMatch");
		throw ERR_TYPE::IMG_PREPROCESS_ERR;
	}
	Scalar meanChannel = mean(src);
	int meanGray = floor(meanChannel[0]);
	src.copySize(dst);
	double srcVal;
	double dstVal;
	double dualGammaParam = 1.25;
	for(int i=0; i<src.size().height; i++)
		for (int j = 0; j < src.size().width; j++) {
			srcVal = (double)src.at<uchar>(i, j);
			if (src.at<uchar>(i, j) <= meanGray) {
				dstVal = pow(srcVal / (double)meanGray, dualGammaParam) * (double)meanGray;
				dst.at<uchar>(i, j) = floor(dstVal);
			}
			else {
				dstVal = pow((srcVal - (double)meanGray) / (255.0 - (double)meanGray), 1.0 / dualGammaParam) * (255.0 - (double)meanGray) + (double)meanGray;
				dst.at<uchar>(i, j) = floor(dstVal);
			}
		}
}
//待完善
int MyImagePreprocess::Preprocess(Mat& src, Mat& dst)
{
	switch (preprocessType) {
	case(enum_PreprocessorType::CVT_GRAY):
		cvtColor(src, dst, COLOR_RGB2GRAY);
		break;
	case(enum_PreprocessorType::NONE):
		src.copyTo(dst);
		break;
	case(enum_PreprocessorType::INVERSE_COLOR):
		inverseColor(src, dst);
		break;
	case(enum_PreprocessorType::HIST_EQUAL):
		histEqualDehaze(src, dst);
		break;
	case(enum_PreprocessorType::FILTER):
		applyFilter(src, dst);
		break;
	case(enum_PreprocessorType::HIST_MATCH):
		histMatch(src, dst);
		break;
	default:
		exceptionHandler.ReportWarning(WARN_TYPE::FUNCTION_UNCOMPLETE_WARN, "MyImagePreprocess::preprocess");
		src.copyTo(dst);
		break;
	}
	return 0;
}


MyImageProcess::MyImageProcess()
{
	this->processType = enum_ProcessorType::NONE_PROCESS;
	this->imagingCondition = enum_ImagingCondition::CAM170_IN_AIR_640P;
	//this->imgPreprocessor = MyImagePreprocess();
	this->exceptionHandler = MyExceptionHandle();
	this->poseEstimator = MyPoseEstimator();
}
MyImageProcess::~MyImageProcess()
{}
void MyImageProcess::setParams(enum_ProcessorType processType, enum_ImagingCondition imagingCondition)
{
	this->processType = processType;
	this->imagingCondition = imagingCondition;
	Init();
}
Ptr<aruco::Board> MyImageProcess::GetCenterBoard()
{
	//生成gridBoard
	int markersX = 3;//X轴上标记的数量
	int markersY = 3;//Y轴上标记的数量
	float markerLength = 0.04;//标记的长度，单位是m
	float markerSeparation = 0.02;//每个标记之间的间隔，单位m
	Ptr<aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	vector<int> myIds = { 0,1,2,3,4,5,6,7,8 };//gridboard的码总数如果和myIds对不上就会不报错直接崩溃
	int borderBits = 1;//标记的边界所占的bit位数
	cv::Ptr<cv::aruco::GridBoard> my_grid_board;
	my_grid_board = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
	my_grid_board->setIds(myIds);
	//转换成中心Board
	cv::Size my_grid_size = my_grid_board->getGridSize();
	cout << my_grid_size << endl;
	vector<vector<Point3f>> my_obj_points = my_grid_board->objPoints;
	float board_x_center = float(my_grid_board->getMarkerLength() * my_grid_size.width) / 2.0\
		+ my_grid_board->getMarkerSeparation() * (float(my_grid_size.width - 1) / 2.0);
	float board_y_center = float(my_grid_board->getMarkerLength() * my_grid_size.height) / 2.0\
		+ my_grid_board->getMarkerSeparation() * (float(my_grid_size.height - 1) / 2.0);
	for (int i = 0; i < my_obj_points.size(); i++) {
		for (int j = 0; j < my_obj_points[0].size(); j++) {
			my_obj_points[i][j].x -= board_x_center;
			my_obj_points[i][j].y -= board_y_center;
		}
	}
	Ptr<aruco::Board> myBoard;
	myBoard = aruco::Board::create(my_obj_points, dictionary, my_grid_board->ids);
	return myBoard;
}
void MyImageProcess::AssignCameraParams()
{
	float fx, fy, cx, cy, k1, k2, k3, p1, p2;
	switch (imagingCondition) {
	case(enum_ImagingCondition::CAM4K_IN_AIR_640P):
		fx = 437.9469;
		fy = 437.8578;
		cx = 311.0968;
		cy = 251.3721;
		k1 = 0.0558;
		k2 = -0.0774;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	case(enum_ImagingCondition::CAM4K_THROUGH_GLASS_640P): //直接使用了空气中的参数
		fx = 436.3338;
		fy = 439.1468;
		cx = 308.5593;
		cy = 252.0916;
		k1 = 0.059;
		k2 = -0.0689;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	case(enum_ImagingCondition::CAM4K_UNDERWATER_640P):
		fx = 590.1614;
		fy = 594.2582;
		cx = 310.6945;
		cy = 255.3156;
		k1 = 0.5088;
		k2 = -0.065;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	case(enum_ImagingCondition::CAM170_IN_AIR_640P):
		fx = 293.2017;
		fy = 294.5789;
		cx = 315.5418;
		cy = 239.7708;
		k1 = -0.2358;
		k2 = 0.0345;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	//密封后在空气中的参数并未标定，直接使用未密封的参数
	case(enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P):
		fx = 293.2017;
		fy = 294.5789;
		cx = 315.5418;
		cy = 239.7708;
		k1 = -0.2358;
		k2 = 0.0345;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	case(enum_ImagingCondition::CAM170_SEALED_UNDERWATER_1024P):
		fx = 584.0546;
		fy = 580.7337;
		cx = 490.1054;
		cy = 394.5607;
		k1 = -0.2193;
		k2 = 0.1219;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	//推算得到的，有机会重新标定
	case(enum_ImagingCondition::CAM170_SEALED_UNDERWATER_640P):
		//fx = 584.0546 / 1024.0 * 640.0;
		//fy = 580.7337 / 1024.0 * 640.0;
		//cx = 490.1054 / 1024.0 * 640.0;
		//cy = 394.5607 / 1024.0 * 640.0;
		//k1 = -0.2193;
		//k2 = 0.1219;
		//k3 = 0;
		//p1 = 0;
		//p2 = 0;
		fx = 365.15;
		fy = 363.64;
		cx = 318.04;
		cy = 251.82;
		k1 = -0.2223;
		k2 = 0.1138;
		k3 = 0;
		p1 = 0;
		p2 = 0;
		break;
	}
	cameraMatrix = (cv::Mat_<float>(3, 3) <<
		fx, 0.0, cx,
		0.0, fy, cy,
		0.0, 0.0, 1.0);
	distCoeffs = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
}
void MyImageProcess::Init()
{
	//imgPreprocessor.setParams(enum_PreprocessorType::HIST_EQUAL);
	AssignCameraParams();
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	//dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
	arucoBoard = GetCenterBoard();
}
//加了无关代码
int MyImageProcess::Process(Mat& src, Mat& dst, Mat& drawImg)
{
	exceptionHandler.ReportWarning(WARN_TYPE::FUNCTION_UNCOMPLETE_WARN, "MyImageProcess::Process");
	vector<Vec3d> rvecs, tvecs;
	switch (processType) {
	case(enum_ProcessorType::NONE_PROCESS):
		src.copyTo(dst);
		break;
	case(enum_ProcessorType::ARUCO_SINGLE):
		arucoNum += DetectAruco(src);
		cout << arucoNum << endl;
		drawArucoSingle(drawImg);
		src.copyTo(dst);
		break;
	case(enum_ProcessorType::ARUCO_BOARD):
		poseEstimator = MyPoseEstimator(enum_ObjType::ARUCO_BOARD);
		arucoNum += DetectAruco(src);
		cout << arucoNum << endl;
		drawArucoBoard(drawImg);
		src.copyTo(dst);
		break;
	case(enum_ProcessorType::ARUCO_CUBE):
		poseEstimator = MyPoseEstimator(enum_ObjType::ARUCO_CUBE);
		arucoNum += DetectAruco(src);
		cout << arucoNum << endl;
		src.copyTo(dst);
		aruco::drawDetectedMarkers(dst, arucoCorners, arucoIds);
		aruco::estimatePoseSingleMarkers(arucoCorners, 0.03, cameraMatrix, distCoeffs, rvecs, tvecs);
		poseEstimator.EstimateObjectPose(rvecs, tvecs);
		poseEstimator.getEstimatedPose(objRvec, objTvec);
		if (objTvec[2] != 0)
			aruco::drawAxis(drawImg, cameraMatrix, distCoeffs, objRvec, objTvec, 0.04);
		break;
	case(enum_ProcessorType::UNDISTORT):
		//initUndistortRectifyMap(cameraMatrix, distCoeffs, noArray);
		undistort(src, dst, cameraMatrix, distCoeffs);
	}
	return 0;
}
int MyImageProcess::DetectAruco(Mat& src)
{
	vector<vector<cv::Point2f>> rej_points;
	aruco::detectMarkers(src, dictionary, arucoCorners, arucoIds, \
		aruco::DetectorParameters::create(), noArray(), cameraMatrix, distCoeffs);//检测该帧是否有标记
	return arucoIds.size();
}
//关闭了防反射
//以后需要将位姿提取部分从draw中抽离
int MyImageProcess::drawArucoSingle(Mat& src)
{
	if (arucoIds.empty()) {
		exceptionHandler.ReportWarning(WARN_TYPE::WRONG_PARAM_TYPE_WARN, "MyImageProcess::drawArucoSingle");
		return 0;
	}
	aruco::drawDetectedMarkers(src, arucoCorners, arucoIds);
	vector<Vec3d> rvec, tvec;
	aruco::estimatePoseSingleMarkers(arucoCorners, 0.04, cameraMatrix, distCoeffs, rvec, tvec);
	for (int i = 0; i < arucoIds.size(); i++) {
		aruco::drawAxis(src, cameraMatrix, distCoeffs, rvec[i], tvec[i], 0.04);
	}
	//EstimateMedianPose(rvec, tvec);
	objRvec = rvec[0];
	objTvec = tvec[0];
	cout << arucoIds.size() << " markers detected!" << endl;
	cout << objRvec[0] << " " << objRvec[1] << " " << objRvec[2] << endl;
	cout << objTvec[0] << " " << objTvec[1] << " " << objTvec[2] << endl;
	return 0;
}
//关闭了防反射
int MyImageProcess::drawArucoBoard(Mat& src)
{
	if (arucoIds.empty()) {
		exceptionHandler.ReportWarning(WARN_TYPE::WRONG_PARAM_TYPE_WARN, "MyImageProcess::drawArucoSingle");
		return 0;
	}
	cv::Vec3d rvec, tvec;
	aruco::estimatePoseBoard(arucoCorners, arucoIds, arucoBoard, cameraMatrix, distCoeffs, rvec, tvec, false);
	aruco::drawDetectedMarkers(src, arucoCorners, arucoIds);
	aruco::drawAxis(src, cameraMatrix, distCoeffs, rvec, tvec, 0.08);
	objRvec = rvec;
	objTvec = tvec;
	return 0;
}

/*位姿估计MyPoseEstimator类*/
MyPoseEstimator::MyPoseEstimator()
{
	*this = MyPoseEstimator(enum_ObjType::ARUCO_CUBE);
}
MyPoseEstimator::MyPoseEstimator(enum_ObjType objType)
{
	this->objType = objType;
	this->estRvec[0] = 0;
	this->estRvec[1] = 0;
	this->estRvec[2] = 0;
	this->estTvec[0] = 0;
	this->estTvec[1] = 0;
	this->estTvec[2] = 0;
}
MyPoseEstimator::~MyPoseEstimator() {}
void MyPoseEstimator::EstimateObjectPose(std::vector<cv::Vec3d> rvecs, std::vector<cv::Vec3d> tvecs)
{
	if (rvecs.size() == 0) {
		estTvec[0] = 0;
		estTvec[1] = 0;
		estTvec[2] = 0;
		estRvec[0] = 0;
		estRvec[1] = 0;
		estRvec[2] = 0;
		return;
	}
	switch (objType) {
	case enum_ObjType::ARUCO_BOARD:
		break;
	case enum_ObjType::ARUCO_CUBE:
		for (int i = 0; i < tvecs.size(); i++) {
			estTvec += EstimateCubeCenter(rvecs[i], tvecs[i]);
		}
		estTvec[0] /= tvecs.size();
		estTvec[1] /= tvecs.size();
		estTvec[2] /= tvecs.size();
		estRvec[0] = 0;
		estRvec[1] = 0;
		estRvec[2] = 0;
		break;
	}
}
Vec3d MyPoseEstimator::EstimateCubeCenter(Vec3d rvec, Vec3d tvec)
{
	PoseMatrix rotMat = RotMat({ rvec[0], rvec[1], rvec[2] });
	Vec3d avec, cubeCenterTvec;
	avec[0] = rotMat.elements[0][2];
	avec[1] = rotMat.elements[1][2];
	avec[2] = rotMat.elements[2][2];
	return tvec - 0.02 * avec;
}
void MyPoseEstimator::getEstimatedPose(cv::Vec3d& rvec, cv::Vec3d& tvec)
{
	rvec = estRvec;
	tvec = estTvec;
}