#include "MyExperiment.h"

using namespace std;
using namespace cv;


int locateErrorExp()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(30.0);
	MyImagePreprocess imgPreprocessor = MyImagePreprocess();
	imgPreprocessor.setParams(enum_PreprocessorType::NONE);
	MyImageProcess imgProcessor = MyImageProcess();
	imgProcessor.setParams(enum_ProcessorType::ARUCO_SINGLE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, drawImg, srcImgGray, srcImgHSV,  prepImg, dstImg, dstImgHSV, dstImgRGB;
	Mat srcRoi, srcRoiHSV, dstRoiHSV, dstRoiRGB;
	Scalar limL = Scalar(75, 80, 80);
	Scalar limH = Scalar(150, 255, 255);
	Mat mask, Mask, roi;
	vector<vector<Point>> contour, maskcontour;
	vector<Point> corners;
	clock_t timeTag;
	while (1) {
		srcImg = streamObj.getFrame();
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		//bitwise_not(srcImg, srcImgHSV);
		cvtColor(srcImg, srcImgHSV, COLOR_RGB2HSV);
		imgPreprocessor.Preprocess(srcImgGray, prepImg);
		srcImg.copyTo(drawImg);
		imgProcessor.Process(prepImg, dstImg, drawImg);
		imshow("Captured", drawImg);
		//inRange(srcImgHSV, limL, limH, mask);
		//srcImgHSV.copyTo(dstImgHSV, mask);
		//cvtColor(dstImgHSV, dstImgRGB, COLOR_HSV2RGB);
		//imshow("Center", dstImgRGB);
		if (imgProcessor.arucoIds.size() == 0) {
			waitKey(1);
			continue;
		}
		corners.clear();
		Point2f detectedCenter = Point2f(0.0, 0.0);
		Point2f markedCenter = Point2f(0.0, 0.0);
		for (int i = 0; i < 4; i++) {
			detectedCenter.x += imgProcessor.arucoCorners[0][i].x;
			detectedCenter.y += imgProcessor.arucoCorners[0][i].y;
		}
		detectedCenter.x /= 4;
		detectedCenter.y /= 4;
		for (int i = 0; i < 4; i++) {
			corners.push_back(Point2f(0.8 * imgProcessor.arucoCorners[0][i].x + 0.2 * detectedCenter.x, 0.8 * imgProcessor.arucoCorners[0][i].y + 0.2 * detectedCenter.y));
		}
		contour.clear();
		contour.push_back(corners);
		Mat roi = Mat::zeros(srcImg.size(), CV_8U);
		mask = Mat::zeros(srcImg.size(), CV_8U);
		srcRoi = Mat::zeros(srcImg.size(), CV_8UC3);
		srcRoiHSV = Mat::zeros(srcImg.size(), CV_8UC3);
		dstRoiRGB = Mat::zeros(srcImg.size(), CV_8UC3);
		drawContours(roi, contour, 0, Scalar::all(255), -1);
		srcImg.copyTo(srcRoi, roi);
		cvtColor(srcRoi, srcRoiHSV, COLOR_RGB2HSV);
		inRange(srcRoiHSV, limL, limH, mask);
		srcRoi.copyTo(dstRoiRGB, mask);
		maskcontour.clear();
		findContours(mask, maskcontour, RETR_EXTERNAL, CHAIN_APPROX_NONE);
		imshow("ROI", srcRoi);
		imshow("Center", dstRoiRGB);
		if (maskcontour.size() > 0 && maskcontour[0].size() <= 30){
			for (int i = 0; i < maskcontour[0].size(); i++) {
				markedCenter.x += maskcontour[0][i].x;
				markedCenter.y += maskcontour[0][i].y;
			}
			markedCenter.x /= maskcontour[0].size();
			markedCenter.y /= maskcontour[0].size();
		}
		//if (maskcontour.size() > 0)
		//	cout << "Contour Length: " << maskcontour[0].size() << endl;
		timeTag = clock();
		cout << "Detected Position (px): " << detectedCenter.x << ", " << detectedCenter.y << endl;
		cout << "Marked Position (px): " << markedCenter.x << ", " << markedCenter.y << endl;
		cout << "Time (ms): " << timeTag << endl;
		waitKey(1);
	}
	return 0;
}

int detectDelayExp()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(60.0);
	MyImagePreprocess imgPreprocessor = MyImagePreprocess();
	imgPreprocessor.setParams(enum_PreprocessorType::NONE);
	MyImageProcess imgProcessor = MyImageProcess();
	imgProcessor.setParams(enum_ProcessorType::ARUCO_SINGLE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, srcImgGray, prepImg, dstImg;
	clock_t startTime, endTime;
	while (1) {
		startTime = clock();
		srcImg = streamObj.getFrame();
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		imgPreprocessor.Preprocess(srcImgGray, prepImg);
		imgProcessor.Process(prepImg, dstImg, srcImg);
		imshow("Detected", srcImg);
		endTime = clock();
		cout << endTime - startTime << "ms" << endl;
		waitKey(1);
	}
	return 0;
}