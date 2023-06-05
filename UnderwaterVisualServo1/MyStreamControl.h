#ifndef MYSTREAMCONTROL_H
#define MYSTREAMCONTROL_H

#pragma warning(disable:4996)

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "MyExceptionHandle.h"
#include "MyImageProcess.h"
//#include "MyTcpip.h"

//using namespace std;
//using namespace cv;

//enum ERR_TYPE{STREAM_CAP_ERR, IMG_PREPROCESS_ERR, IMG_PROCESS_ERR, DATA_TRANS_ERR, DATA_SAVE_ERR, UNKNOWN_ERR};
enum class enum_StreamType {NONE_STREAM, CAMERA, VIDEO, IMAGE};

class MyStreamControl
{
private:
	cv::VideoCapture cap;
	enum_StreamType streamType;
	std::string videoAddr;
	std::string imageAddr;
	int deviceIpAddr;
	bool logOn;
	bool tcpOn;
	bool videoSaveOn;
	//VideoWriter videoWriter;
	std::string logPath;
	std::string savePath;
	cv::Mat srcImg;
	cv::Mat preprocessedImg;
	cv::Mat processedImg;
	MyImagePreprocess imgPreprocessor;
	MyImageProcess imgProcessor;
	MyExceptionHandle exceptionHandler;
	tm* ltm;

	int init();
	void stop();
	//void reportErr(ERR_TYPE errType);
	int saveData(std::vector<float> dataFloat);
	int saveVideoFrame(cv::VideoWriter& videoWriter);
	//void initArUcoBoard();
	std::string getDate();
	std::string getTime();

public:
	MyStreamControl();
	~MyStreamControl();
	void setInitParams(enum_StreamType streamType, std::string address, std::string logPath = "", std::string savePath = "");
	int setResolution(int width, int height);
	int setFps(double fps);
	cv::Mat getFrame();
	void beginPreview();
	void beginDetect();
	void beginDemo();
};

#endif