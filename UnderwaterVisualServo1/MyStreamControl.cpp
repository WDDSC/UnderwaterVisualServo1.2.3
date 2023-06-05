#include "MyStreamControl.h"

using namespace std;
using namespace cv;

//����
MyStreamControl::MyStreamControl()
{
	streamType = enum_StreamType::NONE_STREAM;
	videoAddr = "";
	imageAddr = "";
	deviceIpAddr = -1;
	imgPreprocessor = MyImagePreprocess();
	imgProcessor = MyImageProcess();
	exceptionHandler = MyExceptionHandle();
}
//������
MyStreamControl::~MyStreamControl()
{
	cap.release();
}
//����
void MyStreamControl::stop()
{
	this->~MyStreamControl();
}
//����
int MyStreamControl::saveData(vector<float> dataFloat)
{
	ofstream fout(logPath.data());
	for (int i = 0; i < dataFloat.size(); i++) {
		fout << dataFloat[i] << " ";
	}
	fout << "\n";
	return 0;
}
//����
int MyStreamControl::saveVideoFrame(VideoWriter& videoWriter)
{
	videoWriter.write(processedImg);
	return 0;
}
//����
string MyStreamControl::getDate()
{
	time_t now = time(0);
	ltm = localtime(&now);
	return to_string(ltm->tm_year+1900) + "-" + to_string(ltm->tm_mon) + "-" + to_string(ltm->tm_mday);
	//return to_string(ltm->tm_year + 1900) + to_string(ltm->tm_mon) + to_string(ltm->tm_mday);
}
//����
string MyStreamControl::getTime()
{
	time_t now = time(0);
	ltm = localtime(&now);
	return to_string(ltm->tm_hour) + "-" + to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);
}
//����
void MyStreamControl::setInitParams(enum_StreamType streamType, string address, string logPath, string savePath)
{
	this->streamType = streamType;
	this->logPath = logPath;
	this->savePath = savePath;
	switch (streamType) {
	case(enum_StreamType::CAMERA):
		this->deviceIpAddr = atoi(address.data());
		break;
	case(enum_StreamType::VIDEO):
		this->videoAddr = address;
		break;
	case(enum_StreamType::IMAGE):
		this->imageAddr = address;
		break;
	default:
		throw ERR_TYPE::UNKNOWN_ERR;
		break;
	}
	init();
}
//����
int MyStreamControl::init()
{
	if (logPath == "") {
		logPath = "..\\WorkSpace\\log.txt";
		exceptionHandler.ReportWarning("Default log path is applied (" + logPath + ")!");
	}
	if (savePath == "") {
		savePath = "..\\WorkSpace\\";
		exceptionHandler.ReportWarning("Default log path is applied (" + savePath + ")!");
	}
	switch (streamType) {
	case(enum_StreamType::CAMERA):
		cap.open(deviceIpAddr, CAP_DSHOW);
		break;
	case(enum_StreamType::VIDEO):
		cap.open(videoAddr);
		break;
	case(enum_StreamType::IMAGE):
		//srcImg = imread(imageAddr);
		break;
	default:
		throw ERR_TYPE::UNKNOWN_ERR;
		break;
	}
	return 0;
}
//����
int MyStreamControl::setResolution(int width, int height)
{
	cap.set(CAP_PROP_BUFFERSIZE, width*height*10);
	cap.set(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH, width);
	//cap.set(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, height);
	return 0;
}
//����
int MyStreamControl::setFps(double fps)
{
	cap.set(CAP_PROP_FPS, fps);
	return 0;
}
//����
Mat MyStreamControl::getFrame()
{
	//cap.set(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH, 1284);
	//cap.set(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, 963);
	//cap.set(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH, 4);
	//cap.set(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, 3);
	switch (streamType) {
	case(enum_StreamType::CAMERA):
		cap >> srcImg;
		break;
	case(enum_StreamType::VIDEO):
		cap >> srcImg;
		break;
	case(enum_StreamType::IMAGE):
		srcImg = imread(imageAddr);
		break;
	default:
		throw ERR_TYPE::UNKNOWN_ERR;
		break;
	}
	return srcImg;
}
//����
void MyStreamControl::beginPreview()
{
	getFrame();
	int keyVal = -1;
	int imgNum = 0;
	clock_t time = clock();
	while (!srcImg.empty())
	{
		imshow("Stream", srcImg);
		keyVal = waitKey(10);
		//esc�˳�
		if (keyVal == 27) break;
		//����궨ͼ
		else if (keyVal == 's') {
			imgNum++;
			//string fileName = savePath + to_string(imgNum) + ".jpg";
			string fileName = savePath + getDate() + " " + getTime() + ".jpg";
			if (imwrite(fileName, srcImg))
				cout << to_string(imgNum) << "th image has been saved." << endl;
			else
				exceptionHandler.ReportWarning("Failed to save the " + to_string(imgNum) + "th image.");
		}
		getFrame();
		cout << "Spend Time: " << time - clock() << "ms" << endl;
		time = clock();
	}
	stop();
}
//������
void MyStreamControl::beginDetect()
{
	exceptionHandler.ReportWarning(WARN_TYPE::FUNCTION_UNCOMPLETE_WARN, "MyStreamControl::beginDetect");
}
//������
void MyStreamControl::beginDemo()
{
	Mat srcImgGray, prepImg, procImg;
	getFrame();
	//cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
	int imgNum = 0;
	int keyVal = -1;
	VideoWriter dstVideo;
	//Size dstSize = srcImgGray.size();
	Size dstSize = srcImg.size();
	double fps = 30.0;
	//string dstPath = "D:\\ˮ�»�е��\\�Ӿ��ŷ�\\UnderwaterVisualServo1\\WorkSpace\\���Ƕȴ���󣨽�ֱ��ͼ���⣩.mp4";
	//dstVideo = VideoWriter(dstPath, CV_FOURCC('D', 'I', 'V', 'X'), fps, dstSize, true);
	while (!srcImg.empty())
	{
		imshow("Stream", srcImg);

		imgPreprocessor.setParams(enum_PreprocessorType::FILTER,
			enum_HistEqualType::SINGLE_CHANNEL_GLOBAL,
			enum_FilterType::SHARPEN_FILTER);
		imgPreprocessor.Preprocess(srcImg, prepImg);
		imshow("Preprocessed3", prepImg);

		/*ת�Ҷ�*/
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		srcImg.copyTo(prepImg);
		srcImg.copyTo(procImg);
		/*�ֲ�ֱ��ͼ����*/
		imgPreprocessor.setParams(enum_PreprocessorType::HIST_EQUAL,
			enum_HistEqualType::SINGLE_CHANNEL_LOCAL,
			enum_FilterType::NONE_FILTER);
		imgPreprocessor.setHistScale(12);
		imgPreprocessor.Preprocess(srcImgGray, prepImg);
		/*��ɫ*/
		imgPreprocessor.setParams(enum_PreprocessorType::INVERSE_COLOR,
			enum_HistEqualType::SINGLE_CHANNEL_LOCAL,
			enum_FilterType::MEDIAN_BLUR);
		imgPreprocessor.Preprocess(prepImg, prepImg);
		imshow("Preprocessed0", prepImg);
		/*˫٤��任*/
		imgPreprocessor.setParams(enum_PreprocessorType::HIST_MATCH,
			enum_HistEqualType::SINGLE_CHANNEL_LOCAL,
			enum_FilterType::MEDIAN_BLUR);
		imgPreprocessor.Preprocess(prepImg, prepImg);
		imshow("Preprocessed1", prepImg);
		/*��ֵ�˲�*/
		imgPreprocessor.setParams(enum_PreprocessorType::FILTER,
			enum_HistEqualType::SINGLE_CHANNEL_LOCAL,
			enum_FilterType::MEDIAN_BLUR);
		imgPreprocessor.setKernel(5);
		imgPreprocessor.Preprocess(prepImg, prepImg);
		imshow("Preprocessed2", prepImg);
		/*ͼ����*/
		imgPreprocessor.setParams(enum_PreprocessorType::FILTER,
			enum_HistEqualType::SINGLE_CHANNEL_LOCAL,
			enum_FilterType::SHARPEN_FILTER);
		imgPreprocessor.Preprocess(prepImg, prepImg);
		imshow("Preprocessed3", prepImg);
		/*ArUcoʶ��*/
		imgProcessor.setParams(enum_ProcessorType::ARUCO_SINGLE,
			enum_ImagingCondition::CAM4K_UNDERWATER_640P);
		imgProcessor.Process(prepImg, procImg, srcImg);
		//imgProcessor.Process(srcImg, procImg);
		imshow("Processed", srcImg);

		//prepImg.copyTo(preprocessedImg);
		//procImg.copyTo(processedImg);
		srcImg.copyTo(preprocessedImg);
		srcImg.copyTo(processedImg);
		prepImg.copyTo(processedImg);
		//saveVideoFrame(dstVideo);

		keyVal = waitKey(10);
		//esc�˳�
		if (keyVal == 27) break;
		//����궨ͼ
		else if (keyVal == 's') {
			imgNum++;
			//string fileName = savePath + to_string(imgNum) + ".jpg";
			string fileName = savePath + getDate() + " " + getTime() + ".jpg";
			if (imwrite(fileName, prepImg))
				cout << to_string(imgNum) << "th image has been saved." << endl;
			else
				exceptionHandler.ReportWarning("Failed to save the " + to_string(imgNum) + "th image.");
		}
		getFrame();
	}
	stop();
}