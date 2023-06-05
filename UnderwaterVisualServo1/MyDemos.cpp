#include "MyDemos.h"

using namespace std;
using namespace cv;

int demoImageUndistort()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	MyImageProcess imgProcessor = MyImageProcess();
	imgProcessor.setParams(enum_ProcessorType::UNDISTORT, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, dstImg;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		srcImg = streamObj.getFrame();
		imgProcessor.Process(srcImg, dstImg, srcImg);
		endTime = clock();
		imshow("Raw", srcImg);
		imshow("Undistorted", dstImg);
		//cout << endTime - startTime << "ms" << endl;
		waitKey(10);
	}
}
int demoPreview()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	//streamObj.setResolution(960, 720);
	streamObj.setResolution(640, 480);
	streamObj.setFps(30.0);
	streamObj.beginPreview();
	return 0;
}
int demoDetect()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	MyImagePreprocess imgPreprocessor = MyImagePreprocess();
	imgPreprocessor.setParams(enum_PreprocessorType::INVERSE_COLOR);
	MyImageProcess imgProcessor = MyImageProcess();
	imgProcessor.setParams(enum_ProcessorType::ARUCO_BOARD, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, srcImgGray, prepImg, dstImg;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		srcImg = streamObj.getFrame();
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		imgPreprocessor.Preprocess(srcImgGray, prepImg);
		imgProcessor.Process(prepImg, dstImg, srcImg);
		endTime = clock();
		imshow("Detected", srcImg);
		cout << endTime - startTime << "ms" << endl;
		waitKey(10);
	}
}
int demoEstimatePosition()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	MyImagePreprocess imgPreprocessor = MyImagePreprocess();
	imgPreprocessor.setParams(enum_PreprocessorType::INVERSE_COLOR);
	MyImageProcess imgProcessor = MyImageProcess();
	imgProcessor.setParams(enum_ProcessorType::ARUCO_BOARD, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, srcImgGray, prepImg, dstImg;
	double x, y, z;
	x = 0;
	y = 0;
	z = 0;
	string textX, textY, textZ;
	stringstream textStream;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		srcImg = streamObj.getFrame();
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		imgPreprocessor.Preprocess(srcImgGray, prepImg);
		imgProcessor.Process(prepImg, dstImg, srcImg);
		x = 0.9 * x + 0.1 * imgProcessor.objTvec[0];
		y = 0.9 * y + 0.1 * imgProcessor.objTvec[1];
		z = 0.9 * z + 0.1 * imgProcessor.objTvec[2];
		endTime = clock();

		textStream.str("");//不知道为什么用clear()不行
		textStream << x;
		textX = "X = " + textStream.str();
		putText(srcImg, textX, Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		textStream.str("");
		textStream << y;
		textY = "Y = " + textStream.str();
		putText(srcImg, textY, Point(30, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8);
		textStream.str("");
		textStream << z;
		textZ = "Z = " + textStream.str();
		putText(srcImg, textZ, Point(30, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
		imshow("Detected", srcImg);
		cout << endTime - startTime << "ms" << endl;
		waitKey(1);
	}
}
int demoUndistortDetect()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	MyImagePreprocess imgPreprocessor1 = MyImagePreprocess();
	imgPreprocessor1.setParams(enum_PreprocessorType::CVT_GRAY);
	MyImagePreprocess imgPreprocessor2 = MyImagePreprocess();
	imgPreprocessor2.setParams(enum_PreprocessorType::INVERSE_COLOR);
	MyImageProcess imgProcessor1 = MyImageProcess();
	imgProcessor1.setParams(enum_ProcessorType::UNDISTORT, enum_ImagingCondition::CAM170_IN_AIR_640P);
	MyImageProcess imgProcessor2 = MyImageProcess();
	imgProcessor2.setParams(enum_ProcessorType::ARUCO_BOARD, enum_ImagingCondition::CAM170_IN_AIR_640P);
	Mat srcImg, srcImgGray, prepImg, procImg, dstImg, undistImg;
	Mat newCameraMatrix, newDistCoeffs;

	srcImg = streamObj.getFrame();
	undistort(srcImg, dstImg, imgProcessor1.cameraMatrix, imgProcessor1.distCoeffs, imgProcessor1.cameraMatrix);
	cout << newCameraMatrix.rows << " " << newCameraMatrix.cols << endl;

	newDistCoeffs = (cv::Mat_<float>(5, 1) << 0,0,0,0,0);
	//imgProcessor2.cameraMatrix = newCameraMatrix;
	imgProcessor2.distCoeffs = newDistCoeffs;

	double x, y, z;
	x = 0;
	y = 0;
	z = 0;
	stringstream textStream;
	string textX, textY, textZ;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		srcImg = streamObj.getFrame();
		cvtColor(srcImg, srcImgGray, COLOR_RGB2GRAY);
		imgProcessor1.Process(srcImg, undistImg, srcImg);
		imgPreprocessor1.Preprocess(undistImg, srcImgGray);
		imgPreprocessor2.Preprocess(srcImgGray, prepImg);
		imgProcessor2.Process(prepImg, dstImg, undistImg);

		x = 0.9 * x + 0.1 * imgProcessor2.objTvec[0];
		y = 0.9 * y + 0.1 * imgProcessor2.objTvec[1];
		z = 0.9 * z + 0.1 * imgProcessor2.objTvec[2];

		endTime = clock();
		cout << endTime - startTime << "ms" << endl;

		textStream.str("");//不知道为什么用clear()不行
		textStream << x * 100;
		textX = "X = " + textStream.str() + "cm";
		putText(undistImg, textX, Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		textStream.str("");
		textStream << y * 100;
		textY = "Y = " + textStream.str() + "cm";
		putText(undistImg, textY, Point(30, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8);
		textStream.str("");
		textStream << z * 100;
		textZ = "Z = " + textStream.str() + "cm";
		putText(undistImg, textZ, Point(30, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
		imshow("Raw", srcImg);
		imshow("Detect", undistImg);

		waitKey(1);
	}
}
int demoEstimatePose()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);

	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	//VecPreprocessor.push_back(MyImagePreprocess());
	//VecPreprocessor[1].setParams(enum_PreprocessorType::NONE);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_SINGLE, enum_ImagingCondition::CAM170_IN_AIR_640P);

	vector<Mat> VecImg = {Mat(), Mat(), Mat(), Mat()};
	double x, y, z;
	x = 0, y = 0, z = 0;
	stringstream textStream;
	string textX, textY, textZ;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		x = 0.9 * x + 0.1 * VecProcessor[0].objTvec[0];
		y = 0.9 * y + 0.1 * VecProcessor[0].objTvec[1];
		z = 0.9 * z + 0.1 * VecProcessor[0].objTvec[2];

		endTime = clock();
		cout << endTime - startTime << "ms" << endl;

		textStream.str("");//不知道为什么用clear()不行
		textStream << x * 100;
		textX = "X = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textX, Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		textStream.str("");
		textStream << y * 100;
		textY = "Y = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textY, Point(30, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8);
		textStream.str("");
		textStream << z * 100;
		textZ = "Z = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textZ, Point(30, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
		imshow("Raw", VecImg[0]);
		imshow("Step1", VecImg[1]);
		imshow("Step2", VecImg[2]);
		imshow("Detect", VecImg[VecImg.size() - 1]);

		waitKey(1);
	}
}
int demoEstimateCubePose()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);

	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);

	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	double x, y, z;
	x = 0, y = 0, z = 0;
	stringstream textStream;
	string textX, textY, textZ;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		if (VecProcessor[0].objTvec[2] != 0) {
			x = 0.9 * x + 0.1 * VecProcessor[0].objTvec[0];
			y = 0.9 * y + 0.1 * VecProcessor[0].objTvec[1];
			z = 0.9 * z + 0.1 * VecProcessor[0].objTvec[2];
		}

		endTime = clock();
		cout << endTime - startTime << "ms" << endl;

		textStream.str("");//不知道为什么用clear()不行
		textStream << x * 100;
		textX = "X = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textX, Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		textStream.str("");
		textStream << y * 100;
		textY = "Y = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textY, Point(30, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2, 8);
		textStream.str("");
		textStream << z * 100;
		textZ = "Z = " + textStream.str() + "cm";
		putText(VecImg[VecImg.size() - 1], textZ, Point(30, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2, 8);
		imshow("Raw", VecImg[0]);
		imshow("Step1", VecImg[1]);
		imshow("Step2", VecImg[2]);
		imshow("Detect", VecImg[VecImg.size() - 1]);

		waitKey(1);
	}
}
int demoKinematics()
{
	try {
		/*实例化机械臂运动学对象*/
		RoboticKinematics KinematicsObj = RoboticKinematics();
		KinematicsObj.setDefaultRobot();
		//KinematicsObj.setPredefinedRobot(enum_RobotType::HYDRUALIC_PREDFINED);
		PoseMatrix basePoseMat;//基座的位姿矩阵
		basePoseMat.IdentityMat();

		/*第一组关节角度*/
		cout << "================================================================================================" << endl;
		vector<double> theta = { 0,0,0, 0 };//关节角度
		vector<double> endPose;//末端位置Position和姿态Orientation
		PoseMatrix posMat = KinematicsObj.ForwardKinematics(theta, endPose) * TransMat(0,0,0);//计算末端位姿矩阵posMat和位姿向量endPose
		cout << "When joint angles = [" << theta[0] << " " << theta[1] << " " << theta[2] << " " << theta[3] << "]" << endl;//显示关节角度
		cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
		cout << "End orientation (deg): Row(横滚角) = " << endPose[3] << ", Pitch(俯仰角) = " << endPose[4] << ", Yaw(朝向角) = " << endPose[5] << endl; //endPose的后3个为横滚角、俯仰角、朝向角
		posMat.Display();//展示末端位姿矩阵 
		//basePoseMat.drawAxes(100.0);//绘制基座的坐标轴，轴长100mm（暂无绘图的功能）
		//posMat.drawAxes(100.0);//绘制末端的坐标轴，轴长100mm（暂无绘图的功能）
		std::vector<double> endPosition = { endPose[0],endPose[1],endPose[2] };//末端位置
		std::vector<double> jointAngles;//用于存储逆运动学得出的关节角
		posMat = KinematicsObj.InverseKinematics3DofPosition(jointAngles, endPosition);//把正运动学的末端位置结果作为逆运动学的输入，测试逆运动学得到的关节角度是否与正运动学所用到的关节角度一致
		cout << "Inverse kinematics returns: Joint angels = [" << jointAngles[0] << " " << jointAngles[1] << " " << jointAngles[2] << "]" << endl;

		/*第二组关节角度*/
		cout << "================================================================================================" << endl;
		theta = { 5,30,-45, 150 };
		posMat = KinematicsObj.ForwardKinematics(theta, endPose);
		cout << "When joint angles = [" << theta[0] << " " << theta[1] << " " << theta[2] << " " << theta[3] << "]" << endl;//显示关节角度
		cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
		cout << "End orientation (deg): Row = " << endPose[3] << ", Pitch = " << endPose[4] << ", Yaw = " << endPose[5] << endl;//endPose的后3个为横滚角、俯仰角、朝向角
		posMat.Display();//展示末端位姿矩阵
		endPosition = { endPose[0],endPose[1],endPose[2] };
		posMat = KinematicsObj.InverseKinematics3DofPosition(jointAngles, endPosition);//把正运动学的末端位置结果作为逆运动学的输入，测试逆运动学得到的关节角度是否与正运动学所用到的关节角度一致
		(jointAngles, endPosition);
		cout << "Inverse kinematics returns: Joint angels = [" << jointAngles[0] << " " << jointAngles[1] << " " << jointAngles[2] << "]" << endl;

		/*第二组关节角度*/
		cout << "================================================================================================" << endl;
		theta = { 5,30,-45, 150 };
		posMat = KinematicsObj.ForwardKinematics(theta, endPose);
		cout << "When joint angles = [" << theta[0] << " " << theta[1] << " " << theta[2] << " " << theta[3] << "]" << endl;//显示关节角度
		cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
		cout << "End orientation (deg): Row = " << endPose[3] << ", Pitch = " << endPose[4] << ", Yaw = " << endPose[5] << endl;//endPose的后3个为横滚角、俯仰角、朝向角
		posMat.Display();//展示末端位姿矩阵
		endPosition = { endPose[0],endPose[1],endPose[2] };
		posMat = KinematicsObj.InverseKinematics3DofToward(jointAngles, endPosition, 20);
		cout << "Inverse kinematics returns: Joint angels = [" << jointAngles[0] << " " << jointAngles[1] << " " << jointAngles[2] << "]" << endl;

		/*逆运动学求解速度测试*/
		clock_t startTime, endTime;
		double aveTime = 0;
		vector<vector<double>> endPositionVec = {};
		int iternum = 1000;
		for (int i = 0; i < iternum; i++) {
			theta = { 1.0 * (rand() % 100), 1.0 * (rand() % 100), 1.0 * (rand() % 100), 1.0 * (rand() % 100) };
			PoseMatrix posMat = KinematicsObj.ForwardKinematics(theta, endPose);//计算末端位姿矩阵posMat和位姿向量endPose
			endPosition = { endPose[0],endPose[1],endPose[2] };
			endPositionVec.push_back(endPosition);
		}
		startTime = clock();
		for (int i = 0; i < iternum; i++) {
			endPosition = endPositionVec[i];
			posMat = KinematicsObj.InverseKinematics3DofPosition(jointAngles, endPosition, 10);
		}
		endTime = clock();
		aveTime = (endTime - startTime) / (iternum * 1.0);
		cout << "Average cost time: " << aveTime << "ms" << endl;
	}
	catch (...)
	{
		return -1;
	}
	return 0;
}
int demoDifferentialKinematics()
{
	/*实例化机械臂运动学对象*/
	RoboticKinematics KinematicsObj = RoboticKinematics();
	//KinematicsObj.setDefaultRobot();
	KinematicsObj.setPredefinedRobot(enum_RobotType::HYDRUALIC_PREDFINED);
	PoseMatrix basePoseMat;//基座的位姿矩阵
	basePoseMat.IdentityMat();
	
	/*第一组关节角度*/
	cout << "================================================================================================" << endl;
	vector<double> theta = { 0, 0, 0, 0};//关节角度
	vector<double> endPose;//末端位置Position和姿态Orientation
	PoseMatrix posMat = KinematicsObj.ForwardKinematics(theta, endPose);//计算末端位姿矩阵posMat和位姿向量endPose
	cout << "When joint angles = [" << theta[0] << " " << theta[1] << " " << theta[2] << " " << theta[3] << "]" << endl;//显示关节角度
	cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
	cout << "End orientation (deg): Row(横滚角) = " << endPose[3] << ", Pitch(俯仰角) = " << endPose[4] << ", Yaw(朝向角) = " << endPose[5] << endl; //endPose的后3个为横滚角、俯仰角、朝向角
	posMat.Display();//展示末端位姿矩阵
	//basePoseMat.drawAxes(100.0);//绘制基座的坐标轴，轴长100mm（暂无绘图的功能）
	//posMat.drawAxes(100.0);//绘制末端的坐标轴，轴长100mm（暂无绘图的功能）
	cout << "6-DOF Jacobian:" << KinematicsObj.calcJacobianNumerical(theta);
	cout << "3-DOF Jacobian:" << truncate(KinematicsObj.calcJacobianNumerical(theta), { 0,1,2 }, {});
	return 0;
}
int demoUniversalIK()
{
	/*实例化机械臂运动学对象*/
	RoboticKinematics KinematicsObj = RoboticKinematics();
	//KinematicsObj.setDefaultRobot();
	KinematicsObj.setPredefinedRobot(enum_RobotType::HYDRUALIC_PREDFINED);
	PoseMatrix basePoseMat;//基座的位姿矩阵
	basePoseMat.IdentityMat();
	clock_t startTime, endTime;
	double meanTime = 0;

	vector<double> theta;
	vector<double> thetaInit;
	vector<double> IKsolution;
	vector<double> endPose;//末端位置Position和姿态Orientation
	vector<double> endPosition;
	PoseMatrix posMat;
	double err;
	for (int i = 0; i < 100; i++) {
		cout << "================================================================================================" << endl;
		theta = { 1.0*(rand() % 100), 1.0 * (rand() % 100), 1.0 * (rand() % 100), 1.0 * (rand() % 100)};
		thetaInit = theta + vector<double>({ 1.0 * (rand() % 5), 1.0 * (rand() % 5), 1.0 * (rand() % 5), 1.0 * (rand() % 5) });
		posMat = KinematicsObj.ForwardKinematics(theta, endPose);//计算末端位姿矩阵posMat和位姿向量endPose
		//PoseMatrix posMat = KinematicsObj.ForwardKinematics(theta, endPose);//计算末端位姿矩阵posMat和位姿向量endPose
		cout << "When joint angles = [" << theta[0] << " " << theta[1] << " " << theta[2] << " " << theta[3] << "]" << endl;//显示关节角度
		cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
		//cout << "End orientation (deg): Row(横滚角) = " << endPose[3] << ", Pitch(俯仰角) = " << endPose[4] << ", Yaw(朝向角) = " << endPose[5] << endl; //endPose的后3个为横滚角、俯仰角、朝向角
		//posMat.Display();//展示末端位姿矩阵
		endPosition = truncate(endPose, { 0,1,2 });
		startTime = clock();
		err = KinematicsObj.InverseKinematicsGDPosition(IKsolution, endPosition, thetaInit);
		//err = KinematicsObj.InverseKinematicsGNPosition(IKsolution, endPosition, thetaInit);
		endTime = clock();
		posMat = KinematicsObj.ForwardKinematics(IKsolution, endPose);//计算末端位姿矩阵posMat和位姿向量endPose
		cout << "IK solved joint angles = [" << IKsolution[0] << " " << IKsolution[1] << " " << IKsolution[2] << " " << IKsolution[3] << "]" << endl;//显示关节角度
		cout << "End position (mm): X = " << endPose[0] << ", Y = " << endPose[1] << ", Z = " << endPose[2] << endl;//endPose的前3个为xyz坐标
		//posMat.Display();//展示末端位姿矩阵
		cout << "IK error: " << err << "mm" << endl;
		cout << "Cost time: " << endTime - startTime << endl;
		meanTime += endTime - startTime;
	}
	meanTime /= 100.0;
	cout << "================================================================================================" << endl;
	cout << "Average cost time: " << meanTime << "ms" << endl;
	return 0;
}
int demoUnderwaterRecover()
{
	string videoPath = "D:\\水下机械手\\视觉伺服\\实验视频\\视频材料\\低浊度cut.mp4";
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::VIDEO, videoPath);
	streamObj.beginDemo();
	return 0;
}
int demoRealTimeTraj()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);

	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);

	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, {0,0,0});

	double x, y, z;
	x = 0, y = 0, z = 0;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;
	while (1) {
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		if (VecProcessor[0].objTvec[2] != 0) {
			x = 0.95 * x + 0.05 * VecProcessor[0].objTvec[0];
			y = 0.95 * y + 0.05 * VecProcessor[0].objTvec[1];
			z = 0.80 * z + 0.20 * VecProcessor[0].objTvec[2];
		}

		RTPlanner.NewPoint({ x,y,z });
		RTPlanner.getCurrentTraj();
		outText.str("");
		outText << "X = " << x * 100.0 << "cm";
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		outText.str("");
		outText << "Y = " << y * 100.0 << "cm";
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);
		outText.str("");
		outText << "Z = " << z * 100.0 << "cm";
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2, 8);

		cout << x << ", " << y << ", " << z << endl;

		endTime = clock();
		cout << endTime - startTime << "ms" << endl;

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}


	return 0;
}
int demoPointTransTest()
{
	MyPoint2D A_2d = MyPoint2D(2, -2);
	MyPoint2DP A_2dp = Rect2Polar(A_2d);
	MyPoint2D B_2d = MyPoint2D(-2, 0);
	MyPoint2DP B_2dp = Rect2Polar(B_2d);
	cout << B_2dp.rho << ", " << B_2dp.theta << endl;
	cout << Polar2Rect(A_2dp + B_2dp).x << ", " << Polar2Rect(A_2dp + B_2dp).y << endl;
	return 0;
}
int demoTcpServerSend_1()
{
	const char* tcpAddr = "127.0.0.1";
	int port = 5000;
	MyTcpipServer TcpServer = MyTcpipServer(tcpAddr, port);
	TcpServer.buildServer();
	int tag = 1234;
	vector<double> data = { 2000.0, 0.6, 2022.0, 0.828 };
	string info = "wddSC";
	TcpServer.setMsg(tag, data, info);
	if (!TcpServer.sendMotorMsg()) return -1;
	waitKey(10000);
	//while (1) {
	//	TcpClient.setMsg(tag, data, info);
	//	if(!TcpClient.sendMsg()) return -1;
	//}
	return 0;
}
int demoTcpServerSend_2()
{
	const char* tcpAddr = "127.0.0.1";
	int port = 5000;
	MyTcpipServer TcpServer = MyTcpipServer(tcpAddr, port);
	TcpServer.buildServer();
	TcpServer.setMsg(msgCameraTemplate(100, -1, { 90,-60,-30,20 }, 0, { 100,10,-50,0,0,0 }, { 100,600,-50 }, { 90, 550, -50 }, "wddscws"));
	if (!TcpServer.sendMotorMsg()) return -1;
	waitKey(10000);
	//while (1) {
	//	TcpClient.setMsg(tag, data, info);
	//	if(!TcpClient.sendMsg()) return -1;
	//}
	return 0;
}
int demoTcpServerSend_Linux()
{
	const char* tcpAddr = "192.168.0.36";
	int port = 8081;
	//const char* tcpAddr = "0.0.0.0";
	//int port = 8000;
	MyTcpipServer TcpServer = MyTcpipServer(tcpAddr, port);
	TcpServer.buildServer();
	TcpServer.msgSend[0] = '1';
	TcpServer.msgSend[1] = '2';
	TcpServer.msgSend[2] = '3';
	TcpServer.msgSend[3] = 'a';
	TcpServer.msgSend[4] = 'b';
	TcpServer.msgSend[5] = 'c';
	TcpServer.msgSend[6] = '\0';
	if (!TcpServer.sendMotorMsg()) return -1;
	waitKey(10000);
	//while (1) {
	//	TcpClient.setMsg(tag, data, info);
	//	if(!TcpClient.sendMsg()) return -1;
	//}
	return 0;
}
int demoTcpClientSend_Linux()
{
	const char* tcpAddr = "192.168.0.36";
	int port = 8080;
	//const char* tcpAddr = "0.0.0.0";
	//int port = 8000;
	MyTcpipClient TcpClient = MyTcpipClient(tcpAddr, port);
	TcpClient.buildClient();
	TcpClient.msgSend[0] = '1';
	TcpClient.msgSend[1] = '2';
	TcpClient.msgSend[2] = '3';
	TcpClient.msgSend[3] = 'a';
	TcpClient.msgSend[4] = 'b';
	TcpClient.msgSend[5] = 'c';
	TcpClient.msgSend[6] = '\0';
	if (!TcpClient.sendMsg()) return -1;
	waitKey(10000);
	//while (1) {
	//	TcpClient.setMsg(tag, data, info);
	//	if(!TcpClient.sendMsg()) return -1;
	//}
	return 0;
}
int demoTcpServer()
{
	const char* tcpAddr = "127.0.0.1";
	int port = 5000;
	MyTcpipServer TcpServer = MyTcpipServer(tcpAddr, port);
	TcpServer.buildServer();
	for (int i = 0; i < 100; i++) {
		TcpServer.setMsg(msgCameraTemplate(100, i, { 90,-60,-30,20 }, 0, { 100,10,-50,0,0,0 }, { 100,600,-50 }, { 90, 550, -50 }, "Here is camera message from server!"));
		TcpServer.sendMotorMsg();
		TcpServer.receiveCameraMsg();
	}
	return 0;
}
int demoTcpClient()
{
	//const char* tcpAddr = "127.0.0.1";
	//int port = 5000;
	const char* tcpAddr = "192.168.0.36";
	int port = 8080;
	MyTcpipClient TcpClient = MyTcpipClient(tcpAddr, port);
	TcpClient.buildClient();
	for (int i = 0; i < 10000; i++) {
		TcpClient.setMsg(msgMotorTemplate(100, i, { 90, -60, -30, 20 }, 3.5, {10,15,20,20,5}, "Here is " + to_string(i) + "th message from client!"));
		TcpClient.sendMsg();
		//TcpClient.receiveCameraMsg();
		waitKey(1000);
	}
	return 0;
}

int demoVisualServo_OnlyCamera()
{
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(60.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });
	//设置TCP从端
	const char* tcpAddr = "127.0.0.1";
	MyTcpipClient tcpClient = MyTcpipClient(tcpAddr, 8080);
	tcpClient.buildClient();

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 0;
	const double yCam = -52;
	const double zCam = -8;

	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;
	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		jointActualAngles = { 0,90,-45,0 };

		if (VecProcessor[0].objTvec[2] != 0) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			xTarRela = 0.80 * xTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[0];
			yTarRela = 0.80 * yTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[1];
			//zTarRela = 0.80 * zTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[2];
			//使用目标前方0.1m的点！！！！！！！！！！！
			zTarRela = 0.80 * zTarRela + 0.20 * 1000 * (VecProcessor[0].objTvec[2] - 0.1);
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			//相机沿左边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
			yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
			zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}

		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}

			cameraMsgSent.clamperDesiredSpace = 0;
		}

		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		tcpClient.setMsg(cameraMsgSent);
		tcpClient.sendMsg();

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}


	return 0;
}

int demoVisualServo_Bilateral()
{
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });
	//设置TCP从端
	const char* tcpAddr = "192.168.0.36";
	MyTcpipClient tcpClient = MyTcpipClient(tcpAddr, 8080);
	//const char* tcpAddr = "127.0.0.1";
	//MyTcpipClient tcpClient = MyTcpipClient(tcpAddr, 8080);
	tcpClient.buildClient();

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	//相机安装相对位置（目标在镜头面前13cm）
	const double xCam = 0;
	const double yCam = -0;
	const double zCam = -130;
	////相机安装相对位置（目标在手爪中）
	//const double xCam = 0;
	//const double yCam = -52;
	//const double zCam = -8;

	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;
	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		//jointActualAngles = { 0,90,-90,0 };
		
		tcpClient.receiveMotorMsg();
		motorMsgReceived = tcpClient.getMotorMsg();
		display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}
		cout << endl;

		if (VecProcessor[0].objTvec[2] != 0) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			xTarRela = 0.80 * xTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[0];
			yTarRela = 0.80 * yTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[1];
			//zTarRela = 0.80 * zTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[2];
			//使用目标前方0.1m的点！！！！！！！！！！！
			zTarRela = 0.80 * zTarRela + 0.20 * 1000 * (VecProcessor[0].objTvec[2] - 0.1);
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
			yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
			zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}

		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";//暂时无效
		}
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}

			cameraMsgSent.clamperDesiredSpace = 0;
		}

		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2]<<", q23 = "<< q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);
		
		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		tcpClient.setMsg(cameraMsgSent);
		tcpClient.sendMsg();

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}


	return 0;
}

int demoVisualServo_Bilateral_Grab()
{
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P);
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });
	//设置TCP从端
	const char* tcpAddr = "192.168.0.36";
	MyTcpipClient tcpClient = MyTcpipClient(tcpAddr, 8080);
	//const char* tcpAddr = "127.0.0.1";
	//MyTcpipClient tcpClient = MyTcpipClient(tcpAddr, 8080);
	tcpClient.buildClient();

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 0;
	const double yCam = -50;//-60
	const double zCam = -50;//10

	////期望的目标相对末端位置
	//double end2tarX = 0;
	//double end2tarY = 40;
	//double end2tarZ = 90;
	//抓取指令相关变量
	double angErrTolerance = 0.2;//关节角度误差容忍量
	int grabScore = 0;//角度控制误差小+1，误差大-1，分数达到grabTrigger开始抓取，分数低到0停止抓取
	const int grabTrigger = 40;
	bool canGrab = false;
	const double relaDepthDesiredMax = 200;
	const double relaDepthDesiredMin = 55;
	double relaDepthDesired = relaDepthDesiredMax;
	double relaDepthDesiredFiltered = relaDepthDesired;
	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	bool targetDetected = false;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;
	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		//jointActualAngles = { 0,90,-90,0 };

		tcpClient.receiveMotorMsg();
		motorMsgReceived = tcpClient.getMotorMsg();
		display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}
		cout << endl;

		//计算位置
		if (VecProcessor[0].objTvec[2] != 0)
			targetDetected = true;
		else
			targetDetected = false;
		if (targetDetected) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			if (xTarRela == 0 && yTarRela == 0 && zTarRela == 0) {

				xTarRela = 1000 * VecProcessor[0].objTvec[0];
				yTarRela = 1000 * VecProcessor[0].objTvec[1];
				zTarRela = 1000 * VecProcessor[0].objTvec[2];
			}
			else {
				xTarRela = 0.50 * xTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[0]);
				yTarRela = 0.50 * yTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[1]);
				zTarRela = 0.50 * zTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[2]);
			}
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[0] = objPose.getPosition()[0];
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			if (xTarGlobal == 0 && yTarGlobal == 0 && zTarGlobal == 0) {
				xTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[0];
				yTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[1];
				zTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[2];
				//xTarGlobal = objPose.getPosition()[0];
				//yTarGlobal = objPose.getPosition()[1];
				//zTarGlobal = objPose.getPosition()[2];
			}
			else {
				xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
				yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
				zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			}
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		//目标丢失，标志位置-1
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}

		//逆运动学与轨迹规划
		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			//Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal, relaDepthDesiredFiltered);
			//Robot.InverseKinematics3DofAround(jointDesiredAngles, tarGlobal, {end2tarX, end2tarY, end2tarZ});
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		//逆运动学失败
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		//逆运动学成功
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}
			cameraMsgSent.clamperDesiredSpace = 0;
		}
		//抓取分数及期望距离计算（以后记得把目标丢失、超限位的判断都放进来）
		if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) <= angErrTolerance) {
			if (grabScore < grabTrigger) {
				grabScore++;
			}
			else {
				canGrab = true;
			}
		}
		else if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) >= 0.5 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) >= 0.5 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) >= 0.5) {
			if (grabScore > 0) {
				grabScore--;
			}
			else {
				canGrab = false;
			}
		}
		cout << "******************************" << endl;
		cout << "********Grab Score = " << grabScore << "********" << endl;
		cout << "******************************" << endl;
		//if (grabScore - grabTrigger / 2 > 0) {
		//	relaDepthDesired = relaDepthDesiredMax - (1.0 * (grabScore - grabTrigger / 2)) / (0.5 * grabTrigger) * (relaDepthDesiredMax - relaDepthDesiredMin);
		//}
		//else {
		//	relaDepthDesired = relaDepthDesiredMax;
		//}
		if ((1.0 * grabScore) / (1.0 * grabTrigger) > 0.7) {
			relaDepthDesired = relaDepthDesiredMin;
		}
		else {
			relaDepthDesired = relaDepthDesiredMax;
		}
		relaDepthDesiredFiltered = 0.95 * relaDepthDesiredFiltered + 0.05 * relaDepthDesired;
		//是否可以开始抓取的判断
		if (canGrab && !IKerror && cameraMsgSent.id >= 200) {
			//标志位置1
			noteStr = "Start Grabing! ";
			cameraMsgSent.state = 1;
			//if (targetDetected) {
			//	cameraMsgSent.jointDesiredAngles[0] = motorMsgReceived.jointActualAngles[0];
			//	cameraMsgSent.jointDesiredAngles[1] = motorMsgReceived.jointActualAngles[1];
			//	cameraMsgSent.jointDesiredAngles[2] = motorMsgReceived.jointActualAngles[2];
			//}
		}

		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		tcpClient.setMsg(cameraMsgSent);
		tcpClient.sendMsg();

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}


	return 0;
}


//2023-03-17 还没写完
int upStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, mutex& motorClientLock,
	mutex& plannerClientLock, mutex& msgLock, msgCameraTemplate& msg)
{
	/*设置图像处理功能*/
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P);
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);

	/*设置运动规划功能*/
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });

	/*其他变量初始化*/
	int msgSendId = 0;
	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/
	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 0;
	const double yCam = -50;//-60
	const double zCam = -50;//10
	//抓取指令相关变量
	const double angErrTolerance = 0.2;//关节角度误差容忍量
	int grabScore = 0;//角度控制误差小+1，误差大-1，分数达到grabTrigger开始抓取，分数低到0停止抓取
	const int grabTrigger = 40;
	bool canGrab = false;
	const double relaDepthDesiredMax = 200;
	const double relaDepthDesiredMin = 55;
	double relaDepthDesired = relaDepthDesiredMax;
	double relaDepthDesiredFiltered = relaDepthDesired;
	//目标实际相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	bool targetDetected = false;
	//目标实际全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointActualAngles_last, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	jointActualAngles_last = { 0, -90, -90, 0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	//图像输出
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };//处理过程中的图像序列
	ostringstream outText;//叠加在输出图像上的文字
	//计时变量
	clock_t startTime, endTime;
	clock_t grabStartTime = 0;
	const int grabDuration = 20;

	clock_t time1, time2, time3, time4, time5;
	clock_t time21, time22, time23;

	/*开始循环*/
	while (true)
	{
		try {
		msgSendId++;
		if (msgSendId == 1)
			cout << "***进入上行链路循环***" << endl;
		startTime = clock();

		/*接收电机端数据*/
		//cout << "上行线程：1.从电机端接受数据" << endl;
			//motorClientLock.lock();
		if (clientMotor.receiveMotorMsg() <= 0) {
			cout << "电机端暂无数据" << endl;
			continue;
		}
		//cout << "本轮flush了 " << clientMotor.flushRcvBuf() << " 次" << endl;
		cout << "本轮flush了 " << clientMotor.flushRcvBuf() << " bytes" << endl;
		motorMsgReceived = clientMotor.getMotorMsg();
		//motorClientLock.unlock();
		//display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		//cout << "实际关节角度： " << jointActualAngles;

		//}
		//catch (...) {
		//	cout << "up1有问题" << endl;
		//	jointActualAngles = jointActualAngles_last;
		//}
		time1 = clock();
		cout << "time1 = " << time1 - startTime << endl;

		/*图像处理*/
		//cout << "上行线程：2.处理数据" << endl;
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}
		//判断是否发现目标
		if (VecProcessor[0].objTvec[2] != 0)
			targetDetected = true;
		else
			targetDetected = false;

		time21 = clock();
		cout << "time21 = " << time21 - time1 << endl;

		/*目标位置解算*/
		if (targetDetected) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			if (xTarRela == 0 && yTarRela == 0 && zTarRela == 0) {

				xTarRela = 1000 * VecProcessor[0].objTvec[0];
				yTarRela = 1000 * VecProcessor[0].objTvec[1];
				zTarRela = 1000 * VecProcessor[0].objTvec[2];
			}
			else {
				xTarRela = 0.50 * xTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[0]);
				yTarRela = 0.50 * yTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[1]);
				zTarRela = 0.50 * zTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[2]);
			}
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[0] = objPose.getPosition()[0];
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			if (xTarGlobal == 0 && yTarGlobal == 0 && zTarGlobal == 0) {
				xTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[0];
				yTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[1];
				zTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[2];
			}
			else {
				xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
				yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
				zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			}
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		//目标丢失，标志位置-1
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}
		time2 = clock();
		cout << "time2 = " << time2 - time1 << endl;

		/*逆运动学*/
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal, relaDepthDesiredFiltered);
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			IKerror = true;
			jointDesiredAngles = jointDesiredAngles_last;
		}
		//逆运动学失败
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		//逆运动学成功
		else {
			noteStr += "IK solved. ";
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			cameraMsgSent.clamperDesiredSpace = 0;
		}

		/*抓取分数及期望距离计算*/
		//控制误差够小、目标未丢失、逆运动学有解时增加抓取分数，接近目标
		if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) <= angErrTolerance &&
			targetDetected && !IKerror) {
			//误差分数增加
			if (grabScore < grabTrigger) {
				grabScore++;
			}
			//如果误差分数到顶了，开始抓取
			else {
				canGrab = true;
			}
		}
		//控制误差太大时减小抓取分数，远离目标
		else if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) >= 0.5 ||
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) >= 0.5 ||
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) >= 0.5)
		{
			if (grabScore > 0) {
				grabScore--;
			}
			else {
				canGrab = false;
			}
		}
		//根据抓取分数计算接近目标的程度
		if ((1.0 * grabScore) / (1.0 * grabTrigger) > 0.7) {
			relaDepthDesired = relaDepthDesiredMin;
		}
		else {
			relaDepthDesired = relaDepthDesiredMax;
		}
		relaDepthDesiredFiltered = 0.95 * relaDepthDesiredFiltered + 0.05 * relaDepthDesired;
		//是否可以开始抓取的判断
		if (canGrab && !IKerror && cameraMsgSent.id >= 200) {
			//标志位置1
			noteStr = "Start Grabing! ";
			cameraMsgSent.state = 1;
			if (grabStartTime == 0) {
				grabStartTime = clock();
			}
		}
		//抓取时长是否到位的判断
		if (clock() - grabStartTime >= grabDuration * 1000) {
			noteStr = "Grabing Complete! ";
			cameraMsgSent.state = 2;
		}

		time3 = clock();
		cout << "time3 = " << time3 - time2 << endl;

		/*图像叠加输出文字*/
		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i]; //最多只能存56个字符
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);


		/*计算大致帧率*/
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;


		time4 = clock();
		cout << "time4 = " << time4 - time3 << endl;

		//try{
		/*传输关节角度信号*/
		//cout << "上行线程：3.发送数据给规划端" << endl;
		display(cameraMsgSent);
		//调试用
		cameraMsgSent.jointDesiredAngles[0] = 0;
		cameraMsgSent.jointDesiredAngles[1] = -90.0 / (msgSendId / 10.0 + 1.0);
		cameraMsgSent.jointDesiredAngles[2] = -90.0 / (msgSendId / 10.0 + 1.0);
		//msgLock.lock();
		msg = cameraMsgSent;
		//msgLock.unlock();
		//plannerClientLock.lock();
		clientPlanner.sendDoubleMsg(vector<double>({cameraMsgSent.jointDesiredAngles[0], cameraMsgSent.jointDesiredAngles[1],
			cameraMsgSent.jointDesiredAngles[2]}));
		//plannerClientLock.unlock();
		

		time5 = clock();
		cout << "time5 = " << time5 - time4 << endl;

		}
		catch (...) {
			cout << "up2有问题" << endl;
			continue;
		}

		//imshow("Detect", VecImg[VecImg.size() - 1]);
		//waitKey(1);

		Sleep(1);

	}
	return -1;
}
//接受电机端数据+视觉伺服+发送给simulink轨迹规划，频率30hz左右
int upStream(MyTcpipClient clientMotor, MyTcpipClient clientPlanner)
{
	//以下是不需要进循环的内容，所以放在线程while外面
	// 
	// 
	// 
	//视觉伺服处理，预计30hz左右
	//以下是视觉伺服部分代码 从demoVisualServo_Bilateral魔改  20230318
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });

	//相机安装相对位置（目标在镜头面前13cm）
	const double xCam = 0;
	const double yCam = -0;
	const double zCam = -130;
	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;


	while (true)
	{
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}
		//jointActualAngles = { 0,90,-90,0 };



		//接受电机端（clientMotor）的数据
		clientMotor.receiveMotorMsg();
		cout << "上行线程：1.从电机端接受数据" << endl;


		//tcpClient.receiveMotorMsg();
		motorMsgReceived = clientMotor.getMotorMsg();
		display(motorMsgReceived);


		cout << "上行线程：2.处理数据" << endl;
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}
		cout << endl;

		if (VecProcessor[0].objTvec[2] != 0) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			xTarRela = 0.80 * xTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[0];
			yTarRela = 0.80 * yTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[1];
			//zTarRela = 0.80 * zTarRela + 0.20 * 1000 * VecProcessor[0].objTvec[2];
			//使用目标前方0.1m的点！！！！！！！！！！！
			zTarRela = 0.80 * zTarRela + 0.20 * 1000 * (VecProcessor[0].objTvec[2] - 0.1);
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
			yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
			zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}

		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";//暂时无效
		}
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}

			cameraMsgSent.clamperDesiredSpace = 0;
		}

		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);



		//发送规划端（clientPlanner）期望角度，角度+状态标志
		//需要重写cameraMsgSent部分 把状态标志一并发过去

		cout << "上行线程：3.发送数据给规划端" << endl;
		clientPlanner.sendDoubleMsg(vector<double>({ -1.1, 3.18, pow(10,20) }));
		//clientPlanner.setMsg(cameraMsgSent);
		//clientPlanner.sendMsg();
		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}
	return -1;
}

//接受simulink轨迹规划结果+数据包打包+发送给电机端，频率500hz左右
int downStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, mutex& motorClientLock,
	mutex& plannerClientLock, mutex& msgLock, msgCameraTemplate& msg)
{
	int msgSendId = 0;
	int len = 0;
	double q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	clock_t timeLast = clock();
	while (true)
	{
		msgSendId++;
		if (msgSendId == 1)
			cout << "***进入下行链路循环***" << endl;

		//接受规划端（clientPlanner）的数据，注意判断缓存区是否有新数据
		vector<double> rcvData;
		rcvData.resize(9);
		//cout << "下行线程：1.从规划端接收数据" << endl;
		//plannerClientLock.lock();
		len = clientPlanner.receiveDoubleMsg(rcvData);
		//cout << "接收到数据 " << len << " 个字节" << endl;
		//plannerClientLock.unlock();
		//接收到非空数据
		if (len) {
			q1 = rcvData[0];
			q2 = rcvData[1];
			q3 = rcvData[2];
			dq1 = rcvData[3];
			dq2 = rcvData[4];
			dq3 = rcvData[5];
			ddq1 = rcvData[6];
			ddq2 = rcvData[7];
			ddq3 = rcvData[8];
			//cout << "===========收到的规划端数据============" << endl;
			bool validData = true;
			for (int i = 0; i++; i < 9) {
				if (isnan(rcvData[i]))
					validData = false;
				//cout << rcvData[i] << ", ";
			}
			//cout << endl;
			if (!validData)
				continue;
		}
		//暂无数据可接收
		else {
			cout << "!!!!暂无数据可接收!!!!" << endl;
			continue;
		}


		if (msgSendId % 100 == 0) {
			//cout << msgSendId << endl;
			//cout << clientPlanner.flushRcvBuf() << endl;
			clientPlanner.flushRcvBuf();
		}


		while (clock() - timeLast <= 2) {
			continue;
		}
		
		//数据打包（角度、角速度、角加速度、状态标志打包成msgCameraTemplate）
		//cout << "下行线程：2.数据打包" << endl;
		//msgLock.lock();
		cameraMsgSent = msg;
		//msgLock.unlock();
		cameraMsgSent.jointDesiredAngles[0] = q1;
		cameraMsgSent.jointDesiredAngles[1] = q2;
		cameraMsgSent.jointDesiredAngles[2] = q3;
		cameraMsgSent.jointDesiredAngles[3] = 0;
		cameraMsgSent.reservedData[0] = dq1;
		cameraMsgSent.reservedData[1] = dq2;
		cameraMsgSent.reservedData[2] = dq3;
		cameraMsgSent.reservedData[3] = ddq1;
		cameraMsgSent.reservedData[4] = ddq2;
		cameraMsgSent.reservedData[5] = ddq3;
		cameraMsgSent.state = 0;//不抓取
		cameraMsgSent.id = msgSendId;

		//发送数据给电机端（clientMotor）
		//cout << "下行线程：3.发送数据给电机端" << endl;
		//motorClientLock.lock();
		timeLast = clock();
		cameraMsgSent.id = msgSendId;
		clientMotor.setMsg(cameraMsgSent);
		clientMotor.sendMsg();
		//motorClientLock.unlock();
		//Sleep(1);
	}
	return -1;
}

int dualThread()
{
	//建立电机端通信
	/*
	IP地址：192.168.0.36
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr1 = "192.168.0.36";
	int port1 = 8080;
	MyTcpipClient TcpClient1 = MyTcpipClient(tcpAddr1, port1);

	TcpClient1.buildClient();
	cout << "建立与电机端的通信成功" << endl;

	//建立规划端通信
	/*
	IP地址：192.168.0.37
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr2 = "192.168.0.37";
	int port2 = 8000;
	MyTcpipClient TcpClient2 = MyTcpipClient(tcpAddr2, port2);
	TcpClient2.buildClient();
	//TcpClient.setMsg(msgMotorTemplate(100, i, { 90, -60, -30, 20 }, 3.5, { 10,15,20,20,5 }, "Here is " + to_string(i) + "th message from client!"));
	//TcpClient.sendMsg();

	mutex motorClientLock, plannerClientLock, msgLock;
	msgCameraTemplate msg = msgCameraTemplate();



	//thread upThread(upStreamTest, TcpClient1, TcpClient2, std::ref(motorClientLock), std::ref(plannerClientLock), std::ref(msgLock), std::ref(msg));
	thread downThread(downStreamTest, TcpClient1, TcpClient2, std::ref(motorClientLock), std::ref(plannerClientLock), std::ref(msgLock), std::ref(msg));
	//thread upThread(upStream, TcpClient1, TcpClient2);
	//thread downThread(Stream3Test, TcpClient2);
	//upThread.detach();
	downThread.detach();

	upStreamTest(TcpClient1, TcpClient2, std::ref(motorClientLock), std::ref(plannerClientLock), std::ref(msgLock), std::ref(msg));

	//downStreamTest(TcpClient1, TcpClient2, std::ref(motorClientLock), std::ref(plannerClientLock), std::ref(msgLock), std::ref(msg));

	while (1) {}
	return 0;
}

int downStreamTest2(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, mutex& motorClientLock,
	mutex& plannerClientLock, mutex& msgLock, msgCameraTemplate& msg)
{
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSendId = 0;
	while (true)
	{
		msgSendId++;
		if (msgSendId == 1)
			cout << "***进入下行链路循环***" << endl;

		if (msg.reservedTag == 1) {
			cameraMsgSent = msg;
			cameraMsgSent.id = msgSendId;
			msg.reservedTag = 0;
		}

		//发送数据给电机端（clientMotor）
		//cout << "下行线程：3.发送数据给电机端" << endl;
		//motorClientLock.lock();
		clientMotor.setMsg(cameraMsgSent);
		clientMotor.sendMsg();
		//motorClientLock.unlock();
		//Sleep(1);
	}
	return -1;
}


//2023-03-21 三边通信的抓取控制（PID版）
int demoVisualServo_Trilateral()
{
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P);
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });

	//设置TCP从端

	//建立电机端通信
	/*
	IP地址：192.168.0.36
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr1 = "192.168.0.36";
	int port1 = 8080;
	MyTcpipClient TcpClient1 = MyTcpipClient(tcpAddr1, port1);

	TcpClient1.buildClient();
	cout << "建立与电机端的通信成功" << endl;

	//建立规划端通信
	/*
	IP地址：192.168.0.37
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr2 = "192.168.0.37";
	int port2 = 8000;
	MyTcpipClient TcpClient2 = MyTcpipClient(tcpAddr2, port2);
	TcpClient2.buildClient();
	cout << "建立与规划端的通信成功" << endl;

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 0;
	const double yCam = -55;//-60
	const double zCam = -50;//10

	////期望的目标相对末端位置
	//double end2tarX = 0;
	//double end2tarY = 40;
	//double end2tarZ = 90;
	//抓取指令相关变量
	double angErrTolerance = 0.2;//关节角度误差容忍量
	int grabScore = 0;//角度控制误差小+1，误差大-1，分数达到grabTrigger开始抓取，分数低到0停止抓取
	const int grabTrigger = 40;
	bool canGrab = false;
	const double relaDepthDesiredMax = 200;
	const double relaDepthDesiredMin = 55;
	double relaDepthDesired = relaDepthDesiredMax;
	double relaDepthDesiredFiltered = relaDepthDesired;
	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	bool targetDetected = false;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	ostringstream outText;
	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		//jointActualAngles = { 0,90,-90,0 };

		TcpClient1.receiveMotorMsg();
		motorMsgReceived = TcpClient1.getMotorMsg();
		display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}
		cout << endl;

		//计算位置
		if (VecProcessor[0].objTvec[2] != 0)
			targetDetected = true;
		else
			targetDetected = false;
		if (targetDetected) {
			cameraMsgSent.state = 0;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			if (xTarRela == 0 && yTarRela == 0 && zTarRela == 0) {

				xTarRela = 1000 * VecProcessor[0].objTvec[0];
				yTarRela = 1000 * VecProcessor[0].objTvec[1];
				zTarRela = 1000 * VecProcessor[0].objTvec[2];
			}
			else {
				xTarRela = 0.50 * xTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[0]);
				yTarRela = 0.50 * yTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[1]);
				zTarRela = 0.50 * zTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[2]);
			}
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[0] = objPose.getPosition()[0];
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			if (xTarGlobal == 0 && yTarGlobal == 0 && zTarGlobal == 0) {
				xTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[0];
				yTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[1];
				zTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[2];
				//xTarGlobal = objPose.getPosition()[0];
				//yTarGlobal = objPose.getPosition()[1];
				//zTarGlobal = objPose.getPosition()[2];
			}
			else {
				xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
				yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
				zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			}
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		//目标丢失，标志位置-1
		else {
			cameraMsgSent.state = -1;
			noteStr = "Target missed! ";
		}

		//逆运动学与轨迹规划
		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			//Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal, relaDepthDesiredFiltered);
			//Robot.InverseKinematics3DofAround(jointDesiredAngles, tarGlobal, {end2tarX, end2tarY, end2tarZ});
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		//逆运动学失败
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		//逆运动学成功
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}
			cameraMsgSent.clamperDesiredSpace = 0;
		}
		//抓取分数及期望距离计算（以后记得把目标丢失、超限位的判断都放进来）
		if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) <= angErrTolerance &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) <= angErrTolerance) {
			if (grabScore < grabTrigger) {
				grabScore++;
			}
			else {
				canGrab = true;
			}
		}
		else if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) >= 1.0 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) >= 1.0 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) >= 1.0) {
			if (grabScore > 0) {
				grabScore--;
			}
			else {
				canGrab = false;
			}
		}
		cout << "******************************" << endl;
		cout << "********Grab Score = " << grabScore << "********" << endl;
		cout << "******************************" << endl;
		//if (grabScore - grabTrigger / 2 > 0) {
		//	relaDepthDesired = relaDepthDesiredMax - (1.0 * (grabScore - grabTrigger / 2)) / (0.5 * grabTrigger) * (relaDepthDesiredMax - relaDepthDesiredMin);
		//}
		//else {
		//	relaDepthDesired = relaDepthDesiredMax;
		//}
		if ((1.0 * grabScore) / (1.0 * grabTrigger) > 0.7) {
			relaDepthDesired = relaDepthDesiredMin;
		}
		else {
			relaDepthDesired = relaDepthDesiredMax;
		}
		relaDepthDesiredFiltered = 0.95 * relaDepthDesiredFiltered + 0.05 * relaDepthDesired;
		//是否可以开始抓取的判断
		if (canGrab && !IKerror && cameraMsgSent.id >= 200) {
			//标志位置1
			noteStr = "Start Grabing! ";
			cameraMsgSent.state = 1;
			//if (targetDetected) {
			//	cameraMsgSent.jointDesiredAngles[0] = motorMsgReceived.jointActualAngles[0];
			//	cameraMsgSent.jointDesiredAngles[1] = motorMsgReceived.jointActualAngles[1];
			//	cameraMsgSent.jointDesiredAngles[2] = motorMsgReceived.jointActualAngles[2];
			//}
		}

		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		TcpClient2.sendDoubleMsg({ cameraMsgSent.jointDesiredAngles[0], cameraMsgSent.jointDesiredAngles[1], cameraMsgSent.jointDesiredAngles[2], double(cameraMsgSent.state) });

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}

	return 0;
}

//2023-03-22 三边通信的抓取控制（ARC版）
int demoVisualServo_Trilateral2_Grabbing()
{
	//*******************************************主要对象设置*************************************************
	
	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P);
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_UNDERWATER_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });
	//处理过程中涉及的图像序列
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	//最后需要叠加在图像上的文字
	ostringstream outText;

	//*******************************************设置TCP从端*************************************************

	//建立电机端通信
	/*
	IP地址：192.168.0.36
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr1 = "192.168.0.36";
	int port1 = 8080;
	MyTcpipClient TcpClient1 = MyTcpipClient(tcpAddr1, port1);

	TcpClient1.buildClient();
	cout << "建立与电机端的通信成功" << endl;

	//建立规划端通信
	/*
	IP地址：192.168.0.37
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr2 = "192.168.0.37";
	int port2 = 8000;
	MyTcpipClient TcpClient2 = MyTcpipClient(tcpAddr2, port2);
	TcpClient2.buildClient();
	cout << "建立与规划端的通信成功" << endl;

	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;


	//*******************************************主要参数设置*************************************************

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 15;
	const double yCam = -55;//-60
	const double zCam = -50;//10
	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	bool targetDetected = false;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	////期望的目标相对末端位置
	//double end2tarX = 0;
	//double end2tarY = 40;
	//double end2tarZ = 90;
	//抓取指令相关变量
	double angErrTolerance1 = 0.2;//关节角度误差容忍量1，满足时grabscore+1
	double angErrTolerance2 = 1.0;//关节角度误差容忍量2，不满足时grabscore-1
	int grabScore = 0;//角度控制误差小+1，误差大-1
	const int grabTrigger = 40;//分数达到grabTrigger开始抓取，分数低到0停止抓取（停止抓取功能未实现）
	bool canGrab = false;
	const double relaDepthDesiredMax = 200*0.65;//grabscore最小时，最远的相对距离
	const double relaDepthDesiredMin = 55*0.65;//grabscore最高时，最近的相对距离
	double relaDepthDesired = relaDepthDesiredMax;
	double relaDepthDesiredFiltered = relaDepthDesired;
	bool hasBeenGrabbing = false; //是否开始抓取的标志
	int grabDuration = 20 * 1000; //20s后认为抓取结束，开始归位
	clock_t grabStartTime = INFINITE;
	bool hasBeenHoming = false; //是否开始归位的标志
	clock_t homingStartTime = INFINITE;
	//作业状态
	int state = STATE_NORMAL;

	//*******************************************主循环*************************************************

	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();

		//*******************************************图像处理*************************************************
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		//jointActualAngles = { 0,0,-90,0 };

		//*******************************************接收电机端数据*************************************************
		TcpClient1.receiveMotorMsg();
		motorMsgReceived = TcpClient1.getMotorMsg();
		display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		//jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[1] = jointActualAngles[1];//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}

		cout << endl;


		//*******************************************目标定位与逆运动学*************************************************
		//计算位置
		if (VecProcessor[0].objTvec[2] != 0)
			targetDetected = true;
		else
			targetDetected = false;
		if (targetDetected) {
			state = STATE_NORMAL;
			cameraMsgSent.state = state;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			if (xTarRela == 0 && yTarRela == 0 && zTarRela == 0) {

				xTarRela = 1000 * VecProcessor[0].objTvec[0];
				yTarRela = 1000 * VecProcessor[0].objTvec[1];
				zTarRela = 1000 * VecProcessor[0].objTvec[2];
			}
			else {
				xTarRela = 0.50 * xTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[0]);
				yTarRela = 0.50 * yTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[1]);
				zTarRela = 0.50 * zTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[2]);
			}
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[0] = objPose.getPosition()[0];
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			if (xTarGlobal == 0 && yTarGlobal == 0 && zTarGlobal == 0) {
				xTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[0];
				yTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[1];
				zTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[2];
				//xTarGlobal = objPose.getPosition()[0];
				//yTarGlobal = objPose.getPosition()[1];
				//zTarGlobal = objPose.getPosition()[2];
			}
			else {
				xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
				yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
				zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			}
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		//目标丢失，标志位置-1
		else {
			state = STATE_TARGETLOSS;
			cameraMsgSent.state = state;
			noteStr = "Target missed! ";
		}

		//逆运动学与轨迹规划
		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			//Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal, relaDepthDesiredFiltered);
			//Robot.InverseKinematics3DofAround(jointDesiredAngles, tarGlobal, {end2tarX, end2tarY, end2tarZ});
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		//逆运动学失败
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		//逆运动学成功
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			//cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1];
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}
			cameraMsgSent.clamperDesiredSpace = 0;
		}

		//*******************************************抓取策略*************************************************
		//抓取分数及期望距离计算（以后记得把目标丢失、超限位的判断都放进来）
		if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) <= angErrTolerance1 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) <= angErrTolerance1 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) <= angErrTolerance1) {
			if (grabScore < grabTrigger) {
				grabScore++;
			}
			else {
				if (!hasBeenGrabbing) {
					grabStartTime = clock();
				}
				canGrab = true;
				hasBeenGrabbing = true;
			}
		}
		else if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) >= angErrTolerance2 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) >= angErrTolerance2 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) >= angErrTolerance2) {
			if (grabScore > 0) {
				grabScore--;
			}
			else {
				canGrab = false;
			}
		}
		//cout << "******************************" << endl;
		//cout << "********Grab Score = " << grabScore << "********" << endl;
		//cout << "******************************" << endl;
		if ((1.0 * grabScore) / (1.0 * grabTrigger) > 0.7) {
			relaDepthDesired = relaDepthDesiredMin;
		}
		else {
			relaDepthDesired = relaDepthDesiredMax;
		}
		relaDepthDesiredFiltered = 0.95 * relaDepthDesiredFiltered + 0.05 * relaDepthDesired;
		//是否可以开始抓取的判断
		if (canGrab && !IKerror && cameraMsgSent.id >= 200) {
			//标志位置1
			noteStr = "Start Grabing! ";
			state = STATE_GRAB;
			cameraMsgSent.state = state;
		}
		//是否已经抓取够久了
		if (hasBeenGrabbing) {
			//开始抓取后则不再改变关节角度
			jointDesiredAngles = jointDesiredAngles_last;
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			//cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1];
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			if (!hasBeenHoming && motorMsgReceived.state == 2) {
				state = STATE_HOMING;
				cameraMsgSent.state = state;
				noteStr = "Start Homing! ";
				homingStartTime = clock();
				hasBeenHoming = true;
			}
			if (hasBeenHoming) {
				state = STATE_HOMING;
				cameraMsgSent.state = state;
				noteStr = "Start Homing! ";
				cameraMsgSent.jointDesiredAngles[0] = 0;
				cameraMsgSent.jointDesiredAngles[1] = 0;
			}
			if (hasBeenHoming && clock() - homingStartTime >= 5*1000) {
				//cameraMsgSent.jointDesiredAngles[2] = -80;
				cameraMsgSent.jointDesiredAngles[2] = -90;
			}
		}

		//if (state == STATE_HOMING) {
		//	cameraMsgSent.jointDesiredAngles[0] = 0;
		//	cameraMsgSent.jointDesiredAngles[1] = 0;
		//	cameraMsgSent.jointDesiredAngles[2] = -80;
		//}

		//*******************************************图像界面输出文字*************************************************
		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		//*******************************************向规划端传输数据*************************************************
		TcpClient2.sendDoubleMsg({ cameraMsgSent.jointDesiredAngles[0], cameraMsgSent.jointDesiredAngles[1], cameraMsgSent.jointDesiredAngles[2], double(cameraMsgSent.state) });

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}

	return 0;
}


//2023-03-26 三边通信的跟随控制（ARC版）
int demoVisualServo_Trilateral2_Tracking()
{
	//*******************************************主要对象设置*************************************************

	//设置从摄像头读取视频的参数
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(20.0);
	//设置针对Aruco方块的图像处理器
	vector<MyImagePreprocess> VecPreprocessor;
	VecPreprocessor.push_back(MyImagePreprocess());
	VecPreprocessor[0].setParams(enum_PreprocessorType::CVT_GRAY);
	vector<MyImageProcess> VecProcessor;
	VecProcessor.push_back(MyImageProcess());
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_IN_AIR_640P);
	//VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_IN_AIR_640P);
	VecProcessor[0].setParams(enum_ProcessorType::ARUCO_CUBE, enum_ImagingCondition::CAM170_SEALED_UNDERWATER_640P);
	//设置机器人模型
	RoboticKinematics Robot = RoboticKinematics();
	Robot.setDefaultRobot();
	//设置实时轨迹规划器
	RealTimeTrajPlanner RTPlanner = RealTimeTrajPlanner(enum_TrajType::REALTIME_ASYMPTOTIC, 1.0, { 0,0,0 });
	//处理过程中涉及的图像序列
	vector<Mat> VecImg = { Mat(), Mat(), Mat(), Mat() };
	//最后需要叠加在图像上的文字
	ostringstream outText;

	//*******************************************设置TCP从端*************************************************

	//建立电机端通信
	/*
	IP地址：192.168.0.36
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr1 = "192.168.0.36";
	//const char* tcpAddr1 = "127.0.0.36";
	int port1 = 8080;
	MyTcpipClient TcpClient1 = MyTcpipClient(tcpAddr1, port1);

	TcpClient1.buildClient();
	cout << "建立与电机端的通信成功" << endl;

	//建立规划端通信
	/*
	IP地址：192.168.0.37
	端口号：8080
	电机端为Server本地为Client
	*/
	const char* tcpAddr2 = "192.168.0.37";
	int port2 = 8000;
	MyTcpipClient TcpClient2 = MyTcpipClient(tcpAddr2, port2);
	TcpClient2.buildClient();
	cout << "建立与规划端的通信成功" << endl;

	//消息体
	msgMotorTemplate motorMsgReceived;
	msgCameraTemplate cameraMsgSent = msgCameraTemplate();
	int msgSentId = 0;
	string noteStr;


	//*******************************************主要参数设置*************************************************

	/*
	* 密封舱长宽高47x44x40mm
	* 圆柱关节到虎口90mm
	* 镜头距离亚克力板3mm、亚克力板厚4mm
	* 夹持时物体中心距离虎口40mm、跟随时150mm
	*/

	////相机安装相对位置（目标在镜头面前13cm）
	//const double xCam = 0;
	//const double yCam = -0;
	//const double zCam = -130;
	//相机安装相对位置（目标在手爪中）
	const double xCam = 15;
	const double yCam = -55;//-60
	const double zCam = -50;//10
	//目标相对位置
	double xTarRela, yTarRela, zTarRela;
	xTarRela = 0, yTarRela = 0, zTarRela = 0;
	bool targetDetected = false;
	//目标全局位置
	double xTarGlobal, yTarGlobal, zTarGlobal;
	vector<double> tarGlobal;
	xTarGlobal = 0, yTarGlobal = 0, zTarGlobal = 0;
	//关节实际与期望角度
	vector<double> jointActualAngles, jointDesiredAngles, endPose;
	jointDesiredAngles = { 0,0,0,0 };
	PoseMatrix objPose;
	////期望的目标相对末端位置
	//double end2tarX = 0;
	//double end2tarY = 40;
	//double end2tarZ = 90;
	//抓取指令相关变量
	double angErrTolerance1 = 0.2;//关节角度误差容忍量1，满足时grabscore+1
	double angErrTolerance2 = 1.0;//关节角度误差容忍量2，不满足时grabscore-1
	int grabScore = 0;//角度控制误差小+1，误差大-1
	const int grabTrigger = 40;//分数达到grabTrigger开始抓取，分数低到0停止抓取（停止抓取功能未实现）
	bool canGrab = false;
	const double relaDepthDesiredMax = 200*0.65;//grabscore最小时，最远的相对距离
	const double relaDepthDesiredMin = 55*0.65;//grabscore最高时，最近的相对距离
	double relaDepthDesired = relaDepthDesiredMax;
	double relaDepthDesiredFiltered = relaDepthDesired;
	bool hasBeenGrabbing = false; //是否开始抓取的标志
	int grabDuration = 20 * 1000; //20s后认为抓取结束，开始归位
	clock_t grabStartTime = INFINITE;
	bool hasBeenHoming = false; //是否开始归位的标志
	clock_t homingStartTime = INFINITE;
	//作业状态
	int state = STATE_NORMAL;

	//*******************************************主循环*************************************************

	while (1) {
		cameraMsgSent.id = ++msgSentId;
		clock_t startTime, endTime;
		startTime = clock();

		//*******************************************图像处理*************************************************
		streamObj.getFrame().copyTo(VecImg[0]);
		VecImg[0].copyTo(VecImg[VecImg.size() - 1]);
		for (int iStep = 0; iStep < VecPreprocessor.size(); iStep++)
		{
			VecPreprocessor[iStep].Preprocess(VecImg[iStep], VecImg[iStep + 1]);
		}
		for (int iStep = VecPreprocessor.size(); iStep < VecPreprocessor.size() + VecProcessor.size(); iStep++)
		{
			VecProcessor[iStep - VecPreprocessor.size()].Process(VecImg[iStep], VecImg[iStep + 1], VecImg[VecImg.size() - 1]);
		}

		//jointActualAngles = { 0,0,-90,0 };

		//*******************************************接收电机端数据*************************************************
		TcpClient1.receiveMotorMsg();
		motorMsgReceived = TcpClient1.getMotorMsg();
		display(motorMsgReceived);
		jointActualAngles.clear();
		for (int i = 0; i < 4; i++) {
			jointActualAngles.push_back(motorMsgReceived.jointActualAngles[i]);
		}
		//jointActualAngles = vector<double>(motorMsgReceived.jointActualAngles, motorMsgReceived.jointActualAngles + 4);
		jointActualAngles[0] = jointActualAngles[0];//控制端：1关节右转为正?，视觉端：1关节左转为正
		//jointActualAngles[1] = jointActualAngles[1] + 90;//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[1] = jointActualAngles[1];//控制端：2关节外抬为负，下垂为0度，视觉端：2关节内缩为正，抬平0度
		jointActualAngles[2] = -jointActualAngles[2];//控制端：3关节内缩为负，视觉端：3关节内缩为正
		for (int i = 0; i < 4; i++) {
			cout << jointActualAngles[i] << " ";
		}

		cout << endl;


		//*******************************************目标定位与逆运动学*************************************************
		//计算位置
		if (VecProcessor[0].objTvec[2] != 0)
			targetDetected = true;
		else
			targetDetected = false;
		if (targetDetected) {
			state = STATE_NORMAL;
			cameraMsgSent.state = state;
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i] = 1000 * VecProcessor[0].objTvec[i];
			for (int i = 0; i < 3; i++)
				cameraMsgSent.targetRelativePose[i + 3] = VecProcessor[0].objRvec[i];
			if (xTarRela == 0 && yTarRela == 0 && zTarRela == 0) {

				xTarRela = 1000 * VecProcessor[0].objTvec[0];
				yTarRela = 1000 * VecProcessor[0].objTvec[1];
				zTarRela = 1000 * VecProcessor[0].objTvec[2];
			}
			else {
				xTarRela = 0.50 * xTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[0]);
				yTarRela = 0.50 * yTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[1]);
				zTarRela = 0.50 * zTarRela + 0.50 * 1000 * (VecProcessor[0].objTvec[2]);
			}
			////相机沿下边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(xTarRela, yTarRela, zTarRela));
			////相机沿左边安装
			//objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(yTarRela, -xTarRela, zTarRela));
			//相机沿右边安装
			objPose = Robot.ForwardKinematics(jointActualAngles, endPose) * (TransMat(xCam, yCam, zCam)) * (TransMat(-yTarRela, xTarRela, zTarRela));
			cameraMsgSent.targetGlobalPosition[0] = objPose.getPosition()[0];
			cameraMsgSent.targetGlobalPosition[1] = objPose.getPosition()[1];
			cameraMsgSent.targetGlobalPosition[2] = objPose.getPosition()[2];
			if (xTarGlobal == 0 && yTarGlobal == 0 && zTarGlobal == 0) {
				xTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[0];
				yTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[1];
				zTarGlobal = Robot.ForwardKinematics(jointActualAngles, endPose).getPosition()[2];
				//xTarGlobal = objPose.getPosition()[0];
				//yTarGlobal = objPose.getPosition()[1];
				//zTarGlobal = objPose.getPosition()[2];
			}
			else {
				xTarGlobal = 0.95 * xTarGlobal + 0.05 * objPose.getPosition()[0];
				yTarGlobal = 0.95 * yTarGlobal + 0.05 * objPose.getPosition()[1];
				zTarGlobal = 0.95 * zTarGlobal + 0.05 * objPose.getPosition()[2];
			}
			//xTarGlobal = 0.98 * xTarGlobal + 0.02 * objPose.getPosition()[0];
			//yTarGlobal = 0.98 * yTarGlobal + 0.02 * objPose.getPosition()[1];
			//zTarGlobal = 0.98 * zTarGlobal + 0.02 * objPose.getPosition()[2];
			cameraMsgSent.objectivePointGlobalPosition[0] = xTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[1] = yTarGlobal;
			cameraMsgSent.objectivePointGlobalPosition[2] = zTarGlobal;
			noteStr = "Target localized. ";
		}
		//目标丢失，标志位置-1
		else {
			state = STATE_TARGETLOSS;
			cameraMsgSent.state = state;
			noteStr = "Target missed! ";
		}

		//逆运动学与轨迹规划
		//RTPlanner.NewPoint({ xTarGlobal,yTarGlobal,zTarGlobal });
		//RTPlanner.NewPoint({ objPose.getPosition()[0], objPose.getPosition()[1], objPose.getPosition()[2] });
		//tarGlobal = RTPlanner.getCurrentTraj();
		tarGlobal = { xTarGlobal,yTarGlobal,zTarGlobal };
		bool IKerror = false;
		bool TowardIKerror = false;
		double q23_last = (jointDesiredAngles[1] + jointDesiredAngles[2]) / 180.0 * 3.1415926535;
		vector<double> jointDesiredAngles_last = jointDesiredAngles;
		try {
			//Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal);
			Robot.InverseKinematics3DofPosition(jointDesiredAngles, tarGlobal, relaDepthDesiredFiltered);
			//Robot.InverseKinematics3DofAround(jointDesiredAngles, tarGlobal, {end2tarX, end2tarY, end2tarZ});
		}
		catch (...) {
			TowardIKerror = true;
			//try {
			//	//if(!isnan(q23_last))
			//		Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, q23_last);
			//	//else
			//		//Robot.InverseKinematics3DofToward(jointDesiredAngles, tarGlobal, 0);
			//}
			//catch (...) {
			//	TowardIKerror = true;
			//}
			IKerror = true;
		}
		if (isnan(jointDesiredAngles[2]) || !Robot.IsValidAngles(jointDesiredAngles)) {
			jointDesiredAngles = jointDesiredAngles_last;
		}
		//逆运动学失败
		if (IKerror) {
			if (TowardIKerror)noteStr += "IK unsolvable! ";
			else noteStr += "Switched to toward IK! ";
		}
		//逆运动学成功
		else {
			noteStr += "IK solved. ";
			//for (int i = 0; i < 4; i++)
			//	cameraMsgSent.jointDesiredAngles[i] = jointDesiredAngles[i];
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			//cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1];
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			//for (int i = 0; i < 4; i++) {
			//	cameraMsgSent.jointDesiredAngles[i] = 0.10 * cameraMsgSent.jointDesiredAngles[i] + 0.90 * jointActualAngles[i];
			//	if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] > 2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] + 2.0;
			//	else if (cameraMsgSent.jointDesiredAngles[i] - jointActualAngles[i] < -2.0)
			//		cameraMsgSent.jointDesiredAngles[i] = jointActualAngles[i] - 2.0;
			//}
			cameraMsgSent.clamperDesiredSpace = 0;
		}

		//*******************************************抓取策略*************************************************
		//抓取分数及期望距离计算（以后记得把目标丢失、超限位的判断都放进来）
		if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) <= angErrTolerance1 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) <= angErrTolerance1 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) <= angErrTolerance1) {
			if (grabScore < grabTrigger) {
				grabScore++;
			}
			else {
				if (!hasBeenGrabbing) {
					grabStartTime = clock();
				}
				canGrab = true;
				hasBeenGrabbing = true;
			}
		}
		else if (abs(motorMsgReceived.jointActualAngles[0] - cameraMsgSent.jointDesiredAngles[0]) >= angErrTolerance2 &&
			abs(motorMsgReceived.jointActualAngles[1] - cameraMsgSent.jointDesiredAngles[1]) >= angErrTolerance2 &&
			abs(motorMsgReceived.jointActualAngles[2] - cameraMsgSent.jointDesiredAngles[2]) >= angErrTolerance2) {
			if (grabScore > 0) {
				grabScore--;
			}
			else {
				canGrab = false;
			}
		}
		//cout << "******************************" << endl;
		//cout << "********Grab Score = " << grabScore << "********" << endl;
		//cout << "******************************" << endl;
		if ((1.0 * grabScore) / (1.0 * grabTrigger) > 0.7) {
			relaDepthDesired = relaDepthDesiredMin;
		}
		else {
			relaDepthDesired = relaDepthDesiredMax;
		}
		relaDepthDesired = relaDepthDesiredMax;
		relaDepthDesiredFiltered = 0.95 * relaDepthDesiredFiltered + 0.05 * relaDepthDesired;
		//是否可以开始抓取的判断
		if (canGrab && !IKerror && cameraMsgSent.id >= 200) {
			//标志位置1
			noteStr = "Start Grabing! ";
			state = STATE_GRAB;
			cameraMsgSent.state = state;
		}
		//是否已经抓取够久了
		if (hasBeenGrabbing) {
			////开始抓取后则不再改变关节角度
			//jointDesiredAngles = jointDesiredAngles_last;
			cameraMsgSent.jointDesiredAngles[0] = jointDesiredAngles[0];
			//cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1] - 90;
			cameraMsgSent.jointDesiredAngles[1] = jointDesiredAngles[1];
			cameraMsgSent.jointDesiredAngles[2] = -jointDesiredAngles[2];
			cameraMsgSent.jointDesiredAngles[3] = jointDesiredAngles[3];
			if (!hasBeenHoming && motorMsgReceived.state == 2) {
				state = STATE_HOMING;
				cameraMsgSent.state = state;
				noteStr = "Start Homing! ";
				homingStartTime = clock();
				hasBeenHoming = true;
			}
			if (hasBeenHoming) {
				state = STATE_HOMING;
				cameraMsgSent.state = state;
				noteStr = "Start Homing! ";
				cameraMsgSent.jointDesiredAngles[0] = 0;
				cameraMsgSent.jointDesiredAngles[1] = 0;
			}
			if (hasBeenHoming && clock() - homingStartTime >= 5 * 1000) {
				//cameraMsgSent.jointDesiredAngles[2] = -80;
				cameraMsgSent.jointDesiredAngles[2] = -90;
			}
		}
		cameraMsgSent.state = STATE_NORMAL;
		//if (state == STATE_HOMING) {
		//	cameraMsgSent.jointDesiredAngles[0] = 0;
		//	cameraMsgSent.jointDesiredAngles[1] = 0;
		//	cameraMsgSent.jointDesiredAngles[2] = -80;
		//}

		//*******************************************图像界面输出文字*************************************************
		outText.str("");
		outText << "Relative: X = " << xTarRela << ", Y = " << yTarRela << ", Z = " << zTarRela;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Global: X = " << xTarGlobal << ", Y = " << yTarGlobal << ", Z = " << zTarGlobal;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Solved: q1 = " << jointDesiredAngles[0] << ", q2 = " << jointDesiredAngles[1] << ", q3 = " << jointDesiredAngles[2] << ", q23 = " << q23_last;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Desired: q1 = " << cameraMsgSent.jointDesiredAngles[0] << ", q2 = " << cameraMsgSent.jointDesiredAngles[1] << ", q3 = " << cameraMsgSent.jointDesiredAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		outText.str("");
		outText << "Actual: q1 = " << motorMsgReceived.jointActualAngles[0] << ", q2 = " << motorMsgReceived.jointActualAngles[1] << ", q3 = " << motorMsgReceived.jointActualAngles[2];
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		endTime = clock();

		noteStr = "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " + noteStr;//这部分最多只能22字符了
		for (int i = 0; i < min(56, noteStr.length()); i++) {
			cameraMsgSent.noteStr[i] = noteStr[i];
		}
		cameraMsgSent.noteStr[min(55, noteStr.length())] = '\0';
		cout << "[Time:" + to_string((int)ceil(endTime / 1000.0)) + \
			"s Fps:" + to_string((int)round(1000.0 / (endTime - startTime))) + "] " << endl;
		display(cameraMsgSent);

		outText.str("");
		outText << noteStr;
		putText(VecImg[VecImg.size() - 1], outText.str(), Point(30, 180), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1, 8);

		//*******************************************向规划端传输数据*************************************************
		TcpClient2.sendDoubleMsg({ cameraMsgSent.jointDesiredAngles[0], cameraMsgSent.jointDesiredAngles[1], cameraMsgSent.jointDesiredAngles[2], double(cameraMsgSent.state) });

		imshow("Detect", VecImg[VecImg.size() - 1]);
		waitKey(1);
	}

	return 0;
}

int demoTrack()
{
	MyOptFlowTrack Tracker = MyOptFlowTrack();
	Tracker.init();
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(30.0);
	Mat src, dst;
	clock_t startTime, endTime;
	while (!(src=streamObj.getFrame()).empty()) {
		startTime = clock();
		Tracker.track(src);
		Tracker.drawResult(dst);
		endTime = clock();
		imshow("output", dst);
		waitKey(1);
		cout << "Cost "<< (endTime-startTime) <<" ms" << endl;
	}
	return 0;
}

int demoMatch()
{
	FeatureExtractAndMatch matcher = FeatureExtractAndMatch();
	matcher.init();
	Mat img1, img2, dst;
	img1 = imread("D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-28.jpg");
	img2 = imread("D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-48.jpg");
	matcher.Match(img2, img1);
	//matcher.Sift();
	matcher.drawResult(dst);
	imshow("result", dst);
	waitKey(0);
	return 0;
}

int demoRealTimeMatch()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(30.0);
	Mat refer, input, dst;
	refer = imread("D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-28.jpg");
	FeatureExtractAndMatch matcher = FeatureExtractAndMatch();
	matcher.init(refer);
	clock_t startTime, endTime;
	while (!(input = streamObj.getFrame()).empty())
	{
		startTime = clock();
		matcher.Match(input);
		//matcher.Sift();
		matcher.drawResult(dst);
		endTime = clock();
		imshow("result", dst);
		waitKey(1);
		cout << "Cost " << (endTime - startTime) << " ms" << endl;
	}
	return 0;
}

int demoMatrixOperation()
{
	vector<vector<double>> A = { {1,2,3,4}, {5,6,7,8}, {9,10,11,12} };
	auto B = pinv(A);
	cout << "pinv(A) = " << B << endl;
	//cout << vec2mat(A) << endl;
	vector<vector<double>> U, S, V;
	svd(A, U, S, V);
	cout << "U = " << U << endl;
	cout << "S = " << S << endl;
	cout << "V = " << V << endl;
	return 0;
}

int demoImageJacobian()
{
	vector<double> U = { 0,100,200,300 };
	vector<double> V = { 0,100,200,300 };
	//vector<double> Z = { 50,50,50,50 };
	double z = 0.1;
	ImgJacMatrix J = ImgJacMatrix(U, V, z);
	cout << "图像雅克比：" << endl << J.elements;
	cout << "图像雅克比伪逆：" << endl << pinv(J);
	return 0;
}

int demoIBVS()
{
	MyStreamControl streamObj = MyStreamControl();
	streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	streamObj.setResolution(640, 480);
	streamObj.setFps(30.0);
	//streamObj.setInitParams(enum_StreamType::IMAGE, "D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-28.jpg");
	Mat refer, input, dst;
	refer = imread("D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\ZJU Badge Template 1.jpg");
	FeatureExtractAndMatch matcher = FeatureExtractAndMatch();
	matcher.init(refer, 20);
	IBVSController controller = IBVSController();
	vector<KeyPoint> keyPointsRefer, keyPointsInput;
	vector<double> desiredEndVelocity;
	clock_t startTime, endTime;
	static double vx, vy, vz, wx, wy, wz;
	MyImageProcess processor = MyImageProcess();
	processor.setParams(enum_ProcessorType::NONE_PROCESS, enum_ImagingCondition::CAM4K_IN_AIR_640P);
	double desiredDepth = 0.25;
	double gain = 0.05;
	while (!(input = streamObj.getFrame()).empty())
	{
		startTime = clock();
		matcher.Match(input);
		matcher.Sift();
		matcher.getResult(keyPointsRefer, keyPointsInput);
		controller.init(keyPointsRefer, desiredDepth, gain, processor.cameraMatrix);
		//controller.update(keyPointsInput);
		controller.update(keyPointsInput, desiredDepth);
		desiredEndVelocity = controller.output();
		//matcher.drawResult(dst);
		endTime = clock();
		//imshow("result", dst);
		waitKey(1);
		cout << "Cost " << (endTime - startTime) << " ms" << endl;
		vx = 0.90 * vx + 0.10 * desiredEndVelocity[0];
		vy = 0.90 * vy + 0.10 * desiredEndVelocity[1];
		vz = 0.90 * vz + 0.10 * desiredEndVelocity[2];
		wx = 0.90 * wx + 0.10 * desiredEndVelocity[3] * 180 / 3.1416;
		wy = 0.90 * wy + 0.10 * desiredEndVelocity[4] * 180 / 3.1416;
		wz = 0.90 * wz + 0.10 * desiredEndVelocity[5] * 180 / 3.1416;
		cout << "xdot_d: " << vx<<"\t" << vy << "\t" << vz << "\t" << wx << "\t" << wy << "\t" << wz << endl;
		cout << "=================================================================================================" << endl;
	}
	return 0;
}

int demoPerspective()
{
	MyStreamControl streamObj = MyStreamControl();
	//streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	//streamObj.setResolution(640, 480);
	//streamObj.setFps(30.0);
	streamObj.setInitParams(enum_StreamType::IMAGE, "D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-48.jpg");
	Mat refer, input, dst;
	refer = imread("D:\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\ZJU Badge Template 3.jpg");
	FeatureExtractAndMatch matcher = FeatureExtractAndMatch();
	matcher.init(refer, 200);
	IBVSController controller = IBVSController();
	vector<KeyPoint> keyPointsRefer, keyPointsInput;
	vector<Point2f> srcPoints, dstPoints;
	Mat H;
	vector<Mat> rotMats, transMats, normalMats;
	MyImageProcess processor = MyImageProcess();
	processor.setParams(enum_ProcessorType::NONE_PROCESS, enum_ImagingCondition::CAM4K_IN_AIR_640P);
	clock_t startTime, endTime;
	while (!(input = streamObj.getFrame()).empty())
	{
		startTime = clock();
		matcher.Match(input);
		matcher.Sift();
		matcher.getResult(keyPointsRefer, keyPointsInput);
		srcPoints.resize(0);
		dstPoints.resize(0);
		for (int i = 0; i < keyPointsRefer.size(); i++) {
			srcPoints.push_back(keyPointsRefer[i].pt);
			dstPoints.push_back(keyPointsInput[i].pt);
		}
		H = findHomography(srcPoints, dstPoints);
		decomposeHomographyMat(H, processor.cameraMatrix, rotMats, transMats, normalMats);
		matcher.drawResult(dst);
		imshow("result", dst);
		waitKey(1);
		endTime = clock();
		cout << "Cost " << (endTime - startTime) << " ms" << endl;
		cout << "rotation: " << endl << rotMats[0] << endl;
		cout << "translation: " << endl << transMats[0] << endl;
		cout << "=================================================================================================" << endl;
	}
	return 0;
}

int demoFeatureExtractAndMatch()
{
	MyStreamControl streamObj = MyStreamControl();
	//streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	//streamObj.setResolution(640, 480);
	//streamObj.setFps(30.0);
	streamObj.setInitParams(enum_StreamType::IMAGE, "D:\\科研\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-48.jpg");
	Mat refer, input, dst;
	refer = imread("D:\\科研\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\ZJU Badge Template 3.jpg");
	MyFeatureExtractor extractor(enum_FeatureExtractor::ORB);
	MyFeatureMatcher matcher(enum_FeatureMatcher::BF);

	clock_t startTime, endTime;
	double costTime;
	startTime = clock();
	int detectedPointNum;
	for (int i = 0; i < 100; i++) {
		extractor.setImg(streamObj.getFrame());
		detectedPointNum = extractor.extract();
	}
	endTime = clock();
	costTime = (endTime - startTime) / 100.0;
	cout << "Average Cost Time of Extraction: " << costTime << "ms" << endl;
	cout << detectedPointNum << " key points detected" << endl;
	auto keypoints2 = extractor.getKeypoints();
	auto descriptors2 = extractor.getDescriptors();
	Mat markedImg = extractor.getMarkedImg();
	imshow("Feature Extraction Result", markedImg);

	extractor.setImg(refer);
	extractor.extract();
	auto keypoints1 = extractor.getKeypoints();
	auto descriptors1 = extractor.getDescriptors();
	
	matcher.setTemplate(keypoints1, descriptors1);
	startTime = clock();
	for (int i = 0; i < 100; i++) {
		matcher.setActual(keypoints2, descriptors2);
		matcher.match();
	}
	endTime = clock();
	costTime = (endTime - startTime) / 100.0;
	cout << "Average Cost Time of Matching: " << costTime << "ms" << endl;
	imshow("Feature Match Result", matcher.getMatchedImg(refer, streamObj.getFrame()));

	waitKey(0);
	
	return 0;
}

int demoMLESAC1()
{
	//对象实例化
	MyStreamControl streamObj = MyStreamControl();
	//streamObj.setInitParams(enum_StreamType::CAMERA, "0");
	//streamObj.setResolution(640, 480);
	//streamObj.setFps(30.0);
	//streamObj.setInitParams(enum_StreamType::IMAGE, "D:\\科研\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\2022-10-12 17-23-48.jpg");
	//streamObj.setInitParams(enum_StreamType::IMAGE, "D:\\科研\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\ZJU Badge Template 2.jpg");
	streamObj.setInitParams(enum_StreamType::IMAGE, "C:\\Users\\WDDSC\\Desktop\\workspace\\HPatches\\hpatches-sequences-release\\v_blueprint\\1.ppm");
	Mat refer, input;
	//refer = imread("D:\\科研\\水下机械手\\视觉伺服\\UnderwaterVisualServo1.2.1\\WorkSpace\\ZJU Badge Template 3.jpg");
	refer = imread("C:\\Users\\WDDSC\\Desktop\\workspace\\HPatches\\hpatches-sequences-release\\v_blueprint\\2.ppm");
	input = streamObj.getFrame();
	MyFeatureExtractor extractor(enum_FeatureExtractor::ORB);
	MyFeatureMatcher matcher(enum_FeatureMatcher::HAMMING);
	MyInlierModelEstimator estimator(enum_InlierModelEstimator::MLESAC);
	//提取模板图片特征点
	int detectedPointNum1, detectedPointNum2;
	extractor.setImg(refer);
	detectedPointNum1 = extractor.extract();
	cout << detectedPointNum1 << " key points detected in Template image" << endl;
	auto keypoints1 = extractor.getKeypoints();
	auto descriptors1 = extractor.getDescriptors();
	imshow("Template", extractor.getMarkedImg());

	clock_t startTime = clock();
	//提取实际图片特征点
	extractor.setImg(input);
	detectedPointNum2 = extractor.extract();
	cout << detectedPointNum2 << " key points detected in Actual image" << endl;
	auto keypoints2 = extractor.getKeypoints();
	auto descriptors2 = extractor.getDescriptors();
	imshow("Actual", extractor.getMarkedImg());
	//特征匹配
	shrink2fit(keypoints1, descriptors1, keypoints2, descriptors2);
	matcher.setTemplate(keypoints1, descriptors1);
	matcher.setActual(keypoints2, descriptors2);
	matcher.match();
	imshow("Feature Match Result", matcher.getMatchedImg(refer, streamObj.getFrame()));

	////手动标注匹配正确与否
	//vector<DMatch> matches = matcher.getMatches();
	//for (int i = 0; i < keypoints1.size(); i++) {
	//	int ind1 = matches[i].trainIdx;
	//	int ind2 = matches[i].queryIdx;
	//	matcher.setTemplate({ keypoints1[ind1] }, { Mat(descriptors1, Rect(0,ind1,32,1)) });
	//	matcher.setActual({ keypoints2[ind2] }, { Mat(descriptors2, Rect(0,ind2,32,1)) });
	//	matcher.match();
	//	imshow("Feature Match Result", matcher.getMatchedImg(refer, streamObj.getFrame()));
	//	auto keyVal = waitKey(0);
	//	if (keyVal == 27) break;
	//	else if (keyVal == 'y') {
	//		cout << ind1 << "-th keypoint in template is well matched with " << ind2 << "-th keypoint in actual image" << endl;
	//	}
	//	else if (keyVal == 'n');
	//	else if (keyVal == 8) {
	//		if (i >= 1)
	//			i -= 2;
	//		else
	//			i = -1;
	//	}
	//	else ;
	//}

	//MLESAC
	estimator.set(keypoints1, keypoints2, matcher.getMatches());
	estimator.MLESAC(3, 0.86, 1.0, 100.0, 0.95);

	clock_t endTime = clock();
	cout << "Total Cost Time: " << endTime - startTime << "ms" << endl;

	imshow("MLESAC Inliers", estimator.drawInliers(refer, input));
	imshow("MLESAC Result", estimator.drawResult(refer, input));

	waitKey(0);

	return 0;
}

int demoSearchFile()
{
	string dir = "C:\\Users\\WDDSC\\Desktop\\workspace\\HPatches\\hpatches-sequences-release";
	string fileMode = "*";
	vector<string> subDirs, filePaths;
	searchFile(dir, fileMode, subDirs, filePaths);
	searchFile(subDirs[0], fileMode, subDirs, filePaths);
	return 0;
}

int demoMLESAC2()
{
	string dir = "C:\\Users\\WDDSC\\Desktop\\workspace\\HPatches\\hpatches-sequences-release";
	vector<string> subDirs, filePaths, temp;
	searchFile(dir, "*", subDirs, temp);

	for (int iDir = 0; iDir < subDirs.size(); iDir++) {

		cout << "=====================" << endl;
		cout << iDir << "-th image pair" << endl;
		//读取待匹配图像
		searchFile(subDirs[iDir], "*.ppm", temp, filePaths);
		if (filePaths.size() != 6)
			ReportError("Not enough files are found in this direction");
		string referFilePath = filePaths[0];
		string inputFilePath = filePaths[1];
		MyStreamControl streamObj = MyStreamControl();
		streamObj.setInitParams(enum_StreamType::IMAGE, inputFilePath);
		Mat refer, input;
		refer = imread(referFilePath);
		input = streamObj.getFrame();

		//MLESAC
		MyRefinedMatcher MLESACmatcher;
		MLESACmatcher.setActualImg(input);
		MLESACmatcher.setTemplateImg(refer);
		MLESACmatcher.setParams(3, 0.80, 1.0, 100.0, 0.95);
		MLESACmatcher.match();
		matchResult res = MLESACmatcher.getResult();

		fstream fs;
		const string dataSaveDir = "D:\\其他\\董哥论文\\视觉伺服论文\\一改\\MLESAC实验\\数据\\";
		string fileName = ".txt"
		fs.open(dataSavePath, ios::out);
		res.printResult(cout);

		//获取并展示结果图片
		Mat rawMatchImg, refinedMatchImg;
		const double scale = 0.5;
		rawMatchImg = MLESACmatcher.getRawMatchImg();
		resize(rawMatchImg, rawMatchImg, Size(rawMatchImg.cols * scale, rawMatchImg.rows * scale));
		refinedMatchImg = MLESACmatcher.getRefinedMatchImg();
		resize(refinedMatchImg, refinedMatchImg, Size(refinedMatchImg.cols * scale, refinedMatchImg.rows * scale));
		imshow("Before MLESAC [" + to_string(round(scale * 100)) + "%]", rawMatchImg);
		imshow("After MLESAC [" + to_string(round(scale * 100)) + "%]", refinedMatchImg);

		waitKey(1);
	}
	waitKey(0);
	return 0;
}






//接受simulink轨迹规划,用于测试
int Stream3Test(MyTcpipClient clientPlanner)
{
	int msgSendId = 0;
	while (1)
	{
		msgSendId++;
		cout << "***进入循环***" << endl;
		//接受规划端（clientPlanner）的数据，注意判断缓存区是否有新数据
		vector<double> rcvData(9) ;
		double q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3;
		int len = 0;
		//接收到非空数据
		//len = clientPlanner.receiveCameraMsg();
		//display(clientPlanner.getCameraMsg());
		if (len =  clientPlanner.receiveDoubleMsg(rcvData)) {
			q1 = rcvData[0];
			q2 = rcvData[1];
			q3 = rcvData[2];
			dq1 = rcvData[3];
			dq2 = rcvData[4];
			dq3 = rcvData[5];
			ddq1 = rcvData[6];
			ddq2 = rcvData[7];
			ddq3 = rcvData[8];
			cout << "===========收到的规划端数据============" << endl;
			for (int i = 0; i++; i < 9)
				cout << rcvData[i] << ", ";
			cout << endl;
		}
		//暂无数据可接收
		else {
			continue;
			cout << "!!!!暂无数据可接收!!!!" << endl;
		}
		

		//Sleep((msgSendId * 10) % 219);
		Sleep(1);

	}
	return -1;
}
