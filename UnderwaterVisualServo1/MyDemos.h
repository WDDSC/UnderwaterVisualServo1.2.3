#ifndef MYDEMOS_H
#define MYDEMOS_H

//#include <sstream>
#include "MyExceptionHandle.h"
#include "MyMath.h"
#include "MyTcpip.h"
#include "MyImageProcess.h"
#include "MyStreamControl.h"
#include "MyRoboticKinematics.h"
#include "MyVisualServo.h"
#include "MyTargetTrack.h"
#include "MyIdentify.h"

int demoImageUndistort();
int demoPreview();
int demoDetect();
int demoEstimatePosition();
int demoUndistortDetect();
int demoEstimatePose();
int demoEstimateCubePose();
int demoKinematics();
int demoDifferentialKinematics();
int demoUniversalIK();
int demoUnderwaterRecover();
int demoRealTimeTraj();
int demoPointTransTest();
int demoTcpServerSend_1();
int demoTcpServerSend_2();
int demoTcpServerSend_Linux();
int demoTcpClientSend_Linux();
int demoTcpServer();
int demoTcpClient();

int demoVisualServo_OnlyCamera();

int demoVisualServo_Bilateral();

int demoVisualServo_Bilateral_Grab();

//2023-03-17 还没写完
int upStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, std::mutex& motorClientLock,
	std::mutex& plannerClientLock, std::mutex& msgLock, msgCameraTemplate& msg);
//接受电机端数据+视觉伺服+发送给simulink轨迹规划，频率30hz左右
int upStream(MyTcpipClient clientMotor, MyTcpipClient clientPlanner);

//接受simulink轨迹规划结果+数据包打包+发送给电机端，频率500hz左右
int downStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, std::mutex& motorClientLock,
	std::mutex& plannerClientLock, std::mutex& msgLock, msgCameraTemplate& msg);

int dualThread();

int Stream3Test(MyTcpipClient clientPlanner);


int demoVisualServo_Trilateral();

int demoVisualServo_Trilateral2_Grabbing();
//2023-03-26 三边通信的跟随控制（ARC版）
int demoVisualServo_Trilateral2_Tracking();

int demoTrack();

int demoMatch();

int demoRealTimeMatch();

int demoMatrixOperation();

int demoImageJacobian();

int demoIBVS();

int demoPerspective();

int demoFeatureExtractAndMatch();

int demoMLESAC1();

int demoSearchFile();

int demoMLESAC2();

#endif // !MYDEMOS_H
