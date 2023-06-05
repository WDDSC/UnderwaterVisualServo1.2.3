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

//2023-03-17 ��ûд��
int upStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, std::mutex& motorClientLock,
	std::mutex& plannerClientLock, std::mutex& msgLock, msgCameraTemplate& msg);
//���ܵ��������+�Ӿ��ŷ�+���͸�simulink�켣�滮��Ƶ��30hz����
int upStream(MyTcpipClient clientMotor, MyTcpipClient clientPlanner);

//����simulink�켣�滮���+���ݰ����+���͸�����ˣ�Ƶ��500hz����
int downStreamTest(MyTcpipClient clientMotor, MyTcpipClient clientPlanner, std::mutex& motorClientLock,
	std::mutex& plannerClientLock, std::mutex& msgLock, msgCameraTemplate& msg);

int dualThread();

int Stream3Test(MyTcpipClient clientPlanner);


int demoVisualServo_Trilateral();

int demoVisualServo_Trilateral2_Grabbing();
//2023-03-26 ����ͨ�ŵĸ�����ƣ�ARC�棩
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
