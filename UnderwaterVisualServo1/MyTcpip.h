#ifndef MYTCPIP_H
#define MYTCPIP_H

#define _WINSOCK_DEPRECATED_NO_WARNINGS 	
#define _CRT_SECURE_NO_WARNINGS

#define TCP_MSG_SIZE 256

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <WS2tcpip.h>
#include <WinSock2.h>						//һ�������,���ͷ�ļ�λ��windows.h֮ǰ,���ⷢ��ĳЩ����
#include <Windows.h>
#include "MyExceptionHandle.h"
#include "MyBasic.h"
#pragma comment(lib, "ws2_32.lib") 			//��ʾ���� ws2_32.dll	ws2_32.dll��������socket�汾��

#define STATE_TARGETLOSS -1
#define STATE_IKERROR -2
#define STATE_NORMAL 0
#define STATE_GRAB 1
#define STATE_HOMING 2

const int len_double = sizeof(double);
const int len_int = sizeof(int);
typedef union doubletochar_8
{
	double num_double;
	char num_char[len_double];
}doubleMsg;
typedef union inttochar_4
{
	double int_double;
	char int_char[len_int];
}intMsg;

//����˷��͵���Ϣ��ʽ
struct msgCameraTemplate
{
	int length, id, state;//12byte
	int reservedTag = 0;//4byte��reservedTagĬ��Ϊ0
	double jointDesiredAngles[4];//len4, 32byte
	double clamperDesiredSpace;//len1, 8byte
	double targetRelativePose[6];//len6, 48byte
	double targetGlobalPosition[3];//len3, 24byte
	double objectivePointGlobalPosition[3];//len3, 24byte
	double reservedData[6] = {0};//len6, 48byte��Ĭ��Ϊ0
	char noteStr[56] = "";//len56, 56byte��Ĭ��Ϊ'\0'
	msgCameraTemplate(int id, int state, std::vector<double> jointDesiredAngles, double clamperDesiredSpace, \
		std::vector<double> targetRelativePose, std::vector<double> targetGlobalPosition, \
		std::vector<double> objectivePointGlobalPosition, std::string noteStr)
	{
		this->length = 256;
		this->id = id;
		this->state = state;
		//this->reservedTag = 0;//������ǩλ��ʼΪ0
		if (jointDesiredAngles.size() != 4) {
			ReportError("jointDesiredAngles has a wrong size (suggested to be 4)");
			throw - 1;
		}
		for (int i = 0; i < 4; i++) this->jointDesiredAngles[i] = jointDesiredAngles[i];
		this->clamperDesiredSpace = clamperDesiredSpace;
		if (targetRelativePose.size() != 6) {
			ReportError("targetRelativePose has a wrong size (suggested to be 6)");
			throw - 1;
		}
		for (int i = 0; i < 6; i++) this->targetRelativePose[i] = targetRelativePose[i];
		if (targetGlobalPosition.size() != 3) {
			ReportError("targetGlobalPosition has a wrong size (suggested to be 3)");
			throw - 1;
		}
		for (int i = 0; i < 3; i++) this->targetGlobalPosition[i] = targetGlobalPosition[i];
		if (objectivePointGlobalPosition.size() != 3) {
			ReportError("objectivePointGlobalPosition has a wrong size (suggested to be 3)");
			throw - 1;
		}
		for (int i = 0; i < 3; i++) this->objectivePointGlobalPosition[i] = objectivePointGlobalPosition[i];
		//for (int i = 0; i < 6; i++) this->reservedData[i] = 0.0;//��������λ��ʼΪ0
		if (noteStr.length() > 56) {
			ReportError("noteStr has a wrong length (suggested to <= 56)");
			throw - 1;
		}
		//this->noteStr = "";//��ע�ַ�����ʼΪ'\0'
		for (int i = 0; i < noteStr.length(); i++) {
			this->noteStr[i] = noteStr[i];
		}
	}
	msgCameraTemplate()
	{
		*this = msgCameraTemplate(0, 0, { 0,0,0,0 }, 0, { 0, 0, 0, 0, 0, 0 }, { 0,0,0 }, { 0,0,0 }, "");
	}
};
typedef union msgCameraTemplateToChar256
{
	char char_msg[256];
	msgCameraTemplate msg;
	msgCameraTemplateToChar256(msgCameraTemplate msg) {
		this->msg = msg;
	}
	msgCameraTemplateToChar256(char char_msg[256]) {
		for (int i = 0; i < 256; i++) this->char_msg[i] = char_msg[i];
	}
	msgCameraTemplateToChar256() {
		for (int i = 0; i < 256; i++) this->char_msg[i] = '\0';
		this->msg = msgCameraTemplate();
	}
}stdMsgCamera;
void display(msgCameraTemplate msg);

//����˷��͵���Ϣ��ʽ
struct msgMotorTemplate
{
	int length, id, state;//12byte
	int reservedTag = 0;//4byte��reservedTagĬ��Ϊ0
	double jointActualAngles[4];//len4, 32byte
	double clamperActualSpace;//len1, 8byte
	double motorVelocity[5];//len5, 40byte
	double reservedData[13] = { 0 };//len13, 104byte��Ĭ��Ϊ0
	char noteStr[56] = "";//len56, 56byte��Ĭ��Ϊ'\0'
	msgMotorTemplate(int id, int state, std::vector<double> jointActualAngles, double clamperActualSpace, \
		std::vector<double> motorVelocity, std::string noteStr)
	{
		this->length = 256;
		this->id = id;
		this->state = state;
		//this->reservedTag = 0;//������ǩλ��ʼΪ0
		if (jointActualAngles.size() != 4) {
			ReportError("jointActualAngles has a wrong size (suggested to be 4)");
			throw - 1;
		}
		for (int i = 0; i < 4; i++) this->jointActualAngles[i] = jointActualAngles[i];
		this->clamperActualSpace = clamperActualSpace;
		if (motorVelocity.size() != 5) {
			ReportError("motorVelocity has a wrong size (suggested to be 5)");
			throw - 1;
		}
		//for (int i = 0; i < 13; i++) this->reservedData[i] = 0.0;//��������λ��ʼΪ0
		if (noteStr.length() > 56) {
			ReportError("noteStr has a wrong length (suggested to <= 56)");
			throw - 1;
		}
		//this->noteStr = "";//��ע�ַ�����ʼΪ'\0'
		for (int i = 0; i < noteStr.length(); i++) {
			this->noteStr[i] = noteStr[i];
		}
	}
	msgMotorTemplate()
	{
		*this = msgMotorTemplate(0, 0, { 0,0,0,0 }, 0, { 0, 0, 0, 0, 0}, "");
	}
};
typedef union msgMotorTemplateToChar256
{
	char char_msg[256];
	msgMotorTemplate msg;
	msgMotorTemplateToChar256(msgMotorTemplate msg) {
		this->msg = msg;
	}
	msgMotorTemplateToChar256(char char_msg[256]) {
		for (int i = 0; i < 256; i++) this->char_msg[i] = char_msg[i];
	}
	msgMotorTemplateToChar256() {
		for (int i = 0; i < 256; i++) this->char_msg[i] = '\0';
		this->msg = msgMotorTemplate();
	}
}stdMsgMotor;
void display(msgMotorTemplate msg);

class MyTcpipServer
{
private:
	SOCKET serviceSocket;
	SOCKET recvClientSocket;
	const char* tcpAddr;
	int post;
	stdMsgCamera msgCamera;
	stdMsgMotor msgMotor;
	char msgReceive[TCP_MSG_SIZE];
	int msgInd;

public:
	char msgSend[TCP_MSG_SIZE];
	MyTcpipServer(const char* tcpAddr, int post);
	~MyTcpipServer();
	int buildServer();
	int closeServer();
	void flushMsgSend();
	void addMsg(double doubleToSend);
	void addMsg(std::string strToSend);
	void setMsg(int tagToSend, std::vector<double> numArrToSend, std::string strToSend);
	void setMsg(msgCameraTemplate msg);
	int sendMotorMsg();
	int sendCameraMsg();
	int receiveCameraMsg();
	int receiveMotorMsg();
	msgCameraTemplate getCameraMsgReceived();
	msgMotorTemplate getMotorMsgReceived();
};

class MyTcpipClient
{
private:

	const char* tcpAddr;
	int post;
	stdMsgCamera msgCamera;
	stdMsgMotor msgMotor;
	char msgReceive[TCP_MSG_SIZE];
	int msgInd;

public:
	SOCKET clientSocket;
	char msgSend[TCP_MSG_SIZE];
	MyTcpipClient(const char* tcpAddr, int post);
	~MyTcpipClient();
	int buildClient();
	int closeClient();
	void flushMsgSend();
	void setMsg(msgMotorTemplate msg);
	void setMsg(msgCameraTemplate msg);
	int sendDoubleMsg(std::vector<double> data);//ֻ�к�Simiulink�滮��ͨ�Ų��õ�������3���ؽڽǶ�double,һ����λ
	int receiveDoubleMsg(std::vector<double>& data);//ֻ�к�Simiulink�滮��ͨ�Ų��õ�������3���ؽڵ�q,dq,ddq��9��double����һ����λ
	int sendMsg();
	int flushRcvBuf();
	int receiveCameraMsg();
	int receiveMotorMsg();
	msgCameraTemplate getCameraMsg();
	msgMotorTemplate getMotorMsg();
	int receiveSimulinkMsg();
};

#endif //!MYEXPERIMENT_H