#include "MyTcpip.h"

using namespace std;
using namespace cv;

void display(msgCameraTemplate msg)
{
	cout << endl;
	std::cout << "--------------标准相机端数据帧-------------" << std::endl;
	std::cout << "Message length: " << msg.length << std::endl;
	std::cout << "Message ID: " << msg.id << std::endl;
	std::cout << "Vision program state: " << msg.state << std::endl;
	std::cout << "Joint desired angles: ";
	for (int i = 0; i < 4; i++)cout << msg.jointDesiredAngles[i] << " ";
	cout << std::endl;
	std::cout << "Clamper desired space: " << msg.clamperDesiredSpace << std::endl;
	std::cout << "Target pose relative to camera: ";
	for (int i = 0; i < 6; i++)cout << msg.targetRelativePose[i] << " ";
	cout << std::endl;
	std::cout << "Target global position: ";
	for (int i = 0; i < 3; i++)cout << msg.targetGlobalPosition[i] << " ";
	cout << std::endl;
	std::cout << "Objective point positioin: ";
	for (int i = 0; i < 3; i++)cout << msg.objectivePointGlobalPosition[i] << " ";
	cout << std::endl;
	std::cout << "Note: " << msg.noteStr << std::endl;
	std::cout << "-------------------------------------------" << std::endl;
}

void display(msgMotorTemplate msg)
{
	cout << endl;
	std::cout << "--------------标准电机端数据帧-------------" << std::endl;
	std::cout << "Message length: " << msg.length << std::endl;
	std::cout << "Message ID: " << msg.id << std::endl;
	std::cout << "Control program state: " << msg.state << std::endl;
	std::cout << "Joint actual angles: ";
	for (int i = 0; i < 4; i++)cout << msg.jointActualAngles[i] << " ";
	cout << std::endl;
	std::cout << "Clamper actual space: " << msg.clamperActualSpace << std::endl;
	std::cout << "Motor velocities: ";
	for (int i = 0; i < 5; i++)cout << msg.motorVelocity[i] << " ";
	cout << std::endl;
	std::cout << "Note: " << msg.noteStr << std::endl;
	std::cout << "-------------------------------------------" << std::endl;
}

MyTcpipServer::MyTcpipServer(const char* tcpAddr, int post)
{
	this->tcpAddr = tcpAddr;
	this->post = post;
	flushMsgSend();
}

MyTcpipServer::~MyTcpipServer()
{
	closeServer();
};

int MyTcpipServer::buildServer()
{
	cout << "-----------服务器-----------" << endl;
	//	1	初始化
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);	//make word,你把鼠标移到WSAStartup看看参数列表,是不是就是一个word啊

	//	2	创建服务器的套接字
	//SOCKET serviceSocket
	serviceSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//socket(协议族,socket数据传输方式,某个协议)	我们默认为0,其实就是一个宏
	if (SOCKET_ERROR == serviceSocket) {
		cout << "套接字创建失败!" << endl;
	}
	else {
		cout << "套接字创建成功!" << endl;
	}

	//	3	绑定套接字	指定绑定的IP地址和端口号
	sockaddr_in socketAddr;								//一个绑定地址:有IP地址,有端口号,有协议族
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.S_un.S_addr = inet_addr(tcpAddr);		// IP
	socketAddr.sin_port = htons(post); // 端口号

	char flag = 0;
	if (setsockopt(serviceSocket, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
		cout << "设置套接字出错" << endl;
	}

	//REBIND:
	int bRes = ::bind(serviceSocket, (SOCKADDR*)&socketAddr, sizeof(SOCKADDR));	//绑定注意的一点就是要调用全局函数，否则接口会变成std::bind
	if (SOCKET_ERROR == bRes) {
		cout << "绑定失败!" << endl;
		//goto REBIND;
	}
	else if (EADDRINUSE == bRes) {
		cout << "绑定失败（端口已使用）!" << endl;
	}
	else {
		cout << "绑定成功!" << endl;
	}

	//	4	服务器监听	
	int lLen = listen(serviceSocket, 5);	//监听的第二个参数就是:能存放多少个客户端请求,到并发编程的时候很有用哦
	if (SOCKET_ERROR == lLen) {
		cout << "监听失败!" << endl;
	}
	else {
		cout << "监听成功!" << endl;
	}

	//	5	接受请求
	sockaddr_in revClientAddr;
	//SOCKET recvClientSocket = INVALID_SOCKET;	//初始化一个接受的客户端socket
	recvClientSocket = INVALID_SOCKET;	//初始化一个接受的客户端socket
	int _revSize = sizeof(sockaddr_in);
	recvClientSocket = accept(serviceSocket, (SOCKADDR*)&revClientAddr, &_revSize);
	if (INVALID_SOCKET == recvClientSocket) {
		cout << "服务端接受请求失败!" << endl;
	}
	else {
		cout << "服务端接受请求成功!" << endl;
	}

	return 0;
}

int MyTcpipServer::closeServer()
{

	//	7	关闭socket
	closesocket(recvClientSocket);
	closesocket(serviceSocket);

	//	8	终止
	WSACleanup();

	cout << "服务器停止" << endl;
	//cin.get();

	return 0;
}

void MyTcpipServer::flushMsgSend()
{
	msgInd = 0;
	for (int i = 0; i < TCP_MSG_SIZE; i++) {
		msgSend[i] = 0;
	}
}

void MyTcpipServer::addMsg(double doubleToSend)
{
	//const int BUF_SIZE = 4096;
	doubleMsg num;
	//char send_arr[len_double];
	num.num_double = doubleToSend;
	for (int i = 0; i < len_double; i++) {
		//小端模式：低字节在低位，高字节在高地址
		//msgSend[msgInd++] = num.num_char[len_double - 1 - i];
		//用大端模式就完事了
		msgSend[msgInd++] = num.num_char[i];
	}
}

void MyTcpipServer::addMsg(string strToSend)
{
	for (int i = 0; i < strToSend.size(); i++) {
		msgSend[msgInd++] = strToSend[i];
	}
}

void MyTcpipServer::setMsg(int tagToSend, std::vector<double> numArrToSend, std::string strToSend)
{
	flushMsgSend();
	addMsg(tagToSend);
	if (numArrToSend.size() >= 20) {
		ReportWarning("Size of the numArrToSend is too large");
	}
	int i = 0;
	while (i < 20) {
		if (i < numArrToSend.size()) {
			addMsg(numArrToSend[i]);
		}
		else {
			addMsg(-1.0);
		}
		i++;
	}
	addMsg(strToSend);
	if (msgInd >= TCP_MSG_SIZE) {
		ReportError("Message to send overflows!");
	}
}

void MyTcpipServer::setMsg(msgCameraTemplate msg)
{
	flushMsgSend();
	this->msgCamera = stdMsgCamera(msg);
	for (int i = 0; i < 256; i++) this->msgSend[i] = this->msgCamera.char_msg[i];//不知为何strcpy拷贝无效。。。
}

int MyTcpipServer::sendMotorMsg()
{
	return send(recvClientSocket, &msgSend[0], TCP_MSG_SIZE, 0);
}

int MyTcpipServer::sendCameraMsg()
{
	return 0;
}

int MyTcpipServer::receiveCameraMsg()
{
	const int BUF_SIZE = 4096;
	//	6	发送/接受 数据	
	char recvBuf[BUF_SIZE];
	int reLen = recv(recvClientSocket, recvBuf, strlen(recvBuf), 0);
	if (SOCKET_ERROR == reLen) {
		cout << "服务端接收数据失败" << endl;
		throw - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "服务端接收到错误长度(" << reLen << ")的数据" << endl;
		throw - 2;
	}
	else{
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "服务端接收到 " << reLen / 256 << " 个数据帧" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgCamera = stdMsgCamera(msgReceive);
		//display(this->msgMotor.msg);
	}
	return 0;
}

int MyTcpipServer::receiveMotorMsg()
{
	const int BUF_SIZE = 4096;
	//	6	发送/接受 数据	
	char recvBuf[BUF_SIZE];
	int reLen = recv(recvClientSocket, recvBuf, strlen(recvBuf), 0);
	if (SOCKET_ERROR == reLen) {
		cout << "服务端接收数据失败" << endl;
		throw - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "服务端接收到错误长度(" << reLen << ")的数据" << endl;
		throw - 2;
	}
	else {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "服务端接收到 " << reLen / 256 << " 个数据帧" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgMotor = stdMsgMotor(msgReceive);
		//display(this->msgMotor.msg);
	}
	return 0;
}

msgCameraTemplate MyTcpipServer::getCameraMsgReceived()
{
	return this->msgCamera.msg;
}

msgMotorTemplate MyTcpipServer::getMotorMsgReceived()
{
	return this->msgMotor.msg;
}


MyTcpipClient::MyTcpipClient(const char* tcpAddr, int post)
{
	this->tcpAddr = tcpAddr;
	this->post = post;
	flushMsgSend();
}

MyTcpipClient::~MyTcpipClient()
{
	closeClient();
};

int MyTcpipClient::buildClient()
{
	cout << "-----------客户端-----------" << endl;
	const int BUF_SIZE = 4096;
	//	1	初始化
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);	//make word,你把鼠标移到WSAStartup看看参数列表,是不是就是一个word啊

	//	2	创建服务器的套接字
	//SOCKET serviceSocket
	clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//socket(协议族,socket数据传输方式,某个协议)	我们默认为0,其实就是一个宏
	if (SOCKET_ERROR == clientSocket) {
		cout << "套接字创建失败!" << endl;
	}
	else {
		cout << "套接字创建成功!" << endl;
	}

	//	3	绑定套接字	指定绑定的IP地址和端口号
	sockaddr_in socketAddr;								//一个绑定地址:有IP地址,有端口号,有协议族
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.S_un.S_addr = inet_addr(tcpAddr);		// IP
	socketAddr.sin_port = htons(post); // 端口号
	if (connect(clientSocket, (SOCKADDR*)&socketAddr, sizeof(SOCKADDR)) < 0) {
		cout << "连接失败!" << endl;
	}
	else{
		cout << "连接成功!" << endl;
	}

	return 0;
}

int MyTcpipClient::closeClient()
{
	//	7	关闭socket
	closesocket(clientSocket);

	//	8	终止
	WSACleanup();

	cout << "客户端停止" << endl;
	//cin.get();

	return 0;
}

void MyTcpipClient::flushMsgSend()
{
	msgInd = 0;
	for (int i = 0; i < TCP_MSG_SIZE; i++) {
		msgSend[i] = 0;
	}
}

void MyTcpipClient::setMsg(msgMotorTemplate msg)
{
	flushMsgSend();
	this->msgMotor = stdMsgMotor(msg);
	for (int i = 0; i < 256; i++) this->msgSend[i] = this->msgMotor.char_msg[i];
}

void MyTcpipClient::setMsg(msgCameraTemplate msg)
{
	flushMsgSend();
	this->msgCamera = stdMsgCamera(msg);
	for (int i = 0; i < 256; i++) this->msgSend[i] = this->msgCamera.char_msg[i];
}

int MyTcpipClient::sendDoubleMsg(vector<double> data)
{
	flushMsgSend();
	char data_char[TCP_MSG_SIZE];
	doubleMsg data_element;
	for (int i = 0; i < data.size(); i++) {
		data_element.num_double = data[i];
		for (int j = 0; j < 8; j++) {
			//data_char[8 * i + j] = data_element.num_char[j];
			data_char[8 * i + 7 - j] = data_element.num_char[j];
		}
	}
	return send(clientSocket, &data_char[0], 8* data.size(), 0);
}

int MyTcpipClient::receiveDoubleMsg(vector<double>& data)
{
	const int rcvlen = 9;
	doubleMsg data_element;
	const int BUF_SIZE = 73;
	char recvBuf[BUF_SIZE];
	//int reLen = recv(clientSocket, recvBuf, BUF_SIZE - 1, 0);
	int reLen = recv(clientSocket, recvBuf, rcvlen*8, 0);
	if (SOCKET_ERROR == reLen || reLen == 0) {
		cout << "规划端尚未返回新数据" << endl;
	}
	else {
		recvBuf[reLen] = '\0';//截断字符数组
		data.resize(rcvlen);
		for (int i = 0; i < rcvlen; i++) {
			for (int j = 0; j < 8; j++) {
				data_element.num_char[j] = recvBuf[8 * i + 7 - j];
				//data_element.num_char[j] = recvBuf[8 * i + j];
			}
			data[i] = data_element.num_double;
		}	
	}
	return reLen;
}

int MyTcpipClient::sendMsg()
{
	return send(clientSocket, &msgSend[0], TCP_MSG_SIZE, 0);
}

int MyTcpipClient::flushRcvBuf()
{
	const int BUF_SIZE = 1024;
	char recvBuf[BUF_SIZE];
	int i = 0;
	int rcvLen = 0;
	int totalRcvLen = 0;
	while (1) {
		rcvLen = recv(clientSocket, recvBuf, BUF_SIZE, 0);
		if (rcvLen == BUF_SIZE) {
			i++;
		}
		totalRcvLen += rcvLen;
		break;
	}
	return totalRcvLen;
}

int MyTcpipClient::receiveCameraMsg()
{
	const int BUF_SIZE = 4096;
	//	6	发送/接受 数据	
	char recvBuf[BUF_SIZE];
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE - 1, 0);
	if (SOCKET_ERROR == reLen) {
		cout << "客户端接收数据失败" << endl;
		throw - 1;
	}
	//else if (reLen % TCP_MSG_SIZE != 0) {
	//	recvBuf[reLen] = '\0';//截断字符数组
	//	cout << "客户端接收到错误长度(" << reLen << ")的数据" << endl;
	//	throw - 2;
	//}
	else {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "客户端接收到 " << reLen / 256 << " 个数据帧" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgCamera = stdMsgCamera(msgReceive);
		//display(this->msgCamera.msg);
	}
	return 0;
}

int MyTcpipClient::receiveMotorMsg()
{
	const int BUF_SIZE = 4096;
	//	6	发送/接受 数据	
	char recvBuf[BUF_SIZE];
	//char* recvBuf;
	//memset(recvBuf, 0, BUF_SIZE);
	//int reLen = recv(clientSocket, &(recvBuf[0]), strlen(recvBuf), 0);
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE-256, 0);
	//cout << reLen << "!!!" << endl;
	if (SOCKET_ERROR == reLen || reLen == 0) {
		cout << "客户端接收数据失败" << endl;
		return - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "客户端接收到错误长度(" << reLen << ")的数据" << endl;
		return - 2;
	}
	else {
		recvBuf[reLen] = '\0';//截断字符数组
		cout << "客户端接收到 " << reLen / 256 << " 个数据帧" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgMotor = stdMsgMotor(msgReceive);
		//display(this->msgCamera.msg);
	}
	return reLen;
}

msgCameraTemplate MyTcpipClient::getCameraMsg()
{
	return this->msgCamera.msg;
}

msgMotorTemplate MyTcpipClient::getMotorMsg()
{
	return this->msgMotor.msg;
}

//读取simulink端传来的9个数据
int MyTcpipClient::receiveSimulinkMsg()
{

	const int BUF_SIZE = 4096;
	////	6	发送/接受 数据	
	char recvBuf[BUF_SIZE];
	////char* recvBuf;
	////memset(recvBuf, 0, BUF_SIZE);
	////int reLen = recv(clientSocket, &(recvBuf[0]), strlen(recvBuf), 0);
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE - 1, 0);
	//if (SOCKET_ERROR == reLen) {
	//	cout << "客户端接收数据失败" << endl;
	//	return -1;
	//}
	//else if (reLen % TCP_MSG_SIZE != 0) {
	//	recvBuf[reLen] = '\0';//截断字符数组
	//	cout << "客户端接收到错误长度(" << reLen << ")的数据" << endl;
	//	return -2;
	//}
	//else {
	//	recvBuf[reLen] = '\0';//截断字符数组
	//	cout << "客户端接收到 " << reLen / 256 << " 个数据帧" << endl;
	//	for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
	//	this->msgMotor = stdMsgMotor(msgReceive);
	//	//display(this->msgCamera.msg);
	//}
	return 0;
}

