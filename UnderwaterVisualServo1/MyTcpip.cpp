#include "MyTcpip.h"

using namespace std;
using namespace cv;

void display(msgCameraTemplate msg)
{
	cout << endl;
	std::cout << "--------------��׼���������֡-------------" << std::endl;
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
	std::cout << "--------------��׼���������֡-------------" << std::endl;
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
	cout << "-----------������-----------" << endl;
	//	1	��ʼ��
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);	//make word,�������Ƶ�WSAStartup���������б�,�ǲ��Ǿ���һ��word��

	//	2	�������������׽���
	//SOCKET serviceSocket
	serviceSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//socket(Э����,socket���ݴ��䷽ʽ,ĳ��Э��)	����Ĭ��Ϊ0,��ʵ����һ����
	if (SOCKET_ERROR == serviceSocket) {
		cout << "�׽��ִ���ʧ��!" << endl;
	}
	else {
		cout << "�׽��ִ����ɹ�!" << endl;
	}

	//	3	���׽���	ָ���󶨵�IP��ַ�Ͷ˿ں�
	sockaddr_in socketAddr;								//һ���󶨵�ַ:��IP��ַ,�ж˿ں�,��Э����
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.S_un.S_addr = inet_addr(tcpAddr);		// IP
	socketAddr.sin_port = htons(post); // �˿ں�

	char flag = 0;
	if (setsockopt(serviceSocket, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
		cout << "�����׽��ֳ���" << endl;
	}

	//REBIND:
	int bRes = ::bind(serviceSocket, (SOCKADDR*)&socketAddr, sizeof(SOCKADDR));	//��ע���һ�����Ҫ����ȫ�ֺ���������ӿڻ���std::bind
	if (SOCKET_ERROR == bRes) {
		cout << "��ʧ��!" << endl;
		//goto REBIND;
	}
	else if (EADDRINUSE == bRes) {
		cout << "��ʧ�ܣ��˿���ʹ�ã�!" << endl;
	}
	else {
		cout << "�󶨳ɹ�!" << endl;
	}

	//	4	����������	
	int lLen = listen(serviceSocket, 5);	//�����ĵڶ�����������:�ܴ�Ŷ��ٸ��ͻ�������,��������̵�ʱ�������Ŷ
	if (SOCKET_ERROR == lLen) {
		cout << "����ʧ��!" << endl;
	}
	else {
		cout << "�����ɹ�!" << endl;
	}

	//	5	��������
	sockaddr_in revClientAddr;
	//SOCKET recvClientSocket = INVALID_SOCKET;	//��ʼ��һ�����ܵĿͻ���socket
	recvClientSocket = INVALID_SOCKET;	//��ʼ��һ�����ܵĿͻ���socket
	int _revSize = sizeof(sockaddr_in);
	recvClientSocket = accept(serviceSocket, (SOCKADDR*)&revClientAddr, &_revSize);
	if (INVALID_SOCKET == recvClientSocket) {
		cout << "����˽�������ʧ��!" << endl;
	}
	else {
		cout << "����˽�������ɹ�!" << endl;
	}

	return 0;
}

int MyTcpipServer::closeServer()
{

	//	7	�ر�socket
	closesocket(recvClientSocket);
	closesocket(serviceSocket);

	//	8	��ֹ
	WSACleanup();

	cout << "������ֹͣ" << endl;
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
		//С��ģʽ�����ֽ��ڵ�λ�����ֽ��ڸߵ�ַ
		//msgSend[msgInd++] = num.num_char[len_double - 1 - i];
		//�ô��ģʽ��������
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
	for (int i = 0; i < 256; i++) this->msgSend[i] = this->msgCamera.char_msg[i];//��֪Ϊ��strcpy������Ч������
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
	//	6	����/���� ����	
	char recvBuf[BUF_SIZE];
	int reLen = recv(recvClientSocket, recvBuf, strlen(recvBuf), 0);
	if (SOCKET_ERROR == reLen) {
		cout << "����˽�������ʧ��" << endl;
		throw - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "����˽��յ����󳤶�(" << reLen << ")������" << endl;
		throw - 2;
	}
	else{
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "����˽��յ� " << reLen / 256 << " ������֡" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgCamera = stdMsgCamera(msgReceive);
		//display(this->msgMotor.msg);
	}
	return 0;
}

int MyTcpipServer::receiveMotorMsg()
{
	const int BUF_SIZE = 4096;
	//	6	����/���� ����	
	char recvBuf[BUF_SIZE];
	int reLen = recv(recvClientSocket, recvBuf, strlen(recvBuf), 0);
	if (SOCKET_ERROR == reLen) {
		cout << "����˽�������ʧ��" << endl;
		throw - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "����˽��յ����󳤶�(" << reLen << ")������" << endl;
		throw - 2;
	}
	else {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "����˽��յ� " << reLen / 256 << " ������֡" << endl;
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
	cout << "-----------�ͻ���-----------" << endl;
	const int BUF_SIZE = 4096;
	//	1	��ʼ��
	WSADATA wsadata;
	WSAStartup(MAKEWORD(2, 2), &wsadata);	//make word,�������Ƶ�WSAStartup���������б�,�ǲ��Ǿ���һ��word��

	//	2	�������������׽���
	//SOCKET serviceSocket
	clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//socket(Э����,socket���ݴ��䷽ʽ,ĳ��Э��)	����Ĭ��Ϊ0,��ʵ����һ����
	if (SOCKET_ERROR == clientSocket) {
		cout << "�׽��ִ���ʧ��!" << endl;
	}
	else {
		cout << "�׽��ִ����ɹ�!" << endl;
	}

	//	3	���׽���	ָ���󶨵�IP��ַ�Ͷ˿ں�
	sockaddr_in socketAddr;								//һ���󶨵�ַ:��IP��ַ,�ж˿ں�,��Э����
	socketAddr.sin_family = AF_INET;
	socketAddr.sin_addr.S_un.S_addr = inet_addr(tcpAddr);		// IP
	socketAddr.sin_port = htons(post); // �˿ں�
	if (connect(clientSocket, (SOCKADDR*)&socketAddr, sizeof(SOCKADDR)) < 0) {
		cout << "����ʧ��!" << endl;
	}
	else{
		cout << "���ӳɹ�!" << endl;
	}

	return 0;
}

int MyTcpipClient::closeClient()
{
	//	7	�ر�socket
	closesocket(clientSocket);

	//	8	��ֹ
	WSACleanup();

	cout << "�ͻ���ֹͣ" << endl;
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
		cout << "�滮����δ����������" << endl;
	}
	else {
		recvBuf[reLen] = '\0';//�ض��ַ�����
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
	//	6	����/���� ����	
	char recvBuf[BUF_SIZE];
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE - 1, 0);
	if (SOCKET_ERROR == reLen) {
		cout << "�ͻ��˽�������ʧ��" << endl;
		throw - 1;
	}
	//else if (reLen % TCP_MSG_SIZE != 0) {
	//	recvBuf[reLen] = '\0';//�ض��ַ�����
	//	cout << "�ͻ��˽��յ����󳤶�(" << reLen << ")������" << endl;
	//	throw - 2;
	//}
	else {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "�ͻ��˽��յ� " << reLen / 256 << " ������֡" << endl;
		for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
		this->msgCamera = stdMsgCamera(msgReceive);
		//display(this->msgCamera.msg);
	}
	return 0;
}

int MyTcpipClient::receiveMotorMsg()
{
	const int BUF_SIZE = 4096;
	//	6	����/���� ����	
	char recvBuf[BUF_SIZE];
	//char* recvBuf;
	//memset(recvBuf, 0, BUF_SIZE);
	//int reLen = recv(clientSocket, &(recvBuf[0]), strlen(recvBuf), 0);
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE-256, 0);
	//cout << reLen << "!!!" << endl;
	if (SOCKET_ERROR == reLen || reLen == 0) {
		cout << "�ͻ��˽�������ʧ��" << endl;
		return - 1;
	}
	else if (reLen % TCP_MSG_SIZE != 0) {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "�ͻ��˽��յ����󳤶�(" << reLen << ")������" << endl;
		return - 2;
	}
	else {
		recvBuf[reLen] = '\0';//�ض��ַ�����
		cout << "�ͻ��˽��յ� " << reLen / 256 << " ������֡" << endl;
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

//��ȡsimulink�˴�����9������
int MyTcpipClient::receiveSimulinkMsg()
{

	const int BUF_SIZE = 4096;
	////	6	����/���� ����	
	char recvBuf[BUF_SIZE];
	////char* recvBuf;
	////memset(recvBuf, 0, BUF_SIZE);
	////int reLen = recv(clientSocket, &(recvBuf[0]), strlen(recvBuf), 0);
	int reLen = recv(clientSocket, recvBuf, BUF_SIZE - 1, 0);
	//if (SOCKET_ERROR == reLen) {
	//	cout << "�ͻ��˽�������ʧ��" << endl;
	//	return -1;
	//}
	//else if (reLen % TCP_MSG_SIZE != 0) {
	//	recvBuf[reLen] = '\0';//�ض��ַ�����
	//	cout << "�ͻ��˽��յ����󳤶�(" << reLen << ")������" << endl;
	//	return -2;
	//}
	//else {
	//	recvBuf[reLen] = '\0';//�ض��ַ�����
	//	cout << "�ͻ��˽��յ� " << reLen / 256 << " ������֡" << endl;
	//	for (int i = 0; i < 256; i++)this->msgReceive[i] = recvBuf[i];
	//	this->msgMotor = stdMsgMotor(msgReceive);
	//	//display(this->msgCamera.msg);
	//}
	return 0;
}

