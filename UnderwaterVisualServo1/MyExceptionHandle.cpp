#include "MyExceptionHandle.h"

using namespace std;
using namespace cv;
//enum ERR_TYPE { STREAM_CAP_ERR, IMG_PREPROCESS_ERR, IMG_PROCESS_ERR, DATA_TRANS_ERR, DATA_SAVE_ERR, UNKNOWN_ERR };
//enum WARN_TYPE { FUNCTION_UNCOMPLETE_WARN, UNKNOWN_WARN };

MyExceptionHandle::MyExceptionHandle()
{}
MyExceptionHandle::~MyExceptionHandle()
{}
void MyExceptionHandle::ReportError(ERR_TYPE errType, string funcName)
{
	switch (errType) {
	case ERR_TYPE::STREAM_CAP_ERR:
		if (funcName == "")
			cout << "[ERROR] Stream error occurs!" << endl;
		else
			cout << "[ERROR] Stream error occurs in " << funcName << " !" << endl;
		break;
	case ERR_TYPE::IMG_PREPROCESS_ERR:
		if (funcName == "")
			cout << "[ERROR] Image preporcess error occurs!" << endl;
		else
			cout << "[ERROR] Image preporcess error occurs in " << funcName << " !" << endl;
		break;
	case ERR_TYPE::IMG_PROCESS_ERR:
		if (funcName == "")
			cout << "[ERROR] Image porcess error occurs!" << endl;
		else
			cout << "[ERROR] Image porcess error occurs in " << funcName << " !" << endl;
		break;
	case ERR_TYPE::DATA_TRANS_ERR:
		if (funcName == "")
			cout << "[ERROR] Data transmit error occurs!" << endl;
		else
			cout << "[ERROR] Data transmit error occurs in " << funcName << " !" << endl;
		break;
	case ERR_TYPE::DATA_SAVE_ERR:
		if (funcName == "")
			cout << "[ERROR] Data save error occurs!" << endl;
		else
			cout << "[ERROR] Data save error occurs in " << funcName << " !" << endl;
		break;
	case ERR_TYPE::UNKNOWN_ERR:
		if (funcName == "")
			cout << "[ERROR] UNKNOWN error occurs!" << endl;
		else
			cout << "[ERROR] UNKNOWN error occurs in " << funcName << " !" << endl;
		break;

	}
}
void MyExceptionHandle::ReportWarning(WARN_TYPE warnType, string funcName)
{
	switch (warnType) {
	case WARN_TYPE::FUNCTION_UNCOMPLETE_WARN:
		if (funcName == "")
			cout << "[WARNING] Uncomplete function is used!" << endl;
		else
			cout << "[WARNING] Uncomplete function (" << funcName << ") is used!" << endl;
		break;
	case WARN_TYPE::WRONG_PARAM_TYPE_WARN:
		if (funcName == "")
			cout << "[WARNING] Wrong parameter type!" << endl;
		else
			cout << "[WARNING] Wrong parameter type in (" << funcName << ") !" << endl;
		break;
	case WARN_TYPE::UNKNOWN_WARN:
		if (funcName == "")
			cout << "[WARNING] Unknown warning!" << endl;
		else
			cout << "[WARNING] Unknown warning from " << funcName << "!" << endl;
		break;
	}
}

void MyExceptionHandle::ReportError(string errStr)
{
	cout << "[ERROR] " << errStr << endl;
}
void MyExceptionHandle::ReportWarning(string warnStr)
{
	cout << "[WARNING] " << warnStr << endl;
}
//void MyExceptionHandle::ReportError(char errStr[])
//{
//	cout << "[ERROR] " << errStr << endl;
//}
//void MyExceptionHandle::ReportWarning(char warnStr[])
//{
//	cout << "[WARNING] " << warnStr << endl;
//}