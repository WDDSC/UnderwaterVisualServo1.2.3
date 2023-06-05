#ifndef MYEXCEPTIONHANDLE_H
#define MYEXCEPTIONHANDLE_H

#include <iostream>
#include <exception>
#include <opencv2/opencv.hpp>

enum class ERR_TYPE { STREAM_CAP_ERR, IMG_PREPROCESS_ERR, IMG_PROCESS_ERR, DATA_TRANS_ERR, DATA_SAVE_ERR, UNKNOWN_ERR };
enum class WARN_TYPE{ FUNCTION_UNCOMPLETE_WARN, WRONG_PARAM_TYPE_WARN, UNKNOWN_WARN };

class MyExceptionHandle
{
private:
	//ERR_TYPE errType;
	//WARN_TYPE warnType;

protected:

public:
	MyExceptionHandle();
	~MyExceptionHandle();
	void ReportError(ERR_TYPE errType, std::string funcName = "");
	void ReportWarning(WARN_TYPE warnType, std::string funcName = "");
	void ReportError(std::string errStr);
	void ReportWarning(std::string warnStr);
	//void ReportError(char errStr[]);
	//void ReportWarning(char warnStr[]);
};

#endif