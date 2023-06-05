#ifndef MYBASIC_H
#define MYBASIC_H

#include <iostream>
#include <fstream>
#include <exception>
#include <vector>
#include <thread>
#include <mutex>
#include <random>
#include <cstring>
#include <io.h>
#include <time.h>
#include <windows.h>

enum class enum_ErrType {
	DEFAULT, INPUT_INVALID, PARAM_UNEXPECTED, RUN_CRASH, CALC_CRASH, IN_DEVELOPMENT, PARAM_UNASSIGNED, UNKNOWN_ERR
};
enum class enum_WarnType {
	DEFAULT, INPUT_INVALID, PARAM_UNEXPECTED, CALC_WARN, IN_DEVELOPMENT, UNKNOWN_WARN
};

//std::fstream logFile;
clock_t getTime();
double getTimeSec();
string getDateStr();
//string getTimeStr();

void ReportError(std::string errInfo);
void ReportWarning(std::string warnInfo);

std::ostream& operator << (std::ostream& out, std::vector<double>& numVec);

//friend void operator >> (ostream& out, std::vector<double>& numVec);

//friend void operator >> (ostream& out, double numArr[]);

template <typename arr>
arr randomSelectFrom(arr data, int N);
template <typename arr>
arr srandomSelectFrom(arr data, int N, unsigned int seed);

template<typename arr>
inline arr randomSelectFrom(arr data, int N)
{
	std::default_random_engine randEngine;
	int size = data.size();
	for (int n = 0; n < N; n++) {
		swap(data[n], data[randEngine() % size]);
	}
	arr ans(data.begin(), data.begin() + N);
	return ans;
}

template <typename arr>
inline arr srandomSelectFrom(arr data, int N, unsigned int seed)
{
	std::default_random_engine randEngine;
	int size = data.size();
	for (int n = 0; n < N; n++) {
		//srand(seed * (n + 1));
		randEngine.seed(seed*100 + n*137);
		swap(data[n], data[randEngine() % size]);
	}
	arr ans(data.begin(), data.begin() + N);
	return ans;
}


void searchFile(std::string dir, std::string fileMode, std::vector<std::string>& subDirPaths, std::vector<std::string>& filePaths);
const wchar_t* CharToWchar(const char* ch);

#endif //!MYBASIC_H