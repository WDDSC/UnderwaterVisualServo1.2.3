#include "MyBasic.h"
using namespace std;

clock_t getTime()
{
	return clock();
}
double getTimeSec()
{
	return (long)getTime() / 1000.0;
}
//string getDateStr()
//{
//	char str[100] = {0};
//	SYSTEMTIME sys;
//	//sprintf(str, "%4d-%02d-%02d %02d:%02d:%02d %03d 星期%1d\n", sys.wYear, sys.wMonth, sys.wDay,
//	//	sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, sys.wDayOfWeek);
//	sprintf(str, "%4d-%02d-%02d", sys.wYear, sys.wMonth, sys.wDay);
//	string s(str);
//	return s;
//}
//string getTimeStr() {
//	char str[100] = { 0 };
//	SYSTEMTIME sys;
//	//sprintf(str, "%4d-%02d-%02d %02d:%02d:%02d %03d 星期%1d\n", sys.wYear, sys.wMonth, sys.wDay,
//	//	sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, sys.wDayOfWeek);
//	sprintf(str, "%02d:%02d:%02d", sys.wHour, sys.wMinute, sys.wSecond);
//	string s(str);
//	return s;
//}
void ReportError(std::string errInfo)
{
	cout << "[ERROR] " << errInfo << endl;
}
void ReportWarning(std::string warnInfo)
{
	cout << "[WARN] " << warnInfo << endl;
}


std::ostream& operator << (ostream& out, std::vector<double>& numVec)
{
	for (int i = 0; i < numVec.size(); i++) {
		out << numVec[i] << " ";
	}
	out << endl;
	return out;
}

void searchFile(std::string dir, std::string fileMode, std::vector<std::string>& subDirPaths, std::vector<std::string>& filePaths)
{
	subDirPaths.clear();
	filePaths.clear();
	string curr = dir + "\\"+ fileMode;
	CharToWchar(curr.c_str());
	_finddata_t finder;
	intptr_t handle = _findfirst(curr.c_str(), &finder);
	if (handle == -1)
	{
		cout << "Failed to find first file!\n";
		return;
	}

	do
	{
		if (finder.attrib & _A_SUBDIR) {    // 是否是子目录
			if (strcmp(finder.name, ".") != 0 && strcmp(finder.name, "..") != 0) {//并且不为"."或".."
				subDirPaths.push_back(dir + "\\" + string(finder.name));
				cout << finder.name << "\t<dir>\n";
			}
		}
		else {
			filePaths.push_back(dir + "\\" + string(finder.name));
			cout << finder.name << "\t" << finder.size << endl;
		}
	} while (_findnext(handle, &finder) == 0);    // 查找目录中的下一个文件
	cout << "Done!\n";
	_findclose(handle);    // 关闭搜索句柄
}

const wchar_t* CharToWchar(const char* ch)
{
	const size_t len = strlen(ch) + 1;
	wchar_t* wch = new wchar_t[len];
	//mbstowcs(len, wch, ch, len);
	size_t converted = 0;
	mbstowcs_s(&converted, wch, len, ch, _TRUNCATE);
	return wch;
}
const char* WcharToChar(const wchar_t* wch)
{	
	const size_t len = wcslen(wch) + 1;
	char* ch = new char[len];
	//wcstombs(ch, wch, len);
	size_t converted = 0;
	wcstombs_s(&converted, ch, len, wch, _TRUNCATE);
	return ch;
}


//void operator << (ostream& out, double numArr[])
//{
//	for (int i = 0; i < sizeof(numArr) / sizeof(double); i++) {
//		out << numArr[i] << " ";
//	}
//	out << endl;
//}
