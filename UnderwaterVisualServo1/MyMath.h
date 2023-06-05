#ifndef MYMATH_H
#define MYMATH_H

#include <iostream>
//#include <opencv2/core/types.hpp>
//#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include "MyBasic.h"

struct RectangularCoordinatePoint;
typedef RectangularCoordinatePoint MyPoint2D;
struct PolarCoordinatePoint;
typedef PolarCoordinatePoint MyPoint2DP;
struct CartesianSpacePoint;
typedef CartesianSpacePoint MyPoint3D;
struct JointSpacePoint;
typedef JointSpacePoint MyJointVec;

/*平面点*/
struct RectangularCoordinatePoint
{
	double x, y;
	RectangularCoordinatePoint() { x = 0, y = 0; }
	RectangularCoordinatePoint(double x0, double y0) { x = x0, y = y0; }
	RectangularCoordinatePoint(std::vector<double>p) { x = p[0], y = p[1]; }
	operator std::vector<double>() { return { this->x, this->y}; }
	//operator MyPoint2DP() { return MyPoint2DP(sqrt(x * x + y * y), atan2(x, y)); }
	//operator MyPoint2DP();
};
MyPoint2D operator + (MyPoint2D A, MyPoint2D B);
MyPoint2D operator - (MyPoint2D A, MyPoint2D B);
MyPoint2D operator * (double k, MyPoint2D A);
MyPoint2D operator * (MyPoint2D A, double k);
MyPoint2D operator / (MyPoint2D A, double k);
bool operator == (MyPoint2D A, MyPoint2D B);
MyPoint2DP Rect2Polar(MyPoint2D A);
MyPoint2D unify(MyPoint2D A);

/*极坐标点*/
struct PolarCoordinatePoint
{
	double rho, theta;
	PolarCoordinatePoint() { rho = 0, theta = 0; }
	PolarCoordinatePoint(double rho0, double theta0) { rho = rho0, theta = theta0; }
	PolarCoordinatePoint(std::vector<double>p) { rho = p[0], theta = p[1]; }
	operator std::vector<double>() { return { this->rho, this->theta }; }
	//operator MyPoint2D() { return MyPoint2D(rho * cos(theta), rho * sin(theta)); }
	//operator MyPoint2D();
};
MyPoint2DP operator + (MyPoint2DP A, MyPoint2DP B);
MyPoint2DP operator * (double k, MyPoint2DP A);
MyPoint2DP operator * (MyPoint2DP A, double k);
MyPoint2DP operator / (MyPoint2DP A, double k);
bool operator == (MyPoint2DP A, MyPoint2DP B);
MyPoint2D Polar2Rect(MyPoint2DP A);
MyPoint2DP unify(MyPoint2DP A);

/*空间点*/
struct CartesianSpacePoint
{
	double x, y, z;
	CartesianSpacePoint() { x = 0, y = 0, z = 0; }
	CartesianSpacePoint(double x0, double y0, double z0) { x = x0, y = y0, z = z0; }
	CartesianSpacePoint(std::vector<double> p) { x = p[0], y = p[1], z = p[2]; }
	operator std::vector<double>() { return { this->x, this->y, this->z }; }
};
const MyPoint3D NONEPOINT3D = MyPoint3D(NAN, NAN, NAN);
MyPoint3D operator + (MyPoint3D A, MyPoint3D B);
MyPoint3D operator - (MyPoint3D A, MyPoint3D B);
MyPoint3D operator * (double k, MyPoint3D A);
MyPoint3D operator * (MyPoint3D A, double k);
MyPoint3D operator / (MyPoint3D A, double k);
bool operator == (MyPoint3D A, MyPoint3D B);
MyPoint3D unify(MyPoint3D A);

/*关节角度向量*/
struct JointSpacePoint
{
	double q1, q2, q3, q4;
	JointSpacePoint() { q1 = 0, q2 = 0, q3 = 0, q4 = 0; }
	JointSpacePoint(double q10, double q20, double q30, double q40) { q1 = q10, q2 = q20, q3 = q30, q4 = q40; }
	JointSpacePoint(std::vector<double> q) { q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; }
	operator std::vector<double>() { return { this->q1, this->q2, this->q3, this->q4 }; }
};
MyJointVec operator + (MyJointVec A, MyJointVec B);
MyJointVec operator - (MyJointVec A, MyJointVec B);
MyJointVec operator * (double k, MyJointVec A);
MyJointVec operator * (MyJointVec A, double k);
MyJointVec operator / (MyJointVec A, double k);
bool operator == (MyJointVec A, MyJointVec B);
MyJointVec unify(MyJointVec A);


/*统计学函数*/
double myMean(std::vector<double> data);
int myCompareDouble(void const* a, void const* b);
double myMedian(std::vector<double> data);
double myNorm(std::vector<double> data);
double myNorm(std::vector<std::vector<double>> A);
//int myMedian(std::vector<int> data);
double myCalcSD(std::vector<double> data);//计算标准差
std::vector<double> myMultiMedian(std::vector<double> data, int k);

/*PoseMatrix类*/
//基于double[4][4]数据类型，定义一系列齐次位姿矩阵的操作
class PoseMatrix
{
private:

public:
	double elements[4][4];//齐次位姿矩阵数据
	PoseMatrix();
	PoseMatrix(double elements[4][4]);
	PoseMatrix(double x, double y, double z, double roll, double pitch, double yaw);
	~PoseMatrix();
	PoseMatrix IdentityMat();//设置为单位阵
	void AssignElements(double elements[4][4]);//赋值
	bool IsValid();//判断矩阵元素是否合法
	void Display();//展示位姿矩阵
	std::vector<std::vector<double>> ToVec();
	std::vector<double> getPosition();
	std::vector<double> getOrientation();
	std::vector<double> getPose();
	void EliminateSmallValue();//将<1e-5的值变为0（防止反三角函数计算误差太大）
};
PoseMatrix operator + (PoseMatrix A, PoseMatrix B);//矩阵加法（好像没用到）
PoseMatrix operator - (PoseMatrix A, PoseMatrix B);//矩阵减法（好像没用到）
PoseMatrix operator * (PoseMatrix A, PoseMatrix B);//矩阵乘法
PoseMatrix operator * (double a, PoseMatrix B);//标量-矩阵乘法
PoseMatrix operator * (PoseMatrix A, double b);
std::ostream& operator << (std::ostream& co, PoseMatrix A);//矩阵输出
PoseMatrix inv(PoseMatrix A);
double det(PoseMatrix A, int excludeRow = -1, int excludeCol = -1);
PoseMatrix TransMat(double x, double y, double z);
PoseMatrix RotMat(double r, double p, double y);
PoseMatrix RotMat(std::vector<double> rvec);


/*2维转3维函数*/
MyPoint3D Convert2DTo3D(MyPoint2D A);
MyPoint3D Convert2DTo3D(MyPoint2DP A);
MyPoint3D Convert2DTo3D(MyPoint2D A, PoseMatrix T);
MyPoint3D Convert2DTo3D(MyPoint2DP A, PoseMatrix T);


/*ImgJacMatrix*/
class ImgJacMatrix
{
public:
	std::vector<std::vector<double>> elements;
	ImgJacMatrix();
	ImgJacMatrix(double u, double v, double z);
	ImgJacMatrix(std::vector<double> U, std::vector<double> V, double z);
	ImgJacMatrix(std::vector<double> U, std::vector<double> V, std::vector<double> Z);
	~ImgJacMatrix();
	double cond();
};
std::vector<std::vector<double>> pinv(ImgJacMatrix J);

/*矩阵通用算符*/
//双层vector通过堆叠行向量形成矩阵，外层size行数，内层size为列数
std::vector<std::vector<double>> mat2vec(cv::Mat& mat);
cv::Mat vec2mat(std::vector<std::vector<double>> A);
void resize(std::vector<std::vector<double>>& A, int rows, int cols);
std::ostream& operator << (std::ostream& co, std::vector<std::vector<double>> A);//矩阵输出
std::vector<std::vector<double>> operator + (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);//矩阵加法
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);//矩阵乘法
std::vector<std::vector<double>> operator * (double a, std::vector<std::vector<double>> B);//标量-矩阵乘法
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, double b);//标量-矩阵乘法2
std::vector<double> operator * (std::vector<std::vector<double>> A, std::vector<double> b);//向量-矩阵乘法
std::vector<double> operator * (std::vector<double> a, std::vector<std::vector<double>> B);//向量-矩阵乘法2
std::vector<std::vector<double>> transpose(std::vector<std::vector<double>> mat);
double det(std::vector<std::vector<double>> mat, int excludeRow = -1, int excludeCol = -1);
std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat);
std::vector<std::vector<double>> pinv(std::vector<std::vector<double>> mat);
std::vector<std::vector<double>> pinv_damp(std::vector<std::vector<double>> mat, double damp = pow(10,-8));
int svd(std::vector<std::vector<double>> mat, std::vector<std::vector<double>>& U, std::vector<std::vector<double>>& Sigma,//注意sigma是特征值数列而非矩阵
	std::vector<std::vector<double>>& V);//注意是V不是V^T
std::vector<std::vector<double>> truncate(std::vector<std::vector<double>> A, std::vector<int> rows, std::vector<int> cols);
std::vector<std::vector<double>> eye(int size);
std::vector<std::vector<double>> zeros(int rows, int cols);
std::vector<std::vector<double>> ones(int rows, int cols);

/*向量通用算符*/
std::vector<std::vector<double>> arr2vec(std::vector<double> a);//向量转矩阵，默认行向量
std::vector<double> vec2arr(std::vector<std::vector<double>> A);//矩阵转向量，默认逐列元素pushback进arr
std::vector<double> operator + (std::vector<double> a, std::vector<double> b);//向量-向量加法
std::vector<double> operator - (std::vector<double> a, std::vector<double> b);//向量-向量减法
std::vector<double> operator * (double k, std::vector<double> a);//标量-向量乘法
std::vector<double> operator * (std::vector<double> a, double k);//标量-向量乘法
std::vector<double> dotProduct(std::vector<double> a, std::vector<double> b);//向量-向量乘法（逐元素）
std::vector<double> dotDivide(std::vector<double> a, std::vector<double> b);//向量-向量除法（逐元素）
double cosine(std::vector<double> a, std::vector<double> b);//余弦相似度
std::vector<double> truncate(std::vector<double> a, std::vector<int> cols);
std::vector<double> unify(std::vector<double> a);

#endif