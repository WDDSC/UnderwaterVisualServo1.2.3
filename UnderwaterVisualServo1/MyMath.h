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

/*ƽ���*/
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

/*�������*/
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

/*�ռ��*/
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

/*�ؽڽǶ�����*/
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


/*ͳ��ѧ����*/
double myMean(std::vector<double> data);
int myCompareDouble(void const* a, void const* b);
double myMedian(std::vector<double> data);
double myNorm(std::vector<double> data);
double myNorm(std::vector<std::vector<double>> A);
//int myMedian(std::vector<int> data);
double myCalcSD(std::vector<double> data);//�����׼��
std::vector<double> myMultiMedian(std::vector<double> data, int k);

/*PoseMatrix��*/
//����double[4][4]�������ͣ�����һϵ�����λ�˾���Ĳ���
class PoseMatrix
{
private:

public:
	double elements[4][4];//���λ�˾�������
	PoseMatrix();
	PoseMatrix(double elements[4][4]);
	PoseMatrix(double x, double y, double z, double roll, double pitch, double yaw);
	~PoseMatrix();
	PoseMatrix IdentityMat();//����Ϊ��λ��
	void AssignElements(double elements[4][4]);//��ֵ
	bool IsValid();//�жϾ���Ԫ���Ƿ�Ϸ�
	void Display();//չʾλ�˾���
	std::vector<std::vector<double>> ToVec();
	std::vector<double> getPosition();
	std::vector<double> getOrientation();
	std::vector<double> getPose();
	void EliminateSmallValue();//��<1e-5��ֵ��Ϊ0����ֹ�����Ǻ����������̫��
};
PoseMatrix operator + (PoseMatrix A, PoseMatrix B);//����ӷ�������û�õ���
PoseMatrix operator - (PoseMatrix A, PoseMatrix B);//�������������û�õ���
PoseMatrix operator * (PoseMatrix A, PoseMatrix B);//����˷�
PoseMatrix operator * (double a, PoseMatrix B);//����-����˷�
PoseMatrix operator * (PoseMatrix A, double b);
std::ostream& operator << (std::ostream& co, PoseMatrix A);//�������
PoseMatrix inv(PoseMatrix A);
double det(PoseMatrix A, int excludeRow = -1, int excludeCol = -1);
PoseMatrix TransMat(double x, double y, double z);
PoseMatrix RotMat(double r, double p, double y);
PoseMatrix RotMat(std::vector<double> rvec);


/*2άת3ά����*/
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

/*����ͨ�����*/
//˫��vectorͨ���ѵ��������γɾ������size�������ڲ�sizeΪ����
std::vector<std::vector<double>> mat2vec(cv::Mat& mat);
cv::Mat vec2mat(std::vector<std::vector<double>> A);
void resize(std::vector<std::vector<double>>& A, int rows, int cols);
std::ostream& operator << (std::ostream& co, std::vector<std::vector<double>> A);//�������
std::vector<std::vector<double>> operator + (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);//����ӷ�
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B);//����˷�
std::vector<std::vector<double>> operator * (double a, std::vector<std::vector<double>> B);//����-����˷�
std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, double b);//����-����˷�2
std::vector<double> operator * (std::vector<std::vector<double>> A, std::vector<double> b);//����-����˷�
std::vector<double> operator * (std::vector<double> a, std::vector<std::vector<double>> B);//����-����˷�2
std::vector<std::vector<double>> transpose(std::vector<std::vector<double>> mat);
double det(std::vector<std::vector<double>> mat, int excludeRow = -1, int excludeCol = -1);
std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat);
std::vector<std::vector<double>> pinv(std::vector<std::vector<double>> mat);
std::vector<std::vector<double>> pinv_damp(std::vector<std::vector<double>> mat, double damp = pow(10,-8));
int svd(std::vector<std::vector<double>> mat, std::vector<std::vector<double>>& U, std::vector<std::vector<double>>& Sigma,//ע��sigma������ֵ���ж��Ǿ���
	std::vector<std::vector<double>>& V);//ע����V����V^T
std::vector<std::vector<double>> truncate(std::vector<std::vector<double>> A, std::vector<int> rows, std::vector<int> cols);
std::vector<std::vector<double>> eye(int size);
std::vector<std::vector<double>> zeros(int rows, int cols);
std::vector<std::vector<double>> ones(int rows, int cols);

/*����ͨ�����*/
std::vector<std::vector<double>> arr2vec(std::vector<double> a);//����ת����Ĭ��������
std::vector<double> vec2arr(std::vector<std::vector<double>> A);//����ת������Ĭ������Ԫ��pushback��arr
std::vector<double> operator + (std::vector<double> a, std::vector<double> b);//����-�����ӷ�
std::vector<double> operator - (std::vector<double> a, std::vector<double> b);//����-��������
std::vector<double> operator * (double k, std::vector<double> a);//����-�����˷�
std::vector<double> operator * (std::vector<double> a, double k);//����-�����˷�
std::vector<double> dotProduct(std::vector<double> a, std::vector<double> b);//����-�����˷�����Ԫ�أ�
std::vector<double> dotDivide(std::vector<double> a, std::vector<double> b);//����-������������Ԫ�أ�
double cosine(std::vector<double> a, std::vector<double> b);//�������ƶ�
std::vector<double> truncate(std::vector<double> a, std::vector<int> cols);
std::vector<double> unify(std::vector<double> a);

#endif