#include "MyMath.h"
using namespace std;
using namespace cv;

MyPoint2D operator + (MyPoint2D A, MyPoint2D B) { return MyPoint2D(A.x + B.x, A.y + B.y); }
MyPoint2D operator - (MyPoint2D A, MyPoint2D B) { return MyPoint2D(A.x - B.x, A.y - B.y); }
MyPoint2D operator * (double k, MyPoint2D A) { return MyPoint2D(k * A.x, k * A.y); }
MyPoint2D operator * (MyPoint2D A, double k) { return MyPoint2D(k * A.x, k * A.y); }
MyPoint2D operator / (MyPoint2D A, double k) { return MyPoint2D(A.x / k, A.y / k); }
bool operator == (MyPoint2D A, MyPoint2D B) { return (A.x == B.x) && (A.y == B.y); }
MyPoint2DP Rect2Polar(MyPoint2D A) { return MyPoint2DP(sqrt(A.x * A.x + A.y * A.y), atan2(A.x, A.y)); }
MyPoint2D unify(MyPoint2D A) { return A / myNorm(A); }

MyPoint2DP operator + (MyPoint2DP A, MyPoint2DP B) { return Rect2Polar(Polar2Rect(A) + Polar2Rect(B)); }
MyPoint2DP operator * (double k, MyPoint2DP A) { return MyPoint2DP(k * A.rho, A.theta); }
MyPoint2DP operator * (MyPoint2DP A, double k) { return MyPoint2DP(k * A.rho, A.theta); }
MyPoint2DP operator / (MyPoint2DP A, double k) { return MyPoint2DP(A.rho / k, A.theta); }
bool operator == (MyPoint2DP A, MyPoint2DP B) { return (A.rho == B.rho) && (A.theta == B.theta); }
MyPoint2D Polar2Rect(MyPoint2DP A){ return MyPoint2D(A.rho * cos(A.theta), A.rho * sin(A.theta)); }
MyPoint2DP unify(MyPoint2DP A) { return MyPoint2DP(1.0, A.theta); }

MyPoint3D operator + (MyPoint3D A, MyPoint3D B) { return MyPoint3D(A.x + B.x, A.y + B.y, A.z + B.z); }
MyPoint3D operator - (MyPoint3D A, MyPoint3D B) { return MyPoint3D(A.x - B.x, A.y - B.y, A.z - B.z); }
MyPoint3D operator * (double k, MyPoint3D A) { return MyPoint3D(k * A.x, k * A.y, k * A.z); }
MyPoint3D operator * (MyPoint3D A, double k) { return MyPoint3D(k * A.x, k * A.y, k * A.z); }
MyPoint3D operator / (MyPoint3D A, double k) { return MyPoint3D(A.x / k, A.y / k, A.z / k); }
bool operator == (MyPoint3D A, MyPoint3D B) { return (A.x == B.x) && (A.y == B.y) && (A.z == B.z); }
MyPoint3D unify(MyPoint3D A) { return A / myNorm(A); }

/*2维转3维函数*/
MyPoint3D Convert2DTo3D(MyPoint2D A)
{
	return MyPoint3D(A.x, A.y, 0.0);
}
MyPoint3D Convert2DTo3D(MyPoint2DP A)
{
	return Convert2DTo3D(Polar2Rect(A));
}
MyPoint3D Convert2DTo3D(MyPoint2D A, PoseMatrix T)
{
	return (T * TransMat(A.x, A.y, 0.0)).getPosition();
}
MyPoint3D Convert2DTo3D(MyPoint2DP A, PoseMatrix T)
{
	return (T * TransMat(Polar2Rect(A).x, Polar2Rect(A).y, 0.0)).getPosition();
}

MyJointVec operator + (MyJointVec A, MyJointVec B) { return MyJointVec(A.q1 + B.q1, A.q2 + B.q2, A.q3 + B.q3, A.q4 + B.q4); }
MyJointVec operator - (MyJointVec A, MyJointVec B) { return MyJointVec(A.q1 - B.q1, A.q2 - B.q2, A.q3 - B.q3, A.q4 - B.q4); }
MyJointVec operator * (double k, MyJointVec A) { return MyJointVec(k * A.q1, k * A.q2, k * A.q3, k * A.q4); }
MyJointVec operator * (MyJointVec A, double k) { return MyJointVec(k * A.q1, k * A.q2, k * A.q3, k * A.q4); }
MyJointVec operator / (MyJointVec A, double k) { return MyJointVec(A.q1 / k, A.q2 / k, A.q3 / k, A.q4 / k); }
bool operator == (MyJointVec A, MyJointVec B) { return (A.q1 == B.q1) && (A.q2 == B.q2) && (A.q3 == B.q3) && (A.q4 == B.q4); }
MyJointVec unify(MyJointVec A) { return A / myNorm(A); }

/*统计学函数*/
double myNorm(std::vector<double> data)
{
	double ans = 0.0;
	for (int i = 0; i < data.size(); i++) {
		ans += data[i] * data[i];
	}
	return sqrt(ans);
}
double myNorm(std::vector<std::vector<double>> A)
{
	return myNorm(vec2arr(A));
}
double myMean(std::vector<double> data)
{
	double answer = 0;
	for (int i = 0; i < data.size(); i++) {
		answer += data[i];
	}
	answer /= double(data.size());
	return answer;
}
int myCompareDouble(void const* a, void const* b)
{
	return int(*((double*)a) * 10000) - int(*((double*)b) * 10000);
}
double myMedian(std::vector<double> data)
{
	double* dataPtr = data.data();
	qsort(dataPtr, data.size(), sizeof(double), myCompareDouble);
	return dataPtr[data.size() / 2];
}
//int myMedian(std::vector<int> data)
double myCalcSD(std::vector<double> data)
{
	if (data.size() == 1) return 0;
	double meanVal = myMean(data);
	double stdDeviation = 0;
	for (int i = 0; i < data.size(); i++) {
		stdDeviation += (data[i] - meanVal) * (data[i] - meanVal);
	}
	stdDeviation /= data.size()-1;
	stdDeviation = sqrt(stdDeviation);
	return stdDeviation;
}
std::vector<double> myMultiMedian(std::vector<double> data, int k)
{
	k = k > data.size() ? data.size() : k;
	double* dataPtr = data.data();
	qsort(dataPtr, data.size(), sizeof(double), myCompareDouble);
	std::vector<double> medianData;
	for (int i = -(k+1)/2; i < k/2; i++) {
		medianData.push_back(dataPtr[data.size() / 2 + i]);
	}
	return medianData;
}



/*PoseMatrix类*/
//基于double[4][4]数据类型，定义一系列齐次位姿矩阵的操作
PoseMatrix::PoseMatrix()
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			this->elements[i][j] = 0;
}
PoseMatrix::PoseMatrix(double elements[4][4])
{
	this->AssignElements(elements);
}
PoseMatrix::PoseMatrix(double x, double y, double z, double roll, double pitch, double yaw)
{
	*this = TransMat(x, y, z) * RotMat(roll, pitch, yaw);
}
PoseMatrix::~PoseMatrix() {}
PoseMatrix PoseMatrix::IdentityMat()
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			if (i == j)elements[i][j] = 1;
			else elements[i][j] = 0;
		}
	return *this;
}
void PoseMatrix::AssignElements(double elements[4][4])
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			this->elements[i][j] = elements[i][j];
}
bool PoseMatrix::IsValid()
{
	if (elements[3][3] != 1) return false;
	if (elements[3][0] != 0 || elements[3][1] != 0 || elements[3][2] != 0) return false;
	double temp = elements[0][0] * elements[0][0] + elements[1][0] * elements[1][0] + elements[2][0] * elements[2][0];
	if (temp < 0.999 || temp>1.001) return false;
	temp = elements[0][1] * elements[0][1] + elements[1][1] * elements[1][1] + elements[2][1] * elements[2][1];
	if (temp < 0.999 || temp>1.001) return false;
	temp = elements[0][2] * elements[0][2] + elements[1][2] * elements[1][2] + elements[2][2] * elements[2][2];
	if (temp < 0.999 || temp>1.001) return false;
	return true;
}
void PoseMatrix::EliminateSmallValue()
{
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (abs(elements[i][j]) < 0.00001) elements[i][j] = 0;
		}
	}
}
void PoseMatrix::Display()
{
	std::cout << "PoseMatrix value:" << std::endl;
	std::cout << *this;
}
std::vector<std::vector<double>> PoseMatrix::ToVec()
{
	std::vector<std::vector<double>> mat;
	for (int i = 0; i < 4; i++) {
		mat.push_back({});
		for (int j = 0; j < 4; j++) {
			mat[i].push_back(elements[i][j]);
		}
	}
	return mat;
}
std::vector<double> PoseMatrix::getPosition()
{
	return { elements[0][3], elements[1][3] , elements[2][3] };
}
std::vector<double> PoseMatrix::getOrientation()
{
	ReportError("Function \"getOrientation\" is uncomplete!");
	throw enum_ErrType::IN_DEVELOPMENT;
	return {};
}
std::vector<double> PoseMatrix::getPose()
{
	ReportError("Function \"getPose\" is uncomplete!");
	throw enum_ErrType::IN_DEVELOPMENT;
	return {};
}

//PoseMatrix运算
PoseMatrix operator + (PoseMatrix A, PoseMatrix B)
{
	PoseMatrix C = PoseMatrix();
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			C.elements[i][j] = A.elements[i][j] + B.elements[i][j];
	return C;
}
PoseMatrix operator - (PoseMatrix A, PoseMatrix B)
{
	PoseMatrix C = PoseMatrix();
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			C.elements[i][j] = A.elements[i][j] - B.elements[i][j];
	return C;
}
PoseMatrix operator * (PoseMatrix A, PoseMatrix B)
{
	PoseMatrix C = PoseMatrix();
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				C.elements[i][j] += A.elements[i][k] * B.elements[k][j];
			}
		}
	}
	return C;
}
PoseMatrix operator * (double a, PoseMatrix B)
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			B.elements[i][j] *= a;
	return B;
}
PoseMatrix operator * (PoseMatrix A, double b)
{
	return b * A;
}
PoseMatrix inv(PoseMatrix A)
{
	std::vector<std::vector<double>> mat = A.ToVec();
	double matDet = det(mat);
	if (matDet == 0) throw;
	PoseMatrix invMat = PoseMatrix();
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if ((i + j) % 2 == 0)
				invMat.elements[j][i] = (det(mat, i, j) / matDet);
			else
				invMat.elements[j][i] = (-det(mat, i, j) / matDet);
		}
	}
	return invMat;
}
double det(PoseMatrix A, int excludeRow, int excludeCol)
{
	std::vector<std::vector<double>> mat = A.ToVec();
	return det(mat, excludeRow, excludeCol);
}
std::ostream& operator << (std::ostream& co, PoseMatrix A)
{
	co.precision(2);
	co.flags(std::ios::fixed);
	for (int i = 0; i < 4; i++) {
		co << "|\t";
		for (int j = 0; j < 4; j++) {
			co << A.elements[i][j] << "\t\t";
		}
		co << "|";
		co << std::endl;
	}
	return co;
}

//平移、旋转矩阵
PoseMatrix TransMat(double x, double y, double z)
{
	double matVal[4][4] = { {1,0,0,x},{0,1,0,y},{0,0,1,z},{0,0,0,1} };
	return PoseMatrix(matVal);
}
PoseMatrix RotMat(double r, double p, double y)
{
	double matRr[4][4] = { {1,0,0,0}, {0,cos(r),-sin(r),0},{0,sin(r),cos(r),0},{0,0,0,1} };
	double matRp[4][4] = { {cos(p),0,sin(p),0}, {0,1,0,0},{-sin(p),0,cos(p),0},{0,0,0,1} };
	double matRy[4][4] = { {cos(y),-sin(y),0,0}, {sin(y),cos(y),0,0},{0,0,1,0},{0,0,0,1} };
	return PoseMatrix(matRy) * PoseMatrix(matRp) * PoseMatrix(matRr);
}
PoseMatrix RotMat(std::vector<double> rvec)
{
	double theta = myNorm(rvec);
	double tmp[4][4] = { {0,-rvec[2],rvec[1], 0},{rvec[2],0,-rvec[0],0},{-rvec[1],rvec[0],0,0},{0,0,0,0} };
	PoseMatrix A = 1.0/theta * PoseMatrix(tmp);
	//cout << "theta = " << theta << ", rvec = " << rvec[0] << " " << rvec[1] << " " << rvec[2] << endl;
	//cout << A << endl;
	//cout << PoseMatrix().IdentityMat() + A * sin(theta) + A * A * (1 - cos(theta)) << endl;
	return PoseMatrix().IdentityMat() + A * sin(theta) + A * A * (1 - cos(theta));
}


/*ImageJacobian*/
//std::vector<double[6]> elements;
ImgJacMatrix::ImgJacMatrix() {};
ImgJacMatrix::ImgJacMatrix(double u, double v, double z)
{
	const double fx = 500.0;
	const double fy = 500.0;
	elements = {};
	elements.push_back({ -fx / z, 0, u / z, u * v / fy, -(fx + u * u / fx), v });
	elements.push_back({ 0, -fy / z, v / z, (fy + v * v / fy), -u * v / fx, -u });
}
ImgJacMatrix::ImgJacMatrix(std::vector<double> U, std::vector<double> V, double z)
{
	const double fx = 500.0;
	const double fy = 500.0;
	elements = {};
	for (int i = 0; i < U.size(); i++) {
		double u = U[i];
		double v = V[i];
		elements.push_back({ -fx / z, 0, u / z, u * v / fy, -(fx + u * u / fx), v });
		elements.push_back({ 0, -fy / z, v / z, (fy + v * v / fy), -u * v / fx, -u });
	}
}
ImgJacMatrix::ImgJacMatrix(std::vector<double> U, std::vector<double> V, std::vector<double> Z)
{
	const double fx = 500.0;
	const double fy = 500.0;
	//double fx = 584.0546 / 1024.0 * 640.0;
	//double fy = 580.7337 / 1024.0 * 640.0;
	elements = {};
	for (int i = 0; i < U.size(); i++) {
		double u = U[i];
		double v = V[i];
		double z = Z[i];
		elements.push_back({ -fx / z, 0, u / z, u * v / fy, -(fx + u * u / fx), v });
		elements.push_back({ 0, -fy / z, v / z, (fy + v * v / fy), -u * v / fx, -u });
	}
}
ImgJacMatrix::~ImgJacMatrix() {}
double ImgJacMatrix::cond()
{
	//vector<vector<double>> A = elements * transpose(elements);
	vector<vector<double>> A = elements;
	return myNorm(A) * myNorm(pinv(A));
}
;
std::vector<std::vector<double>> pinv(ImgJacMatrix J)
{
	return pinv(J.elements);
}

/*通用矩阵算符*/
std::vector<std::vector<double>> mat2vec(cv::Mat& mat)
{
	vector<vector<double>> A;
	A.resize(mat.rows);
	for (int i = 0; i < A.size(); i++) A[i].resize(mat.cols);
	//Mat src = Mat_<float>(mat.size(), mat[0].size());
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			A[i][j] = mat.at<double>(i, j);
		}
	}
	return A;
}

cv::Mat vec2mat(std::vector<std::vector<double>> A)
{
	Mat mat = Mat(A.size(), A[0].size(), CV_64FC1);
	//Mat src = Mat_<float>(mat.size(), mat[0].size());
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			mat.at<double>(i, j) = A[i][j];
		}
	}
	mat.convertTo(mat, CV_64FC1);
	return mat;
}

void resize(std::vector<std::vector<double>>& A, int rows, int cols)
{
	A.resize(rows);
	for (int i = 0; i < rows; i++) {
		A[i].resize(cols);
	}
}

std::ostream& operator << (std::ostream& co, std::vector<std::vector<double>> A)
{
	co.precision(2);
	co.flags(std::ios::fixed);
	co << endl;
	for (int i = 0; i < A.size(); i++) {
		co << "|\t";
		for (int j = 0; j < A[0].size(); j++) {
			co << A[i][j] << "\t";
		}
		co << "|";
		co << std::endl;
	}
	return co;
}

std::vector<std::vector<double>> operator + (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B)
{
	if (A.size() != B.size() || A[0].size() != B[0].size()) {
		throw - 1;
	}
	vector<vector<double>> C;
	resize(C, A.size(), A[0].size());
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < A[0].size(); j++) {
			C[i][j] = A[i][j] + B[i][j];
		}
	}
	return C;
}

std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, std::vector<std::vector<double>> B)
{
	if (A[0].size() != B.size()) throw - 1;
	vector<vector<double>> C;
	C.resize(A.size());
	for (int i = 0; i < C.size(); i++) {
		C[i].resize(B[0].size());
	}
	for (int i = 0; i < A.size(); i++) {
		for (int j = 0; j < B[0].size(); j++) {
			for (int k = 0; k < A[0].size(); k++) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	return C;
}

std::vector<std::vector<double>> operator * (double a, std::vector<std::vector<double>> B)
{
	for (int i = 0; i < B.size(); i++)
		for (int j = 0; j < B[0].size(); j++)
			B[i][j] *= a;
	return B;
}

std::vector<std::vector<double>> operator * (std::vector<std::vector<double>> A, double b)
{
	return b * A;
}

std::vector<double> operator * (std::vector<std::vector<double>> A, std::vector<double> b)
{
	if (A[0].size() != b.size()) throw - 1;
	//vector<vector<double>> B = {};
	//for (int i = 0; i < b.size(); i++) {
	//	B.push_back({ b[i] });
	//}
	vector<vector<double>> B = transpose({ b });
	vector<vector<double>> C = A * B;
	vector<double> c = {};
	for (int i = 0; i < C.size(); i++) {
		c.push_back(C[i][0]);
	}
	return c;
}

std::vector<double> operator * (std::vector<double> a, std::vector<std::vector<double>> B)
{
	if (a.size() != B.size()) throw - 1;
	vector<vector<double>> A = {};
	A.push_back(a);
	vector<vector<double>> C = A * B;
	vector<double> c = C[0];
	return c;
}

std::vector<std::vector<double>> transpose(std::vector<std::vector<double>> mat)
{
	vector<vector<double>> matT;
	matT.resize(mat[0].size());
	for (int i = 0; i < matT.size(); i++) {
		matT[i].resize(mat.size());
	}
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat[0].size(); j++) {
			matT[j][i] = mat[i][j];
		}
	}
	return matT;
}

double det(std::vector<std::vector<double>> mat, int excludeRow, int excludeCol)
{
	std::vector<std::vector<double>> excludedMat;
	int exMatDim = 0;
	for (int i = 0; i < mat.size(); i++) {
		if (i == excludeRow) continue;
		excludedMat.push_back({});
		for (int j = 0; j < mat.size(); j++) {
			if (j == excludeCol) continue;
			excludedMat[exMatDim].push_back(mat[i][j]);
		}
		exMatDim++;
	}
	if (exMatDim == 1) return excludedMat[0][0];
	double result = 0;
	for (int j = 0; j < exMatDim; j++) {
		if (j % 2 == 0)
			result += excludedMat[0][j] * det(excludedMat, 0, j);
		else
			result -= excludedMat[0][j] * det(excludedMat, 0, j);
	}
	return result;
}

std::vector<std::vector<double>> inv(std::vector<std::vector<double>> mat)
{
	if (mat.size() != mat[0].size()) throw -1;
	double matDet = det(mat);
	if (matDet == 0) throw -1;
	vector<vector<double>> invMat(mat);
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat.size(); j++) {
			if ((i + j) % 2 == 0)
				invMat[j][i] = (det(mat, i, j) / matDet);
			else
				invMat[j][i] = (-det(mat, i, j) / matDet);
		}
	}
	return invMat;
}

std::vector<std::vector<double>> pinv(std::vector<std::vector<double>> mat)
{
	//用SVD保证不满秩也通用
	vector<vector<double>> U, invS, Sigma, V;
	svd(mat, U, Sigma, V);
	resize(invS, mat[0].size(), mat.size());
	for (int i = 0; i < min(mat.size(), mat[0].size()); i++) {
		if (Sigma[i][0] == 0.0) invS[i][i] = 0.0;
		else invS[i][i] = 1.0 / Sigma[i][0];
	}
	return V * invS * transpose(U);
}

std::vector<std::vector<double>> pinv_damp(std::vector<std::vector<double>> mat, double damp)
{
	if (mat.size() > mat[0].size()) {
		return inv(transpose(mat) * mat + damp * eye(mat[0].size())) * transpose(mat);
	}
	else {
		return transpose(mat) * inv(mat * transpose(mat) + damp * eye(mat.size()));
	}
}

int svd(std::vector<std::vector<double>> mat, std::vector<std::vector<double>>& U, std::vector<std::vector<double>>& Sigma, std::vector<std::vector<double>>& V)
{
	Mat src = vec2mat(mat);
	Mat w, u, vt;
	SVD::compute(src, w, u, vt, SVD::Flags::FULL_UV);
	//resize(Sigma, mat.size(), mat[0].size());
	//for (int i = 0; i < min(mat.size(), mat[0].size()); i++) {
	//	Sigma[i][i] = w.at<double>(i);
	//}
	Sigma = mat2vec(w);
	U = mat2vec(u);
	V = transpose(mat2vec(vt));
	return 0;
}

std::vector<std::vector<double>> truncate(std::vector<std::vector<double>> A, std::vector<int> rows, std::vector<int> cols)
{
	vector<vector<double>> A_trunc;
	if (rows.empty()) {
		for (int i = 0; i < A.size(); i++) {
			rows.push_back(i);
		}
	}
	if (cols.empty()) {
		for (int i = 0; i < A[0].size(); i++) {
			cols.push_back(i);
		}
	}
	resize(A_trunc, rows.size(), cols.size());
	for (int irow = 0; irow < rows.size(); irow++) {
		for (int icol = 0; icol < cols.size(); icol++) {
			A_trunc[irow][icol] = A[rows[irow]][cols[icol]];
		}
	}
	return A_trunc;
}

std::vector<std::vector<double>> eye(int size)
{
	vector<vector<double>> ans = zeros(size, size);
	for (int i = 0; i < size; i++) {
		ans[i][i] = 1;
	}
	return ans;
}

std::vector<std::vector<double>> zeros(int rows, int cols)
{
	vector<vector<double>> ans;
	resize(ans, rows, cols);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ans[i][j] = 0;
		}
	}
	return ans;
}

std::vector<std::vector<double>> ones(int rows, int cols)
{
	vector<vector<double>> ans;
	resize(ans, rows, cols);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ans[i][j] = 1.0;
		}
	}
	return ans;
}

std::vector<std::vector<double>> arr2vec(std::vector<double> a)
{
	return std::vector<std::vector<double>>({a});
}

std::vector<double> vec2arr(std::vector<std::vector<double>> A)
{
	vector<double> a;
	for (int row = 1; row < A.size(); row++) {
		for (int col = 1; col < A[0].size(); col++) {
			a.push_back(A[row][col]);
		}
	}
	return a;
}

std::vector<double> operator + (std::vector<double> a, std::vector<double> b)
{
	if (a.size() != b.size()) throw - 1;
	vector<double> c = {};
	for (int i = 0; i < a.size(); i++) {
		c.push_back(a[i] + b[i]);
	}
	return c;
}

std::vector<double> operator - (std::vector<double> a, std::vector<double> b)
{
	if (a.size() != b.size()) throw - 1;
	vector<double> c = {};
	for (int i = 0; i < a.size(); i++) {
		c.push_back(a[i] - b[i]);
	}
	return c;
}

std::vector<double> operator * (double k, std::vector<double> a)
{
	vector<double> b;
	for (int i = 0; i < a.size(); i++) {
		b.push_back(k * a[i]);
	}
	return b;
}
std::vector<double> operator * (std::vector<double> a, double k)
{
	return k * a;
}

std::vector<double> dotProduct(std::vector<double> a, std::vector<double> b)
{
	if (a.size() != b.size()) throw - 1;
	vector<double> c = {};
	for (int i = 0; i < a.size(); i++) {
		c.push_back(a[i] * b[i]);
	}
	return c;
}

std::vector<double> dotDivide(std::vector<double> a, std::vector<double> b)
{
	if (a.size() != b.size()) throw - 1;
	vector<double> c = {};
	for (int i = 0; i < a.size(); i++) {
		c.push_back(a[i] / b[i]);
	}
	return c;
}

double cosine(std::vector<double> a, std::vector<double> b)
{
	double numerator = 0;
	for (int i = 0; i < a.size(); i++) {
		numerator += a[i] * b[i];
	}
	double result = numerator / (myNorm(a) * myNorm(b));
	return result;
}

std::vector<double> truncate(std::vector<double> a, std::vector<int> cols)
{
	vector<double> a_trunc;
	a_trunc.resize(cols.size());
	for (int icol = 0; icol < cols.size(); icol++) {
		a_trunc[icol] = a[cols[icol]];
	}
	return a_trunc;
}

std::vector<double> unify(std::vector<double> a)
{
	double mag = myNorm(a);
	vector<double> b = a;
	for (int i = 0; i < a.size(); i++) {
		b[i] /= mag;
	}
	return b;
}
