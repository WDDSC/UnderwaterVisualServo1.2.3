#include "MyIdentify.h"

using namespace std;
using namespace cv;
using namespace xfeatures2d;


//===========================================================================

MyFeatureExtractor::MyFeatureExtractor(enum_FeatureExtractor type)
{
	this->type = type;
	switch (type)
	{
	case enum_FeatureExtractor::SIFT:
		detector.detectorSift = SIFT::create();
		break;
	case enum_FeatureExtractor::SURF:
		detector.detectorSurf = SURF::create();
		break;
	case enum_FeatureExtractor::FREAK:
		detector.detectorFreak = FREAK::create();
		break;
	case enum_FeatureExtractor::ORB:
		detector.detectorOrb = ORB::create();
		//detector.detectorOrb->setMaxFeatures(200);
		break;
	default:
		ReportWarning("[MyFeatureExtractor::MyFeatureExtractor()] Invalid type of feature extractor");
		detector.detectorOrb = ORB::create();
		//detector.detectorOrb->setMaxFeatures(200);
		break;
	}
}

MyFeatureExtractor::~MyFeatureExtractor() {}

void MyFeatureExtractor::setImg(Mat srcImg)
{
	this->srcImg = srcImg;
	keypoints.clear();
	descriptors = Mat();
}

void MyFeatureExtractor::nonMaxSupress()
{
	vector<double> U, V;
	vector<KeyPoint> newKeypoints;
	U.push_back(keypoints[0].pt.x);
	V.push_back(keypoints[0].pt.y);
	newKeypoints.push_back(keypoints[0]);
	double u, v;
	bool isValidPoint;
	for (int i = 0; i < keypoints.size(); i++) {
		isValidPoint = true;
		u = keypoints[i].pt.x;
		v = keypoints[i].pt.y;
		for (int j = 0; j < newKeypoints.size(); j++) {
			if (abs(u - U[j]) + abs(v - V[j]) <= 6.0) {
				if (keypoints[i].response > newKeypoints[j].response) {
					newKeypoints[j] = keypoints[i];
				}
				isValidPoint = false;
				break;
			}
		}
		if (isValidPoint) {
			U.push_back(keypoints[i].pt.x);
			V.push_back(keypoints[i].pt.y);
			newKeypoints.push_back(keypoints[i]);
		}
	}
	this->keypoints = newKeypoints;
}

int MyFeatureExtractor::extract(bool openNMS)
{
	switch (type)
	{
	case enum_FeatureExtractor::SIFT:
		detector.detectorSift->detect(srcImg, keypoints, noArray());
		if(openNMS)
			nonMaxSupress();
		detector.detectorSift->compute(srcImg, keypoints, descriptors);
		break;
	case enum_FeatureExtractor::SURF:
		detector.detectorSurf->detect(srcImg, keypoints, noArray());
		if (openNMS)
			nonMaxSupress();
		detector.detectorSurf->compute(srcImg, keypoints, descriptors);
		break;
	case enum_FeatureExtractor::FREAK:
		detector.detectorFreak->detect(srcImg, keypoints, noArray());
		if (openNMS)
			nonMaxSupress();
		detector.detectorFreak->compute(srcImg, keypoints, descriptors);
		break;
	case enum_FeatureExtractor::ORB:
		detector.detectorOrb->detect(srcImg, keypoints, noArray());
		if (openNMS)
			nonMaxSupress();
		detector.detectorOrb->compute(srcImg, keypoints, descriptors);
		break;
	default:
		detector.detectorOrb->detect(srcImg, keypoints, noArray());
		if (openNMS)
			nonMaxSupress();
		detector.detectorOrb->compute(srcImg, keypoints, descriptors);
		break;
	}
	return keypoints.size();
}

vector<KeyPoint> MyFeatureExtractor::getKeypoints()
{
	return keypoints;
}

Mat MyFeatureExtractor::getDescriptors()
{
	return descriptors;
}

Mat MyFeatureExtractor::getMarkedImg()
{
	Mat markedImg;
	drawKeypoints(srcImg, keypoints, markedImg);
	return markedImg;
}

//===========================================================================

MyFeatureMatcher::MyFeatureMatcher(enum_FeatureMatcher type)
{
	this->type = type;
	switch (type)
	{
	case enum_FeatureMatcher::BF:
		matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
		break;
	case enum_FeatureMatcher::HAMMING:
		matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
		break;
	case enum_FeatureMatcher::KNN:
		matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
		break;
	default:
		ReportWarning("[MyFeatureMatcher::MyFeatureMatcher()] Invalid type of feature matcher");
		matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
		break;
	}
}

MyFeatureMatcher::~MyFeatureMatcher() {}

int MyFeatureMatcher::match()
{
	if (type == enum_FeatureMatcher::KNN) {
		ReportWarning("[MyFeatureMatcher::match()] KNN match has not been complete");
		matcher->knnMatch(descriptors2, descriptors1, knnMatches, 2);		
	}
	else {
		matcher->match(descriptors2, descriptors1, matches);
	}
	return 0;
}

void MyFeatureMatcher::setTemplate(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1)
{
	this->keypoints1 = keypoints1;
	this->descriptors1 = descriptors1;
}

void MyFeatureMatcher::setActual(std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2)
{
	this->keypoints2 = keypoints2;
	this->descriptors2 = descriptors2;
}


vector<DMatch> MyFeatureMatcher::getMatches()
{
	return matches;
}

Mat MyFeatureMatcher::getMatchedImg(Mat src, Mat dst)
{
	Mat out;
	drawMatches(src, keypoints1, dst, keypoints2, matches, out);
	return out;
}

//===========================================================================

MyInlierModel::MyInlierModel()
{
	L = ImgJacMatrix();
	this->dP = {};
	this->depth = 1.0;
}

MyInlierModel::MyInlierModel(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, vector<DMatch> matches, double z)
{
	vector<double> U1, V1, U2, V2, dS;
	this->depth = z;
	for (int i = 0; i < matches.size(); i++) {
		U1.push_back(keypoints1[matches[i].trainIdx].pt.x);
		V1.push_back(keypoints1[matches[i].trainIdx].pt.y);
		U2.push_back(keypoints2[matches[i].queryIdx].pt.x);
		V2.push_back(keypoints2[matches[i].queryIdx].pt.y);
	}
	L = ImgJacMatrix(U1, V1, z);
	for (int i = 0; i < matches.size(); i++) {
		dS.push_back(U2[i] - U1[i]);
		dS.push_back(V2[i] - V1[i]);
	}
	this->dP = pinv(this->L.elements) * dS;
}

MyInlierModel::~MyInlierModel() {}

vector<double> MyInlierModel::predict(std::vector<cv::KeyPoint> keypoints)
{
	vector<double> S = keypoints2doubles(keypoints);
	vector<double> U, V;
	for (int i = 0; i < S.size(); i += 2) {
		U.push_back(S[i]);
		V.push_back(S[i + 1]);
	}
	ImgJacMatrix Js = ImgJacMatrix(U, V, this->depth);
	return S + Js.elements * dP;

	//vector<double> S_hat = S;
	//vector<double> dS;
	//for (int i = 0; i < S.size(); i += 2) {
	//	ImgJacMatrix Js = ImgJacMatrix(S[i], S[i + 1], this->depth);
	//	dS = Js.elements * dP;
	//	S_hat[i] = S[i] + dS[0];
	//	S_hat[i + 1] = S[i + 1] + dS[1];
	//}
	//return S_hat;
}

MyInlierModelEstimator::MyInlierModelEstimator(enum_InlierModelEstimator type)
{
	this->type = type;
}

MyInlierModelEstimator::~MyInlierModelEstimator() {}

void MyInlierModelEstimator::set(vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, vector<DMatch> matches)
{
	this->keypoints1 = keypoints1;
	this->keypoints2 = keypoints2;
	this->matches = matches;
	this->keypoints2_hat = keypoints1;
	for (int i = 0; i < keypoints1.size(); i++) {
		this->predictMatches.push_back(DMatch(i, i, 0.0));
	}
}

vector<double> MyInlierModelEstimator::modelPredErr(MyInlierModel& model)
{
	vector<double> S, S_star, S_pred;
	for (int i = 0; i < matches.size(); i++) {
		S_star.push_back(keypoints1[matches[i].trainIdx].pt.x);
		S_star.push_back(keypoints1[matches[i].trainIdx].pt.y);
		S.push_back(keypoints2[matches[i].queryIdx].pt.x);
		S.push_back(keypoints2[matches[i].queryIdx].pt.y);
	}
	S_pred = model.predict(doubles2keypoints(S_star));
	return S_pred - S;
}

double MyInlierModelEstimator::nLogL(vector<double> modelPredErr, double mismatchRatio, double sigma, double mismatchRange)
{
	double Pr, PE;
	double negLogLikelihood = 0;
	sigma = 0.50;
	for (int i = 0; i < modelPredErr.size(); i+=2) {
		PE = myNorm({ modelPredErr[i], modelPredErr[i + 1] });
		Pr = (1 - mismatchRatio) * (1 / sqrt(2 * 3.1416 * sigma * sigma)) * exp(-PE * PE / 2 / sigma / sigma) + mismatchRatio * (1 / mismatchRange);
		negLogLikelihood -= log10(Pr);
	}
	return negLogLikelihood;
}

void MyInlierModelEstimator::eliminateOutlier(MyInlierModel& bestPutativeModel, double sigma)
{
	double coef = 1.0;
	//不断扩大容许误差，囊括足够的“暂定内点”
	RECHOOSE:
	vector<KeyPoint> putativeInliers1, putativeInliers2;
	vector<DMatch> putativeInlierMatches;
	putativeInliers1 = {};
	putativeInliers2 = {};
	putativeInlierMatches = {};
	vector<double> predErr = modelPredErr(bestPutativeModel);
	double threshold = coef * 1.96 * sigma;
	coef *= 1.2;
	for (int i = 0; i < matches.size(); i++) {
		if (myNorm({ predErr[2 * i], predErr[2 * i + 1] }) < threshold) {
			putativeInliers1.push_back(keypoints1[matches[i].trainIdx]);
			putativeInliers2.push_back(keypoints2[matches[i].queryIdx]);
			putativeInlierMatches.push_back(DMatch(putativeInliers2.size() - 1, putativeInliers1.size() - 1, matches[i].distance));
		}
	}
	if (putativeInlierMatches.size() <= 8)
		goto RECHOOSE;

	//基于“暂定内点”进行模型估计
	MyInlierModel putativeInlierModel = MyInlierModel(putativeInliers1, putativeInliers2, putativeInlierMatches);
	//vector<double> predErrPutativeInlier = keypoints2doubles(putativeInliers2) - putativeInlierModel.predict(putativeInliers1);
	vector<double> predErrPutativeInlier = keypoints2doubles(keypoints2) - putativeInlierModel.predict(keypoints1);
	//从“暂定内点”中选取真正的内点
	coef = 1.0;
	MOREINLIER:
	threshold = coef * 1.96 * sigma;
	this->inliers1 = {};
	this->inliers2 = {};
	this->inlierMatches = {};
	//for (int i = 0; i < putativeInlierMatches.size(); i++) {
	for (int i = 0; i < matches.size(); i++) {
		vector<double> predErrPutativeInlier = keypoints2doubles({ keypoints2[matches[i].queryIdx] }) - putativeInlierModel.predict({ keypoints1[matches[i].trainIdx] });
		//if (myNorm({ predErrPutativeInlier[2*i],predErrPutativeInlier[2*i + 1] }) <= threshold) {
		if (myNorm({ predErrPutativeInlier }) <= threshold) {
			//inliers1.push_back(putativeInliers1[i]);
			//inliers2.push_back(putativeInliers2[i]);
			inliers1.push_back(keypoints1[matches[i].trainIdx]);
			inliers2.push_back(keypoints2[matches[i].queryIdx]);
			inlierMatches.push_back(DMatch(inliers2.size() - 1, inliers1.size() - 1, matches[i].distance));
		}
	}
	if (inliers1.size() <= 4) {
		coef *= 1.2;
		goto MOREINLIER;
	}
	cout << this->inlierMatches.size() << " inliers are selected within error of " << threshold << endl;

	//this->inliers1 = {};
	//this->inliers2 = {};
	//this->inlierMatches = {};
	//const vector<int> goodInd1 = { 105,122,115,111,110,108,108,9,114,99,113,107,126,123,119,113,125 };
	//const vector<int> goodInd2 = { 7,15,17,20,32,35,41,54,56,57,59,63,69,71,73,75,76 };
	//for (int j = 0; j < 17; j++) {
	//	for (int i = 0; i < matches.size(); i++) {
	//		if (matches[i].trainIdx == goodInd1[j] && matches[i].queryIdx == goodInd2[j]) {
	//			inliers1.push_back(keypoints1[matches[i].trainIdx]);
	//			inliers2.push_back(keypoints2[matches[i].queryIdx]);
	//			inlierMatches.push_back(DMatch(inliers2.size() - 1, inliers1.size() - 1, matches[i].distance));
	//			break;
	//		}
	//	}
	//}
	return;
}

MyInlierModel MyInlierModelEstimator::MLESAC(int P, double mismatchRatio, double sigma, double mismatchRange, double confidence)
{
	int K = floor(log10(1 - confidence) / log10(1 - pow(1 - mismatchRatio, P)));
	cout << K << " samples are selected to achieve " << round(100.0 * confidence) << "% confidence level under " << round(100.0 * mismatchRatio) << "% mismatching proportion" << endl;
	vector<KeyPoint> samplePoints1, samplePoints2;
	vector<DMatch> sampleMatches;
	MyInlierModel putativeModel, bestPutativeModel;
	vector<double> costs = {};
	double minCost = DBL_MAX;
	double cost;
	int bestInd = 0;
	for (int k = 0; k < K; k++) {
		//生成sample并计算putativeModels
		default_random_engine e;
		//unsigned int seed = 100 * k + time(NULL);
		unsigned int seed = k+1;
		e.seed(seed);
		seed = e();
		sampleMatches = srandomSelectFrom(matches, P, seed);
		samplePoints1.clear();
		samplePoints2.clear();
		for (int p = 0; p < P; p++) {
			samplePoints1.push_back(keypoints1[sampleMatches[p].trainIdx]);
			samplePoints2.push_back(keypoints2[sampleMatches[p].queryIdx]);
			sampleMatches[p].queryIdx = p;
			sampleMatches[p].trainIdx = p;
		}
		putativeModel = MyInlierModel(samplePoints1, samplePoints2, sampleMatches);
		//筛选putative模型中的最优者
		cost = nLogL(modelPredErr(putativeModel), mismatchRatio, sigma, mismatchRange);
		//cout << k << "-th putative model prediction error = " << cost << ", condition number = " << putativeModel.L.cond() << endl;
		if (cost < minCost) {
			minCost = cost;
			bestPutativeModel = putativeModel;
		}
	}
	cout << "Best putative model prediction cost: " << minCost << ", condition number: " << bestPutativeModel.L.cond() << endl;
	//筛除模型外点
	eliminateOutlier(bestPutativeModel, sigma);
	//重新估计内点模型
	this->estModel = MyInlierModel(inliers1, inliers2, inlierMatches);
	vector<double> predErrInlier = keypoints2doubles(inliers2) - this->estModel.predict(inliers1);
	cout << "Prediction error of inliers:" << endl << predErrInlier << endl;
	cout << "Inlier model prediction cost: " << endl << nLogL(predErrInlier, mismatchRatio, sigma, mismatchRange) << endl;
	//预测keypoints2的位置
	this->keypoints2_hat = doubles2keypoints(this->estModel.predict(keypoints1));

	return this->estModel;
}

Mat MyInlierModelEstimator::drawResult(Mat& img1, Mat& img2)
{
	Mat outImg;
	//drawMatches(img1, inliers1, img2, inliers2, inlierMatches, outImg);
	drawMatches(img1, keypoints1, img2, keypoints2_hat, predictMatches, outImg);
	//drawMatches(img1, inliers1, img2, keypoints2_hat, predictMatches, outImg);
	return outImg;
}

Mat MyInlierModelEstimator::drawInliers(Mat& img1, Mat& img2)
{
	Mat outImg;
	drawMatches(img1, inliers1, img2, inliers2, inlierMatches, outImg);
	//drawMatches(img1, keypoints1, img2, keypoints2_hat, predictMatches, outImg);
	//drawMatches(img1, inliers1, img2, keypoints2_hat, predictMatches, outImg);
	return outImg;
}

void shrink2fit(vector<KeyPoint>& keypoints1, Mat& descriptors1, vector<KeyPoint>& keypoints2, Mat& descriptors2)
{
	if (keypoints1.size() > keypoints2.size()) {
		descriptors1 = descriptors1(Rect(0, 0, 32, keypoints2.size()));
		while (keypoints1.size() > keypoints2.size()) {
			keypoints1.pop_back();
		}
	}
	else {
		descriptors2 = descriptors2(Rect(0, 0, 32, keypoints1.size()));
		while (keypoints1.size() < keypoints2.size()) {
			keypoints2.pop_back();
		}
	}
	return;
}

vector<double> keypoints2doubles(vector<KeyPoint> keypoints)
{
	vector<double> S;
	for (int i = 0; i < keypoints.size(); i++) {
		S.push_back(keypoints[i].pt.x);
		S.push_back(keypoints[i].pt.y);
	}
	return S;
}
vector<KeyPoint> doubles2keypoints(vector<double> S)
{
	vector<KeyPoint> keypoints;
	for (int i = 0; i < S.size(); i+=2) {
		keypoints.push_back(KeyPoint(S[i], S[i + 1], 1.0));
	}
	return keypoints;
}

//===========================================================================

void matchResult::printResult(ostream& os)
{
	os << "Model: " << endl << arr2vec(model.dP);
	os << "Matched points position: " << endl;
	os << "Template KP X" << "\t" << "Template KP Y" << "\t" << "Actual KP X" << "\t" << "Actual KP Y" << endl;
	for (int i = 0; i < matches.size(); i++) {
		int idx1 = matches[i].trainIdx;
		int idx2 = matches[i].queryIdx;
		os << keypoints1[idx1].pt.x << "\t" << keypoints1[idx1].pt.y << "\t" << keypoints2[idx2].pt.x << "\t" << keypoints2[idx2].pt.y << endl;
	}
	return;
}

void matchResult::printResult(fstream& fs)
{
	fs << "Model: " << endl << arr2vec(model.dP);
	fs << "Matched points position: " << endl;
	fs << "Template KP X" << "\t" << "Template KP Y" << "\t" << "Actual KP X" << "\t" << "Actual KP Y" << endl;
	for (int i = 0; i < matches.size(); i++) {
		int idx1 = matches[i].trainIdx;
		int idx2 = matches[i].queryIdx;
		fs << keypoints1[idx1].pt.x << "\t" << keypoints1[idx1].pt.y << "\t" << keypoints2[idx2].pt.x << "\t" << keypoints2[idx2].pt.y << endl;
	}
	return;
}


MyRefinedMatcher::MyRefinedMatcher(enum_FeatureExtractor extractorType, enum_FeatureMatcher matcherType, enum_InlierModelEstimator estimatorType)
	: MyFeatureExtractor(extractorType), MyFeatureMatcher(matcherType), MyInlierModelEstimator(estimatorType)
{
	this->sampleSize = 3;
	this->mismatchRatio = 0.80;
	this->inlierVariance = 1.0;
	this->outlierRange = 100.0;
	this->confidence = 0.95;
}

MyRefinedMatcher::~MyRefinedMatcher() {}

void MyRefinedMatcher::setParams(int sampleSize, double mismatchRatio, double inlierVariance, double outlierRange, double confidence)
{
	this->sampleSize = sampleSize;
	this->mismatchRatio = mismatchRatio;
	this->inlierVariance = inlierVariance;
	this->outlierRange = outlierRange;
	this->confidence = confidence;
}

void MyRefinedMatcher::setTemplateImg(cv::Mat img)
{
	img.copyTo(this->imgTemplate);
}

void MyRefinedMatcher::setActualImg(cv::Mat img)
{
	img.copyTo(this->imgActual);
}

void MyRefinedMatcher::match()
{
	//图像特征提取
	this->MyFeatureExtractor::setImg(this->imgTemplate);
	this->MyFeatureExtractor::extract();
	this->keypoints1 = this->MyFeatureExtractor::getKeypoints();
	this->descriptors1 = this->MyFeatureExtractor::getDescriptors();
	this->MyFeatureExtractor::setImg(this->imgActual);
	this->MyFeatureExtractor::extract();
	this->keypoints2 = this->MyFeatureExtractor::getKeypoints();
	this->descriptors2 = this->MyFeatureExtractor::getDescriptors();
	shrink2fit(keypoints1, descriptors1, keypoints2, descriptors2);
	//初步特征匹配
	this->MyFeatureMatcher::setTemplate(keypoints1, descriptors1);
	this->MyFeatureMatcher::setActual(keypoints2, descriptors2);
	this->MyFeatureMatcher::match();
	this->rawMatches = this->MyFeatureMatcher::getMatches();
	//MLESAC算法优化匹配
	this->MyInlierModelEstimator::set(keypoints1, keypoints2, rawMatches);
	this->MyInlierModelEstimator::MLESAC(sampleSize, mismatchRatio, inlierVariance, outlierRange, confidence);
	this->inliers1 = this->MyInlierModelEstimator::inliers1;
	this->inliers2 = this->MyInlierModelEstimator::inliers2;
	this->inlierMatches = this->MyInlierModelEstimator::inlierMatches;
	this->model = this->MyInlierModelEstimator::estModel;
	this->keypoints2_hat = this->MyInlierModelEstimator::keypoints2_hat;
	this->refinedMatches = this->MyInlierModelEstimator::predictMatches;
}

matchResult MyRefinedMatcher::getResult()
{
	matchResult matchRes;
	matchRes.keypoints1 = this->keypoints1;
	matchRes.keypoints2 = this->keypoints2_hat;
	matchRes.matches = this->refinedMatches;
	matchRes.model = this->model;
	return matchRes;
}

Mat MyRefinedMatcher::getRawMatchImg()
{
	return this->MyFeatureMatcher::getMatchedImg(this->imgTemplate, this->imgActual);
}

Mat MyRefinedMatcher::getRefinedMatchImg()
{
	return this->MyInlierModelEstimator::drawResult(this->imgTemplate, this->imgActual);
}


//===========================================================================