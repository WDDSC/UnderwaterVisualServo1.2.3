#include "MyTargetTrack.h"
using namespace std;
using namespace cv;

/*光流追踪类*/
MyOptFlowTrack::MyOptFlowTrack()
{
	setParams();
}

MyOptFlowTrack::~MyOptFlowTrack(){}

int MyOptFlowTrack::setParams(int maxCount, double qLevel, double minDist)
{
	this->maxCount = maxCount;
	this->qLevel = qLevel;
	this->minDist = minDist;
    return 0;
}

int MyOptFlowTrack::init(int maxCount, double qLevel, double minDist)
{
    setParams(maxCount, qLevel, minDist);
    return 0;
}

int MyOptFlowTrack::track(cv::Mat& src)
{
    cvtColor(src, gray, CV_BGR2GRAY);
    src.copyTo(output);
    // 添加特征点
    if (points[0].size() <= 10)
    {
        goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
        points[0].insert(points[0].end(), features.begin(), features.end());
        initial.insert(initial.end(), features.begin(), features.end());
    }
    // 若是第一帧，则光流法输入的前后两帧都是第一帧
    if (gray_prev.empty())
    {
        gray.copyTo(gray_prev);
    }
    // l-k光流法运动估计
    calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
    // 去掉一些不好的特征点
    int k = 0;
    for (size_t i = 0; i < points[1].size(); i++)
    {
        if (status[i] && ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 2))
        {
            initial[k] = initial[i];
            points[1][k++] = points[1][i];
        }
    }
    points[1].resize(k);
    initial.resize(k);
    swap(gray_prev, gray);
    return 0;
}

int MyOptFlowTrack::drawResult(cv::Mat& dst)
{
    // 显示特征点和运动轨迹
    for (size_t i = 0; i < points[1].size(); i++)
    {
        line(output, initial[i], points[1][i], Scalar(0, 0, 255));
        circle(output, points[1][i], 3, Scalar(0, 255, 0), -1);
    }
    // 把当前跟踪结果作为下一此参考
    swap(points[1], points[0]);
    //imshow(window_name, output);
    //dst = output.clone();
    output.copyTo(dst);
    return 0;
}

int MyOptFlowTrack::getFeaturePoints(std::vector<cv::Point2f>& previousPoints, std::vector<cv::Point2f>& predictedPoints)
{
	previousPoints = this->initial;
	predictedPoints = this->points[1];
    return 0;
}

/*ORB匹配类*/
FeatureExtractAndMatch::FeatureExtractAndMatch() {};

FeatureExtractAndMatch::~FeatureExtractAndMatch() {};

int FeatureExtractAndMatch::init(cv::Mat refImg, int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType)
{
    setParams(refImg, pointNum, matcherType, sifterType);
    detector->detectAndCompute(refImg, Mat(), keypointsRefer, descriptorRefer);
    return 0;
}

int FeatureExtractAndMatch::init(int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType)
{
    setParams(pointNum, matcherType, sifterType);
    return 0;
}

int FeatureExtractAndMatch::Match(cv::Mat& input, cv::Mat& refer)
{
    refer.copyTo(refImg);
    input.copyTo(inputImg);
    //检测并计算成描述子
    detector->detectAndCompute(refImg, Mat(), keypointsRefer, descriptorRefer);
    detector->detectAndCompute(inputImg, Mat(), keypointsInput, descriptorInput);
    //特征匹配
    FlannBasedMatcher fbmatcher(new flann::LshIndexParams(20, 10, 2));
    //将找到的描述子进行匹配并存入matches中
    fbmatcher.match(descriptorRefer, descriptorInput, matches);
    return 0;
}

int FeatureExtractAndMatch::Match(cv::Mat& input)
{
    input.copyTo(inputImg);
    //检测并计算成描述子
    detector->detectAndCompute(inputImg, Mat(), keypointsInput, descriptorInput);
    //特征匹配
    FlannBasedMatcher fbmatcher(new flann::LshIndexParams(20, 10, 2));
    //将找到的描述子进行匹配并存入matches中
    fbmatcher.match(descriptorRefer, descriptorInput, matches);
    return 0;
}

int FeatureExtractAndMatch::Sift()
{
	double minDist = 1000;
	double maxDist = 0;
	//找出最优描述子
	vector<DMatch> goodmatches;
	for (int i = 0; i < descriptorRefer.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < minDist)
		{
			minDist = dist;
		}
		if (dist > maxDist)
		{
			maxDist = dist;
		}

	}
	for (int i = 0; i < descriptorRefer.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < max(2 * minDist, 0.02))
		{
			goodmatches.push_back(matches[i]);
		}
	}
    matches = goodmatches;
    return 0;
}
int FeatureExtractAndMatch::drawResult(cv::Mat& dst)
{
    drawMatches(refImg, keypointsRefer, inputImg, keypointsInput, matches, dst,
        Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return 0;
}

int FeatureExtractAndMatch::getResult(std::vector<cv::KeyPoint>& featurePointsRefer, std::vector<cv::KeyPoint>& featurePointsInput)
{
    featurePointsRefer.resize(0);
    featurePointsInput.resize(0);
    for (int i = 0; i < matches.size(); i++) {
        featurePointsRefer.push_back(keypointsRefer[matches[i].queryIdx]);
        featurePointsInput.push_back(keypointsInput[matches[i].trainIdx]);
    }
    return 0;
}

int FeatureExtractAndMatch::setParams(cv::Mat refImg, int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType)
{
    this->refImg = refImg;
    this->detector = ORB::create(pointNum);
    this->matcherType = matcherType;
    this->sifterType = sifterType;
    return 0;
}

int FeatureExtractAndMatch::setParams(int pointNum, enum_FeatureMatcherType matcherType, enum_MatchSifterType sifterType)
{
    this->refImg = Mat::zeros(Size(640, 480), CV_8UC3);
    this->detector = ORB::create(pointNum);
    this->matcherType = matcherType;
    this->sifterType = sifterType;
    return 0;
}