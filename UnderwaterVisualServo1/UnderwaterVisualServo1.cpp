#include <iostream>

#include "MyDemos.h"
#include "MyExperiment.h"

//#include <opencv2/video/video.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/core.hpp>
//#include <iostream>
//#include <cstdio>


using namespace std;
using namespace cv;

//抓取时置1，其他时候0
static int taskTag;


int main()
{
	/**********旧版就有的**********/
	//demoPreview();
	//demoEstimateCubePose();
	//demoVisualServo_OnlyCamera();
	//demoVisualServo_Bilateral_Grab();

	/************新加的************/
	//demoFeatureExtractAndMatch();
	demoMLESAC2();
	//demoSearchFile();

	/************改进的************/
	//demoIBVS();
	//demoPerspective();
	//demoKinematics();
	//demoDifferentialKinematics();	
	//demoUniversalIK();
	return 0;
}