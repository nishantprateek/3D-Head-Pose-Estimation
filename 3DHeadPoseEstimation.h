#include "ImageRenderer.h"

class 3DHeadPoseEstimation
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
public:
	3DHeadPoseEstimation();
	int Run(HINSTANCE hInstance, int nCmdShow);

};
