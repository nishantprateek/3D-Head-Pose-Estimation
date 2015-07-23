#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include "iostream"

using namespace std;

template<class Interface>

inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
int GetMinor(long double **src, long double **dest, int row, int col, int order)
{
	// indicate which col and row is being copied to dest
	int colCount = 0, rowCount = 0;

	for (int i = 0; i < order; i++)
	{
		if (i != row)
		{
			colCount = 0;
			for (int j = 0; j < order; j++)
			{
				// when j is not the element
				if (j != col)
				{
					dest[rowCount][colCount] = src[i][j];
					colCount++;
				}
			}
			rowCount++;
		}
	}

	return 1;
}
long double CalcDeterminant(long double **mat, int n)
{
	long double d=0;
	int c, subi, i, j, subj;
	long double **submat = new long double*[n];
	for (int i = 0; i<n; i++)
		submat[i] = new long double[n];
	if (n == 2)
	{
		return((mat[0][0] * mat[1][1]) - (mat[1][0] * mat[0][1]));
	}
	else
	{
		for (c = 0; c < n; c++)
		{
			subi = 0;
			for (i = 1; i < n; i++)
			{
				subj = 0;
				for (j = 0; j < n; j++)
				{
					if (j == c)
					{
						continue;
					}
					submat[subi][subj] = mat[i][j];
					subj++;
				}
				subi++;
			}
			d = d + (pow(-1, c) * mat[0][c] * CalcDeterminant(submat,n-1));
		}
	}
	return d;
}



void MatrixInversion(long double **A, int order, long double **Y)
{
	// get the determinant of a
	long double det = 1.0 / CalcDeterminant(A, order);

	// memory allocation
	long double *temp = new long double[(order - 1)*(order - 1)];
	long double **minor = new long double*[order - 1];
	for (int i = 0; i<order - 1; i++)
		minor[i] = temp + (i*(order - 1));

	for (int j = 0; j<order; j++)
	{
		for (int i = 0; i<order; i++)
		{
			// get the co-factor (matrix) of A(j,i)
			GetMinor(A, minor, j, i, order);
			Y[i][j] = det*CalcDeterminant(minor, order - 1);
			if ((i + j) % 2 == 1)
				Y[i][j] = -Y[i][j];
		}
	}

	// release memory
	//delete [] minor[0];
	delete[] temp;
	delete[] minor;
}


void matrixMultiply(long double **arr1, long double **arr2, long double **ans, int N, int L, int M)
{
	for (int R = 0; R < N; R++)
	{
		for (int C = 0; C < M; C++)
		{
			ans[R][C] = 0;
			for (int T = 0; T < L; T++)
				ans[R][C] += (arr1[R][T] * arr2[T][C]);
		}
	}
}
void matrixTranspose(long double **arr1, long double **arr2, int N, int M)
{
	for (int R = 0; R<N; R++)
		for (int C = 0; C<M; C++)
			arr2[R][C] = arr1[C][R];
}

int main()
{
	cv::setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	hResult = pSensor->Open();

	// Source
	IDepthFrameSource* pDepthSourceSub;
	hResult = pSensor->get_DepthFrameSource(&pDepthSourceSub);

	IDepthFrameReader* pDepthReaderSub;
	hResult = pDepthSourceSub->OpenReader(&pDepthReaderSub);

	IFrameDescription* pDepthDescriptionSub;
	hResult = pDepthSourceSub->get_FrameDescription(&pDepthDescriptionSub);

	int depthWidthSub = 0;
	int depthHeightSub = 0;
	pDepthDescriptionSub->get_Width(&depthWidthSub);
	pDepthDescriptionSub->get_Height(&depthHeightSub);
	unsigned int depthBufferSizeSub = depthWidthSub*depthHeightSub*sizeof(unsigned short);
	cv::Mat depthBufferMatSub(depthHeightSub, depthWidthSub, CV_16UC1);
	cv::Mat depthMatSub(depthHeightSub, depthWidthSub, CV_8UC1);

	unsigned short minDepthSub, maxDepthSub;
	pDepthSourceSub->get_DepthMinReliableDistance(&minDepthSub);
	pDepthSourceSub->get_DepthMaxReliableDistance(&maxDepthSub);
	cv::namedWindow("Background Calibration");
	while (1)
	{
		IDepthFrame* pDepthSub = nullptr;
		hResult = pDepthReaderSub->AcquireLatestFrame(&pDepthSub);
		if (SUCCEEDED(hResult))
		{
			hResult = pDepthSub->AccessUnderlyingBuffer(&depthBufferSizeSub, reinterpret_cast<UINT16**>(&depthBufferMatSub.data));
			if (SUCCEEDED(hResult))
			{
				depthBufferMatSub.convertTo(depthMatSub, CV_8U, -255.0f / 8000.0f, 255.0f);
			}
		}
		SafeRelease(pDepthSub);
		cv::imshow("Background Calibration", depthMatSub);
		if (cv::waitKey(30) == VK_ESCAPE)
		{
			break;
		}
	}
	
	SafeRelease(pDepthSourceSub);
	SafeRelease(pDepthReaderSub);
	SafeRelease(pDepthDescriptionSub);

	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	// Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);

	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);

	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);

	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);

	IFrameDescription* pColorDescription;
	hResult = pColorSource->get_FrameDescription(&pColorDescription);

	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	
	int depthWidth = 0; 
	int depthHeight = 0;
	int colorWidth = 0;
	int colorHeight = 0;

//	int old_ctr;
//	int old_colorX = 0, old_colorY = 0;
	pDepthDescription->get_Width(&depthWidth);
	pDepthDescription->get_Height(&depthHeight);
	pColorDescription->get_Width(&colorWidth);
	pColorDescription->get_Height(&colorHeight);
	unsigned int depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);
	unsigned int colorBufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);

	cv::Mat depthBufferMat(depthHeight, depthWidth, CV_16UC1);
	cv::Mat depthMat(depthHeight, depthWidth, CV_8UC1);
	cv::Mat Body(depthHeight, depthWidth, CV_8UC1);
	cv::Mat colorBufferMat(colorHeight, colorWidth, CV_8UC4);
	cv::Mat colorMat(colorHeight , colorWidth , CV_8UC4);

	// To store previous frame depth information
	UINT16* oldArray;
	oldArray = new UINT16[512*424];

	cv::namedWindow("Depth");
	cv::namedWindow("Color");
	
	// To store previous top-left coordinate
	CameraSpacePoint oldTopLeft{ 0, 0, 0 };
	int ctr = 0, old_top = 0, old_left = 0;
	
	// Measure of time
	SYSTEMTIME curr_time, old_time;
	old_time.wMilliseconds = 0;

	long double z, z_old; 
	stringstream s;

	// In case, subject goes out of range, previous values are re-printed
	long double **old_ans = new long double*[6];
	for (int i = 0; i < 6; i++)
	{
		old_ans[i] = new long double[1];
		old_ans[i][0] = 0;
	}

	// Frame-by-frame loop
	while (1)
	{
		// Our current depth array
		UINT16 *depthArray;
		depthArray = new UINT16[512 * 424];
		
		// structure to store matrix A {p,q,-1,r,s,t}
		/* We have only considered 50 points along the line of the center of the frame. In case all pixels are to be mapped,
		  for higher accuracy(but low performance), increase it upto 10,000 and the change should be made in the intermediate 
		   structures to hold values as well*/
		long double **equations = new long double*[50];
		for (int i = 0; i < 50; i++)
		{
 			equations[i] = new long double[6];
		}

		//Acquiring depth information
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult))
		{
			GetSystemTime(&curr_time);
			hResult = pDepthFrame->AccessUnderlyingBuffer(&depthBufferSize, reinterpret_cast<UINT16**>(&depthBufferMat.data));
			hResult = pDepthFrame->CopyFrameDataToArray(depthBufferSize, depthArray);			
			if (SUCCEEDED(hResult))
			{
				depthBufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			}
		}
		
		SafeRelease(pDepthFrame);
		
		// Acquiring RGB information
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult))
		{
			
			if (SUCCEEDED(hResult))
			{
				hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hResult))
				{
					cv::resize(colorBufferMat, colorMat, cv::Size(), 1, 1);
				}
			}
		}

		SafeRelease(pColorFrame);

		// To remove background
		Body = (depthMat - depthMatSub);
		
		int hor_count = 0, left_coord, top_coord;
		int i, j;
		for (i = 0; i < Body.rows; i++)
		{
			bool break_loop = false;
			for (j = 0; j < Body.cols; j++)
			{
				if (int(Body.at<char>(i, j)) >= 10)
				{
					hor_count++;
					if (hor_count > 10)
					{
						top_coord = i;
						left_coord = j;
						break_loop = true;
						break;
					}
					else continue;
				}
				else
				{
					hor_count = 0;
					top_coord = i;
					left_coord = j;
				}
			}
			if (break_loop == true) break;
		}

		DepthSpacePoint point1;
		ColorSpacePoint point_top;
		CameraSpacePoint topLeft;
		point1.X = left_coord;
		point1.Y = top_coord;
		hResult = pCoordinateMapper->MapDepthPointToCameraSpace(point1, depthArray[(top_coord+5) * 512 + (left_coord)], &topLeft);
		int colorX, colorY;
		hResult = pCoordinateMapper->MapDepthPointToColorSpace(point1, depthArray[top_coord * 512 + left_coord], &point_top);
		// creating the box around the face in color space
		if (SUCCEEDED(hResult))
		{
			colorX = static_cast<int>(std::floor(point_top.X + 0.5));
			colorY = static_cast<int>(std::floor(point_top.Y + 0.5));
			
				cv::rectangle(colorMat, cv::Point((colorX - 150), (colorY - 30)), cv::Point((colorX + 150), (colorY + 270)), cv::Scalar(0, 0, 0), +2, 4);
		}
		// box in the depth space
		cv::rectangle(Body, cv::Point((left_coord - 40), (top_coord - 15)), cv::Point((100 + (left_coord - 40)), (100 + (top_coord - 15))), cv::Scalar(255, 255, 255), +2, 4);
		
		cv::imshow("Depth", Body);
		cv::rectangle(colorMat, cv::Point(0, 0), cv::Point(150, 170), cv::Scalar(255, 255, 255), CV_FILLED, 4); 
		

		if ((old_top >= 15) && (top_coord >= 15) && (left_coord >= 40) && (old_left >= 40))              // checking for values in range
		{
			if (depthArray[(512 * (top_coord)) + (left_coord)] >= 0 && depthArray[(512 * (top_coord)) + (left_coord)] < 5000)
			{
				
				if (ctr > 0)
				{
					long double **Z_t = new long double*[10];    // the Z matrix
					for (int i = 0; i < 10; i++)
						Z_t[i] = new long double[1];
					int myCtr = 0;
					int j;
					for (int i = (top_coord+10)*512+left_coord; i < (top_coord+20)*512+left_coord; i += 512)
					{
						j = (old_top+10+myCtr)*512 + old_left;
						CameraSpacePoint myPt, myOldPt;
						z = depthArray[i];                  // depth of current pixel
						z_old = depthArray[j];				// previous depth of current pixel					
						DepthSpacePoint point1, pointOld;
						point1.X = (i % 512);
						point1.Y = (i / 512);
						pointOld.X = j%512;
						pointOld.Y = j/512;

						// Mapping old and new pixels from depth space to camera space
						hResult = pCoordinateMapper->MapDepthPointToCameraSpace(point1, z , &myPt);
						hResult = pCoordinateMapper->MapDepthPointToCameraSpace(pointOld, z_old , &myOldPt);

						// Computing p
						if (myPt.X == myOldPt.X)					// In case dX = 0
							equations[myCtr][0] = 0;
						else
							equations[myCtr][0] = (myPt.Z - myOldPt.Z) / (myPt.X - myOldPt.X);

						// Computing q
						if (myPt.Y == myOldPt.Y)					// In case dY = 0
							equations[myCtr][1] = 0;
						else
							equations[myCtr][1] = (myPt.Z - myOldPt.Z ) / (myPt.Y - myOldPt.Y);

						equations[myCtr][2] = -1;

						// Computing r
						equations[myCtr][3] = -myOldPt.Y - (equations[myCtr][1] * myOldPt.Z);

						// Computing s
						equations[myCtr][4] = myOldPt.X + (equations[myCtr][0] * myOldPt.Z);

						// Computing t
						equations[myCtr][5] = equations[myCtr][1] * myOldPt.X - (equations[myCtr][0] * myOldPt.Y);

						// Computing Zt
						Z_t[myCtr][0] = (myPt.Z - myOldPt.Z)*1000.0f / (curr_time.wMilliseconds - old_time.wMilliseconds);
						
						myCtr++;
					}


					// Transpose of matrix A
					long double **equationTrans = new long double*[6];
					for (int i = 0; i < 6; i++)
						equationTrans[i] = new long double[10];

					// To store A*A_Trans
					long double **output1 = new long double*[6];
					for (int i = 0; i < 6; i++)
						output1[i] = new long double[6];

					// To store A_Trans*Z_t
					long double **output2 = new long double*[6];
					for (int i = 0; i < 6; i++)
						output2[i] = new long double[1];

					// To store inverse(A*A_Trans)
					long double **output3 = new long double*[6];
					for (int i = 0; i < 6; i++)
						output3[i] = new long double[6];
					matrixTranspose(equations, equationTrans, 6, 10);
					matrixMultiply(equationTrans, equations, output1, 6,10, 6);
					
					matrixMultiply(equationTrans, Z_t, output2, 6, 10, 1);
					

					if (CalcDeterminant(output1, 6) != 0)
					{
						MatrixInversion(output1, 6, output3);

						for (int i = 0; i < 6; i++)
							delete[] equationTrans[i];
						delete[] equationTrans;

						for (int i = 0; i < 10; i++)
							delete[] equations[i];
						delete[] equations;

						for (int i = 0; i < 10; i++)
							delete[] Z_t[i];
						delete[] Z_t;

						for (int i = 0; i < 6; i++)
							delete[] output1[i];
						delete[] output1;

						long double **ans = new long double*[6];
						for (int i = 0; i < 6; i++)
							ans[i] = new long double[1];

						matrixMultiply(output3 , output2, ans, 6, 6, 1);

						for (int i = 0; i < 6; i++)
							delete[] output2[i];
						delete[] output2;


						for (int i = 0; i < 6; i++)
							delete[] output3[i];
						delete[] output3;
						
						for (int i = 0; i < 6; i++)
						{
							cout << ans[i][0] << " ";
						}
						cout << endl;
						stringstream s;
						s << "Depth : ";
						s << depthArray[top_coord*512+left_coord];
						cv::putText(colorMat, s.str(), cv::Point2f(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream t;
						t << "U: ";
						t << ans[0][0];
						cv::putText(colorMat, t.str(), cv::Point2f(10, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream u;
						u << "V: ";
						u << ans[1][0];
						cv::putText(colorMat, u.str(), cv::Point2f(10, 70), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream v;
						v << "W: ";
						v << ans[2][0];
						cv::putText(colorMat, v.str(), cv::Point2f(10, 90), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream w;
						w << "A: ";
						w << ans[3][0];
						cv::putText(colorMat, w.str(), cv::Point2f(10, 110), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream x;
						x << "B: ";
						x << ans[4][0];
						cv::putText(colorMat, x.str(), cv::Point2f(10, 130), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						stringstream y;
						y << "C: ";
						y << ans[5][0];
						cv::putText(colorMat, y.str(), cv::Point2f(10, 150), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
						for (int i = 0; i < 6; i++)
						{
							old_ans[i][0] = ans[i][0];
						}
					}
				}
			}
		}
		else
		{
			cv::rectangle(colorMat, cv::Point(0, 0), cv::Point(150, 170), cv::Scalar(255, 255, 255), CV_FILLED, 4);
			

			stringstream s;
			s << "Depth : ";
			s << depthArray[old_top*512 + old_left];
			cv::putText(colorMat, s.str(), cv::Point2f(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream t;
			t << "U: ";
			t << old_ans[0][0];
			cv::putText(colorMat, t.str(), cv::Point2f(10, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream u;
			u << "V: ";
			u << old_ans[1][0];
			cv::putText(colorMat, u.str(), cv::Point2f(10, 70), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream v;
			v << "W: ";
			v << old_ans[2][0];
			cv::putText(colorMat, v.str(), cv::Point2f(10, 90), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream w;
			w << "A: ";
			w << old_ans[3][0];
			cv::putText(colorMat, w.str(), cv::Point2f(10, 110), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream x;
			x << "B: ";
			x << old_ans[4][0];
			cv::putText(colorMat, x.str(), cv::Point2f(10, 130), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
			stringstream y;
			y << "C: ";
			y << old_ans[5][0];
			cv::putText(colorMat, y.str(), cv::Point2f(10, 150), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0, 0), 1);
		}
		old_left = left_coord;
		old_top = top_coord;
		oldTopLeft = topLeft;
		old_time = curr_time;
		ctr++;
		
		for (int i = 0; i < 512 * 424; i++)
			oldArray[i] = depthArray[i];
		delete[] depthArray;

		cv::imshow("Color", colorMat);
		if (cv::waitKey(30) == VK_ESCAPE)
			break;
	}

	SafeRelease(pDepthSource);
	SafeRelease(pDepthReader);
	SafeRelease(pDepthDescription);

	if (pSensor)
		pSensor->Close();

	SafeRelease(pSensor);

	cv::destroyAllWindows();

	return 0;
 }