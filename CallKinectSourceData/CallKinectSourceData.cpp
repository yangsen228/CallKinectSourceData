#pragma once

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Windows.h>
#include "Kinect.h"

#include <Mmsystem.h>
#pragma comment( lib,"winmm.lib" )

using namespace std;

int main()
{
	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		cerr << "Error: GetDefaultKinectSensor" << endl;
		return -1;
	}
	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor::Open()" << endl;
		return -1;
	}

	// Source Color
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor::get_ColorFrameSource()" << endl;
		return -1;
	}
	// Source Body
	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor::get_BodyFrameSource()" << endl;
		return -1;
	}

	// Reader Color
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		cerr << "Error: IColorFrameSource::OpenReader()" << endl;
		return -1;
	}
	// Reader Body
	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		cerr << "Error: IBodyFrameSource::OpenReader()" << endl;
		return -1;
	}

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		cerr << "Error: IKinectSensor::get_CoordinateMapper()" << endl;
		return -1;
	}
	
	// Get color image
	int width = 1920;
	int height = 1080;
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);
	cv::Mat bufferMat(height, width, CV_8UC4);
	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("Body");
	// Color Table
	cv::Vec3b color[6];
	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	bool capture = TRUE;
	int fps = 0;
	int totalTime = 0;

	while (1) {
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)) {
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
		}
		SafeRelease(pColorFrame);

		// Body Frame
		DWORD timeBegin = timeGetTime();
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)) {
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)) {
				for (int count = 0; count < BODY_COUNT; count++) {
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					if (SUCCEEDED(hResult) && bTracked) {
						Joint joint[JointType::JointType_Count];
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
						if (SUCCEEDED(hResult)) {
							//Left Hand State
							//HandState leftHandState = HandState::HandState_Unknown;
							//hResult = pBody[count]->get_HandLeftState(&leftHandState);
							//if (SUCCEEDED(hResult)) {
							//	ColorSpacePoint colorSpacePoint = { 0 };
							//	hResult = pCoordinateMapper->MapCameraPointToColorSpace(joint[JointType::JointType_HandLeft].Position, &colorSpacePoint);
							//	if (SUCCEEDED(hResult)) {
							//		int x = static_cast<int>(colorSpacePoint.X);
							//		int y = static_cast<int>(colorSpacePoint.Y);
							//		if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
							//			if (leftHandState == HandState::HandState_Open) {
							//				cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 128, 0), 5, CV_AA);
							//			}
							//			else if (leftHandState == HandState::HandState_Closed) {
							//				cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(0, 0, 128), 5, CV_AA);
							//			}
							//			else if (leftHandState == HandState::HandState_Lasso) {
							//				cv::circle(bufferMat, cv::Point(x, y), 75, cv::Scalar(128, 128, 0), 5, CV_AA);
							//			}
							//		}
							//	}
							//}
							for (int type = 0; type < JointType::JointType_Count; type++) {
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
								int x = static_cast<int>(colorSpacePoint.X);
								int y = static_cast<int>(colorSpacePoint.Y);
								if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
									cv::circle(bufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[1]), -1, CV_AA);
								}
							}
						}
					}
				}
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
				//while (capture) {
				//	cv::imwrite("2.jpg", bodyMat);
				//	capture = FALSE;
				//}
			}
		}
		SafeRelease(pBodyFrame);
		DWORD timeEnd = timeGetTime();
		int diffTime = timeEnd - timeBegin;
		totalTime += diffTime;
		fps += 1;
		if (totalTime > 1000) {
			cout << "fps = " << fps << endl;
			fps = 0;
			totalTime = 0;
		}
		cout << "Running time per frame(ms) = " << diffTime << endl;

		// Show the result
		cv::imshow("Body", bodyMat);
		if (cv::waitKey(10) == VK_ESCAPE) {
			break;
		}
	}
	return 0;
}