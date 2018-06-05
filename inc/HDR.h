#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
class HDR
{
public:
	HDR(static const float timesArray[], static const char* filenames[], std::string savedName);
	void alignImages();
	void cameraResponseFunction();
	void mergeImages();
	void toneMap_Reinhard();
	void toneMap_Drago();
	void toneMap_Durand();
	void toneMap_Mantiuk();

private:
	std::vector<cv::Mat> images;
	std::vector<float> times;
	cv::Mat responseDebevec;
	cv::Mat hdrDebevec;
	int numImages;
	std::string savedFileName;
};