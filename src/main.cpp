#include "../inc/StereoCalib.hpp"
#include "../inc/StereoVision.hpp"
#include "../inc/HDR.h"
#include <iostream>

void StereoCalibrate();
void StereoVisionDetect(const std::string left, const std::string right);
void HdrProcess(static const float tmArray[], static const char* fname[], std::string savedName );

int main() {
	//Calibrate cameras
	//StereoCalibrate();

	
	//Do HDR process
	/*
	static const float timeArrayLeft[] = { 0.02, 0.04, 0.06, 0.08 };
	static const char* fileNameLeft[] = { "../exposure pics/9_20ms.jpg", "../exposure pics/9_40ms.jpg", 
		"../exposure pics/9_60ms.jpg", "../exposure pics/9_80ms.jpg" };
	std::string savedNameLeft("HDR_Left.jpg");
	HdrProcess(timeArrayLeft, fileNameLeft, savedNameLeft);
	static const float timeArrayRight[] = { 0.02, 0.04, 0.06, 0.08 };
	static const char* fileNameRight[] = { "../exposure pics/11_20ms.jpg", "../exposure pics/11_40ms.jpg",
	"../exposure pics/11_60ms.jpg", "../exposure pics/11_80ms.jpg" };
	std::string savedNameRight("HDR_Right.jpg");
	HdrProcess(timeArrayRight, fileNameRight, savedNameRight);
	*/

	//Detect distance
	//const std::string left_file_direct = "../test_photos/left-1.jpeg";
	//const std::string right_file_direct = "../test_photos/right-1.jpg";
	const std::string left_file_direct = "../Project1/HDR_Left.jpg";
	const std::string right_file_direct = "../Project1/HDR_Right.jpg";
	StereoVisionDetect(left_file_direct, right_file_direct);


	//Maintain the window, no meaning.
	int k;
	std::cin >> k;
	return 0;
}

void StereoCalibrate() {
	/*Images input to calibrate the camares*/
	const std::string left_file_direct = "../additional_files/mini-left.txt";
	const std::string right_file_direct = "../additional_files/mini-right.txt";
	StereoCalib sc(left_file_direct, right_file_direct);
	sc.leftCameraCalibrate();
	sc.leftCameraUndistort();
	sc.rightCameraCalibrate();
	sc.rightCameraUndistort();
	sc.stereoCalibrateAndRectify();
	std::cout << "QMatrix has been wrote" << std::endl;
	cv::waitKey(0);
}

void StereoVisionDetect(const std::string left, const std::string right) {
	/*Detect the coordinate of the object*/
	StereoVision sv(left, right);

	sv.getQMatrix();
	std::cout << "X, Y and Z coordinates of a real-world object is: " << std::endl;

	std::cout << "\t" << sv.calculateDistance(sv.detectObject()) << std::endl;
}

void HdrProcess(static const float tmArray[], static const char* fname[], std::string savedName) {
	HDR sample(tmArray, fname, savedName);
	/*Preprocess*/
	sample.alignImages();
	sample.cameraResponseFunction();
	sample.mergeImages();

	/*Choose one tonemapping algorithm, Reinhard is better (I think) */
	sample.toneMap_Reinhard();
	//sample.toneMap_Drago();
	//sample.toneMap_Durand();
	//sample.toneMap_Mantiuk();
}