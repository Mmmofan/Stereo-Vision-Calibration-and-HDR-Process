#include"../inc/HDR.h"

HDR::HDR(static const float timesArray[], static const char* filenames[], std::string savedName) {
	/*Number of input images*/
	numImages = 4;
	times.assign(timesArray, timesArray + numImages);
	savedFileName = savedName;
	for (int i = 0; i < numImages; i++) {
		cv::Mat im = cv::imread(filenames[i]);
		images.push_back(im);
		std::cout << "Image read: " << filenames[i] << std::endl;
	}
}

void HDR::alignImages() {
	cv::Ptr<cv::AlignMTB> alignMTB = cv::createAlignMTB();
	alignMTB->process(images, images);
}

void HDR::cameraResponseFunction() {
	cv::Ptr<cv::CalibrateDebevec> calibrateDebevec = cv::createCalibrateDebevec();
	calibrateDebevec->process(images, responseDebevec, times);
}

void HDR::mergeImages() {
	/*Merge images into an HDR linear image*/
	cv::Ptr<cv::MergeDebevec> mergeDebevec = cv::createMergeDebevec();
	mergeDebevec->process(images, hdrDebevec, times, responseDebevec);
	/*Save HDR images*/
	cv::imwrite("hdrDebevec.hdr", hdrDebevec);
}

void HDR::toneMap_Reinhard() {
	cv::Mat ldrReinhard;
	/*eateTonemapReinhard has 3 attributes: */
	cv::Ptr<cv::TonemapReinhard> tonemapReinhard = cv::createTonemapReinhard(1.5, 0, 0, 0);
	tonemapReinhard->process(hdrDebevec, ldrReinhard);
	cv::imwrite(savedFileName, ldrReinhard * 255);
	cv::imshow("Reinhard ldr", ldrReinhard);
	cv::waitKey(0);
}

void HDR::toneMap_Drago() {
	cv::Mat ldrDrago;
	/*createTonemapDrago has 3 attributes: gamma, saturation, bias*/
	cv::Ptr<cv::TonemapDrago> tonemapDrago = cv::createTonemapDrago(1.1, 0.7, 0.9);
	tonemapDrago->process(hdrDebevec, ldrDrago);
	ldrDrago = 3 * ldrDrago;
	cv::imwrite(savedFileName, ldrDrago * 255);
	cv::imshow("Drago ldr", ldrDrago);
	cv::waitKey(0);
}

void HDR::toneMap_Durand() {
	cv::Mat ldrDurand;
	/*createTonemapDurand has 5 attributes: gamma, contrast, saturation, sigma space, sigma color*/
	cv::Ptr<cv::TonemapDurand> tonemapDurand = cv::createTonemapDurand(1.5, 4, 1.0, 1, 1);
	tonemapDurand->process(hdrDebevec, ldrDurand);
	ldrDurand = 3 * ldrDurand;
	cv::imwrite(savedFileName, ldrDurand * 255);
	cv::imshow("Durand ldr", ldrDurand);
	cv::waitKey(0);
}

void HDR::toneMap_Mantiuk() {
	cv::Mat ldrMantiuk;
	/*createTonemapMantiuk has 3 attributes: gamma, scale, saturation*/
	cv::Ptr<cv::TonemapMantiuk> tonemapMantiuk = cv::createTonemapMantiuk(1.1, 0.85, 1.1);
	tonemapMantiuk->process(hdrDebevec, ldrMantiuk);
	ldrMantiuk = 3 * ldrMantiuk;
	cv::imwrite(savedFileName, ldrMantiuk * 255);
	cv::imshow("Mantiuk ldr", ldrMantiuk);
	cv::waitKey(0);
}
