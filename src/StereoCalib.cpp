#include "../inc/StereoCalib.hpp"
#include <opencv2/calib3d/calib3d.hpp>

StereoCalib::StereoCalib(std::string leftFile, std::string rightFile) {
    /* Open the files containing image names */
    m_leftImagesFileName.assign(leftFile);
    m_rightImagesFileName.assign(rightFile);

    m_leftImagesFilestream.open(m_leftImagesFileName);
    m_rightImagesFilestream.open(m_rightImagesFileName);

	/*Our project Board size is 9 * 12 */
	m_numCornersHor =11;
	m_numCornersVer = 8;
    m_boardSize = cv::Size(m_numCornersHor, m_numCornersVer);

    /* Initialize intrinsic coeffs matrices */
    m_intrinsicCoeffsLeft = (cv::Mat_<float>(3, 3)<<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    m_intrinsicCoeffsRight = (cv::Mat_<float>(3, 3) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	float ii = 1;
	float jj = 1;
    m_intrinsicCoeffsLeft.ptr<float>(0)[0] = ii;
    m_intrinsicCoeffsRight.ptr<float>(0)[0] = jj;
	std::cout << m_intrinsicCoeffsLeft << std::endl;

    /* Populate 2D points matrix */
    for (int i = 0; i < m_numCornersVer; i++) {
        for (int j = 0; j < m_numCornersHor; j++) {
			/*Point's coordinate in Stack*/
            m_objectPoint.push_back(cv::Point3f(float(j * SQUARE_SIZE), float(i * SQUARE_SIZE), 0.0f));
			//std::cout <<"m_objectPoint: "<< m_objectPoint << ", "<<std::endl;
        }
    }
}

StereoCalib::~StereoCalib() {
    m_leftImagesFilestream.close();
    m_rightImagesFilestream.close();
}

bool StereoCalib::leftCameraCalibrate(void) {
    cv::Mat imageL;
    cv::Mat grayImage;
    std::string imageNameL;

    m_objectPoints.clear();

    while (!m_leftImagesFilestream.eof()) {
        bool found;

        /* Load image into the matrix */
        m_leftImagesFilestream >> imageNameL;

        imageL = cv::imread(imageNameL.c_str());
        if (!imageL.empty()) {
            cv::cvtColor(imageL, grayImage, CV_BGR2GRAY);
        } 
		else {
            std::cout << "Error reading left image: " << imageNameL << std::endl;
            return false;
        }

        /* Check for chessboard corners */
        found = cv::findChessboardCorners(imageL, m_boardSize, m_cornersLeft, CV_CALIB_CB_ADAPTIVE_THRESH
                | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            std::cout << "Checkerboard(left) detected on image: " << imageNameL << std::endl;
			
			//Mark ÑÇÏñËØ¼¶¼ì²â½Çµã
            cv::cornerSubPix(grayImage, m_cornersLeft, cv::Size(	11, 9), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(imageL, m_boardSize, m_cornersLeft, found);

            m_imageSize = imageL.size();
            m_imagePointsLeft.push_back(m_cornersLeft);
            m_objectPoints.push_back(m_objectPoint);
			/*Show the corners on boarder*/
			//cvNamedWindow("boarderLeft");
			//cv::imshow("boarderLeft", imageL);
        } 
		else {
            std::cout << "Checkerboard(left) NOT detected on image: " << imageNameL << std::endl;
            return false;
        }
    }

    /* Calibrate using openCV function */
    cv::calibrateCamera(m_objectPoints, m_imagePointsLeft, m_imageSize, m_intrinsicCoeffsLeft,
            m_distortionCoeffsLeft, m_RVectorsLeft, m_TVectorsLeft);
    std::cout << "Focal length left: " << m_intrinsicCoeffsLeft.at<double>(0, 0) << std::endl;

    return true;
}

bool StereoCalib::rightCameraCalibrate(void) {
    cv::Mat imageR;
    cv::Mat grayImage;
    std::string imageNameR;

    m_objectPoints.clear();

    while (!m_rightImagesFilestream.eof()) {
        bool found;

        /* Load image into the matrix */
        m_rightImagesFilestream >> imageNameR;

        imageR = cv::imread(imageNameR.c_str());
        if (!imageR.empty()) {
            cv::cvtColor(imageR, grayImage, CV_BGR2GRAY);
        } 
		else {
            std::cout << "Error reading right image: " << imageNameR << std::endl;
            return false;
        }

        /* Check for chessboard corners */
        found = cv::findChessboardCorners(imageR, m_boardSize, m_cornersRight, CV_CALIB_CB_ADAPTIVE_THRESH
                | CV_CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            std::cout << "Checkerboard(right) detected on image: " << imageNameR << std::endl;

            cv::cornerSubPix(grayImage, m_cornersRight, cv::Size(13, 13), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(grayImage, m_boardSize, m_cornersRight, found);

            m_imageSize = imageR.size();
            m_imagePointsRight.push_back(m_cornersRight);
            m_objectPoints.push_back(m_objectPoint);
			/*Show the corners on boarder*/
			//cvNamedWindow("boarderRight");
			//cv::imshow("boarderRight", imageR);
        } 
		else {
            std::cout << "Checkerboard(right) NOT detected on image: " << imageNameR << std::endl;
            return false;
        }
    }

    /* Calibrate using openCV function */
    cv::calibrateCamera(m_objectPoints, m_imagePointsRight, imageR.size(), m_intrinsicCoeffsRight,
            m_distortionCoeffsRight, m_RVectorsRight, m_TVectorsRight);
    std::cout << "Focal length right: " << m_intrinsicCoeffsRight.at<double>(0, 0) << std::endl;

    return true;
}

bool StereoCalib::leftCameraUndistort(void) {
    cv::Mat distortedImage;
    cv::Mat undistortedImage;
    std::string imageName;

    /* Get the first image*/
    m_leftImagesFilestream.clear();
    m_leftImagesFilestream.seekg(0);
    m_leftImagesFilestream >> imageName;
    distortedImage = cv::imread(imageName.c_str());

    if (distortedImage.empty()) {
        return false;
    }
    /* Undistort the image */
    cv::undistort(distortedImage, undistortedImage, m_intrinsicCoeffsLeft, m_distortionCoeffsLeft);
	std::cout << "intrinsic matrix left: " << std::endl <<m_intrinsicCoeffsLeft << std::endl;
	std::cout << "distortion matrix left: " << std::endl <<m_distortionCoeffsLeft << std::endl;

    /* Show both distorted and undistorted */
    //cv::imshow("Left Distorted", distortedImage);
   // cv::imshow("Left Undistorted", undistortedImage);
    //cv::waitKey(0);

    return true;
}

bool StereoCalib::rightCameraUndistort(void) {
    cv::Mat distortedImage;
    cv::Mat undistortedImage;
    std::string imageName;

    /* Get the first image*/
    m_rightImagesFilestream.clear();
    m_rightImagesFilestream.seekg(0);
    m_rightImagesFilestream >> imageName;
    distortedImage = cv::imread(imageName.c_str());
    if (distortedImage.empty()) {
        return false;
    }

    /* Undistort the image */
    cv::undistort(distortedImage, undistortedImage, m_intrinsicCoeffsRight, m_distortionCoeffsRight);
	std::cout << "intrinsic matrix right: " << std::endl << m_intrinsicCoeffsRight << std::endl;
	std::cout << "distortion matrix right: " << std::endl << m_distortionCoeffsRight << std::endl;

    /* Show both distorted and undistorted */

    //cv::imshow("Right Distorted", distortedImage);
    //cv::imshow("Right Undistorted", undistortedImage);
    //cv::waitKey(0);

    return true;
}

bool StereoCalib::stereoCalibrateAndRectify(void) {
    /* Stereo calibration parameters */
    cv::Mat R = cv::Mat(3, 3, CV_64F);
    cv::Mat T = cv::Mat(3, 1, CV_64F);
    cv::Mat E = cv::Mat(3, 3, CV_64F);
    cv::Mat F = cv::Mat(3, 3, CV_64F);

    /* Stereo rectification parameters */
    cv::Mat R1 = cv::Mat(3, 3, CV_64F);
    cv::Mat R2 = cv::Mat(3, 3, CV_64F);
    cv::Mat P1 = cv::Mat(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat(3, 4, CV_64F);
    m_qMatrix = cv::Mat(4, 4, CV_64F);

    cv::stereoCalibrate(m_objectPoints, 
			m_imagePointsLeft, m_imagePointsRight,
			m_intrinsicCoeffsLeft, m_distortionCoeffsLeft, 
			m_intrinsicCoeffsRight, m_distortionCoeffsRight, 
			m_imageSize, R, T, E, F, 
			CV_CALIB_FIX_INTRINSIC,
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1e-5)
			);

	std::cout << "Rotate matrix R is: " << std::endl << R << std::endl;
	std::cout << "Translation matrix T is: " << std::endl << T << std::endl;
	std::cout << "Intrinsic matrix E is: " <<std::endl << E << std::endl;
	std::cout << "Basic matrix F is: " << std::endl << F << std::endl;

    cv::stereoRectify(m_intrinsicCoeffsLeft, m_distortionCoeffsLeft, m_intrinsicCoeffsRight,
            m_distortionCoeffsRight, m_imageSize, R, T, R1, R2, P1, P2, m_qMatrix, CV_CALIB_ZERO_DISPARITY);

    outputQMatrix();

    return true;
}

void StereoCalib::outputQMatrix(void) {
    std::ofstream qOutput;

    /* Open Q matrix output file */
    qOutput.open("../additional_files/q_matrix");

    /* Write Q matrix to file */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            qOutput << m_qMatrix.at<double>(i, j) << " ";
        }
        qOutput << std::endl;
    }

    qOutput.close();
}
