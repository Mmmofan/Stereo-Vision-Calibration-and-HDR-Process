# HDR-in-stereo-vision-detecting-distance
Based on Windows10 + OpenCV3.4.0 + VS2017

### Description
This is for calibrate the stereo cameras and process high dynamic range photoes (Based on mixr26's code but no vision part)
It can help people to get more details or informations when the background dynamic is too big
Feature detecting is on another repo (see Light-Detect-on-HDR-images)

### What to do?
I use this to get stero cameras' matrix and store them into 5 files: distort_Matrix_Left, distort_Matrix_Right, intrinsic_Matrix_Left, intrinsic_Matrix_Right
When processing images taken by those cameras I can directly use those matrix files and get the right answer

### How to run?
Make sure you are using Visual Studio 2017 and OpenCV 3.4.0, just run main.cpp, and all set
