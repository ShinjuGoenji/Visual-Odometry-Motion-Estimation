#pragma once
#include<opencv2/opencv.hpp>
#include<opencv2/core/core_c.h>
#include<opencv2/calib3d/calib3d_c.h>
#include"epnp.h"
//#include<opencv2/../../../sources/modules/calib3d/src/calib3d_c_api.h>
//#include "cvRANSAC.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
class Dataset_Handler;
#include "Dataset_Handler.h"
using namespace std;
//using namespace cv;


#define BM 0
#define SGBM 1

#define Sift 0
#define Orb 1
#define Surf 2

#define BF 1
#define FLANN 0

class Methods
{
public:
	cv::Mat data;
	void read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1);
	vector<cv::Mat> groundTruthTrajectory(const char* file_name, int data_num);
	cv::Mat computeLeftDisparityMap(cv::Mat img_left, cv::Mat img_right, int matcher_name, bool rgb);
	void decompose_Projection_Matrix(cv::Mat p, cv::Mat* k, cv::Mat* r, cv::Mat* t);
	cv::Mat calc_depth_map(cv::Mat disp_left, cv::Mat k_left, cv::Mat t_left, cv::Mat t_right, bool rectified);
	cv::Mat stereo_2_depth(cv::Mat img_left, cv::Mat img_right, cv::Mat P0, cv::Mat P1, bool matcher, bool rgb, bool rectified);
	cv::Mat extract_features(cv::Mat image, int detector, cv::Mat mask, vector<cv::KeyPoint>* kp);
	vector<vector<cv::DMatch>> match_features(cv::Mat des1, cv::Mat des2, bool matching, int detector, bool sorting, int  k);
	vector<vector<cv::DMatch>> filter_matches_distance(vector<vector<cv::DMatch>> matches, float dist_threshold);
	void visualize_matches(cv::Mat image1, vector<cv::KeyPoint> kp1, cv::Mat image2, vector<cv::KeyPoint> kp2, vector<vector<cv::DMatch>> match);
	void estimate_motion(vector<vector<cv::DMatch>> match, vector<cv::KeyPoint> kp1, vector<cv::KeyPoint> kp2, cv::Mat k, cv::Mat depth1, int max_depth,
		cv::Mat& rmat, cv::Mat& tvec, cv::Mat& image1_points, cv::Mat& image2_points);
	vector<cv::Mat> visual_odometry(Dataset_Handler handler, int detector, bool matching, float filter_match_distance, bool stereo_matcher, int subset, cv::Mat mask);

	vector<vector<cv::DMatch>> orbKNNmatch(cv::Mat des1, cv::Mat des2);
	bool solvePnPRansac_manaul(cv::Mat opoints, cv::Mat ipoints,
		cv::Mat cameraMatrix, cv::Mat distCoeffs,
		cv::Mat _rvec, cv::Mat _tvec, bool useExtrinsicGuess,
		int iterationsCount, float reprojectionError, double confidence,
		cv::Mat _inliers, int flags);


	bool __solvePnPRansac__(cv::Mat opoints, cv::Mat ipoints,
		cv::Mat cameraMatrix, cv::Mat distCoeffs,
		cv::Mat& _rvec, cv::Mat& _tvec, bool useExtrinsicGuess,
		int iterationsCount, float reprojectionError, double confidence,
		int flags);
	bool __solvePnPRansac2__(cv::InputArray opoints, cv::InputArray ipoints,
		cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
		cv::Mat& _rvec, cv::Mat& _tvec, bool useExtrinsicGuess,
		int iterationsCount, float reprojectionError, double confidence,
		int flags);
};

