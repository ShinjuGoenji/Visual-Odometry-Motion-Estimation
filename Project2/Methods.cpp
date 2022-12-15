#include "Methods.h"
using namespace cv;

void Methods::read_calib(const char* filePath, cv::Mat* P0, cv::Mat* P1)
/*******************************************************************************
	Read the calibration file from 'filePath', and return projection matices
	of camera0 and camera1 as P0 and P1.

	Arguments:
		filePath -- file path of the calibration file
		P0 -- pointer to projection matrix of camera0
		P1 -- pointer to projection matrix of camera1

*******************************************************************************/
{
	FILE* fp;
	fopen_s(&fp, filePath, "r");
	char* next_token1 = NULL;
	char* next_token2 = NULL;

	*P0 = cv::Mat(3, 4, CV_32F);
	*P1 = cv::Mat(3, 4, CV_32F);

	if (!fp)
	{
		printf("Could not open the calibration file\n");
	}

	int count = 0;
	bool p;
	char content[1024];
	while (fgets(content, 1024, fp))
	{
		char* v = strtok_s(content, " ,", &next_token1);
		while (v)
		{
			if (--count > 0)
			{
				istringstream os(v);
				float d;
				os >> d;
				if (p)
					P1->at<float>((12 - count) / 4, (12 - count) % 4) = d;
				else
					P0->at<float>((12 - count) / 4, (12 - count) % 4) = d;
			}
			if (!strcmp(v, "P0:"))
			{
				count = 13;
				p = 0;
			}
			else if (!strcmp(v, "P1:"))
			{
				count = 13;
				p = 1;
			}
			v = strtok_s(NULL, " ,", &next_token1);
		}
	}

	fclose(fp);
}


vector<Mat> Methods::groundTruthTrajectory(const char* filePath, int data_num)
/*******************************************************************************
	Read the ground truth poses data from 'filePath', return Matrices of
	ground truth poses in a vector.

	Arguments:
		filePath -- file path of the poses data

	Return:
		poses -- a vector of poses in the form of matrix

*******************************************************************************/
{
	vector<Mat> poses;
	FILE* fp;
	fopen_s(&fp, filePath, "r");
	int cols = 12;
	for (int i = 0; i < data_num; i++)
	{
		Mat mat_i = Mat(3, 4, CV_32F);
		for (int j = 0; j < cols; j++)
		{
			fscanf_s(fp, "%e", &mat_i.at<float>(j / 4, j % 4));
		}
		poses.push_back(mat_i);
	}

	fclose(fp);
	return poses;
}


Mat Methods::computeLeftDisparityMap(Mat img_left, Mat img_right, int matcher_name, bool rgb)
/***************************************************************************************
	Takes a left and right stereo pair of images and computes the disparity
	map for the left image. Pass rgb = true if the images are RGB.

	Arguments:
		img_left -- image from left camera
		img_right -- image from right camera

	Optional Arguments:
		matcher -- (bool) can be 'BM' for StereoBM or 'SGBM' for StereoSGBM matching
		rgb -- (bool) set to true if passing RGB images as input

	Returns:
		disp_left -- disparity map for the left camera image

***************************************************************************************/
{
	// Feel free to read OpenCV documentationand tweak these values.These work well
	int sad_window = 6;
	int num_disparities = sad_window * 16;
	int block_size = 11;

	Ptr<StereoMatcher> matcher;
	Mat disp_left;

	if (matcher_name == BM)
	{
		matcher = StereoBM::create(num_disparities, block_size);
	}
	else if (matcher_name == SGBM)
	{
		matcher = StereoSGBM::create(0, num_disparities, block_size, 8 * 3 * pow(sad_window, 2), 32 * 3 * pow(sad_window, 2), 0, 0, 0, 0, 0, StereoSGBM::MODE_SGBM_3WAY);
	}

	if (rgb)
	{
		cvtColor(img_left, img_left, COLOR_BGR2GRAY);
		cvtColor(img_right, img_right, COLOR_BGR2GRAY);
	}

	printf("\n\tComputing disparity map using Stereo%s...\n", (matcher_name == BM) ? "BM" : "SGBM");
	clock_t start = clock();
	if (matcher_name == BM)
	{
		matcher->compute(img_left, img_right, disp_left);
		disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
	}
	else if (matcher_name == SGBM)
	{
		matcher->compute(img_left, img_right, disp_left);
		disp_left.convertTo(disp_left, CV_32F, 1.0 / 16);
	}

	clock_t end = clock();
	printf("\tTime to compute disparity map using Stereo%s: %lld ms\n", (matcher_name == BM) ? "BM" : "SGBM", end - start);

	int x = 300, y = 1200;
	//printf("\ncompare with python tutorial, disp_left[%d, %d] = %f\n\n", x, y, disp_left.at<float>(x, y));

	return disp_left;
}



void Methods::decompose_Projection_Matrix(Mat p, Mat* k, Mat* r, Mat* t)
/***************************************************************************************
	Shortcut to use cv::decomposeProjectionMatrix(), which only returns k, r, t, and
	divides t by the scale, then returns them through pointers

	Arguments:
	p -- projection matrix to be decomposed

	Returns (call by address):
	k, r, t -- intrinsic matrix, rotation matrix, and 3D translation vector

***************************************************************************************/
{
	decomposeProjectionMatrix(p, *k, *r, *t);

	*t = *t / (t->at<float>(3));
}


Mat Methods::calc_depth_map(Mat disp_left, Mat k_left, Mat t_left, Mat t_right, bool rectified)
/***************************************************************************************
	Calculate depth map using a disparity map, intrinsic camera matrix, and translation
	vectors from camera extrinsic matrices(to calculate baseline).
	Note that default behavior is for rectified projection matrix for right camera.
	If using a regular projection matrix, pass rectified = false to avoid issues.

	Arguments:
		disp_left -- disparity map of left camera
		k_left -- intrinsic matrix for left camera
		t_left -- translation vector for left camera
		t_right -- translation vector for right camera
		rectified-- (bool)set to False if t_right is not from rectified projection
					matrix

	Returns :
		depth_map -- calculated depth map for left camera

***************************************************************************************/
{
	Mat depth_map = Mat::ones(disp_left.rows, disp_left.cols, CV_32F);
	disp_left.convertTo(disp_left, CV_32F);

	// Get focal length of x axis for left camera
	float f = k_left.at<float>(0, 0);

	// Calculate baseline of stereo pair
	float b;
	if (rectified)
		b = t_right.at<float>(0, 0) - t_left.at<float>(0, 0);
	else
		b = t_left.at<float>(0, 0) - t_right.at<float>(0, 0);


	for (int i = 0; i < disp_left.rows; i++)
	{
		for (int j = 0; j < disp_left.cols; j++)
		{
			// Avoid instability and division by zero
			if (disp_left.at<float>(i, j) == 0.0 ||
				disp_left.at<float>(i, j) == -1.0)
				disp_left.at<float>(i, j) = 0.1;

			// Make empty depth map then fill with depth
			depth_map.at<float>(i, j) = f * b / disp_left.at<float>(i, j);
		}
	}

	return depth_map;
}


Mat Methods::stereo_2_depth(Mat img_left, Mat img_right, Mat P0, Mat P1, bool matcher, bool rgb, bool rectified)
/***************************************************************************************
	Takes stereo pair of images and returns a depth map for the left camera.If your
	projection matrices are not rectified, set rectified = false.

	Arguments:
		img_left -- image of left camera
		img_right -- image of right camera
		P0 -- Projection matrix for the left camera
		P1 -- Projection matrix for the right camera

	Optional Arguments :
		matcher-- (str)can be 'bm' for StereoBM or 'sgbm' for StereoSGBM
		rgb-- (bool)set to True if images passed are RGB.Default is False
		rectified-- (bool)set to False if P1 not rectified to P0.Default is True

	Returns :
		depth -- depth map for left camera

***************************************************************************************/
{
	// Compute disparity map
	Mat disp = computeLeftDisparityMap(img_left,
		img_right,
		matcher,
		rgb);

	// Decompose projection matrices
	Mat k_left, r_left, t_left;
	Mat k_right, r_right, t_right;
	decompose_Projection_Matrix(P0, &k_left, &r_left, &t_left);
	decompose_Projection_Matrix(P1, &k_right, &r_right, &t_right);

	// Calculate depth map for left camera
	Mat depth = calc_depth_map(disp, k_left, t_left, t_right, true);

	return depth;
}



Mat Methods::extract_features(Mat image, int detector, Mat mask, vector<KeyPoint>* kp)
/***************************************************************************************
	Find keypoints and descriptors for the image

	Arguments :
		image -- a grayscale image
		detector-- (bool)can be 'Sift' or 'Orb'
		mask -- (Mat) mask to reduce feature search area to where depth information
				available.

	Returns :
		kp (call by address) -- list of the extracted keypoints(features) in an image
		des -- list of the keypoint descriptors in an image

***************************************************************************************/
{

	Ptr<Feature2D> det;
	Mat des;
	//Ptr<SURF> det_surf;

	if (detector == Sift)
	{
		det = SIFT::create();
		det->Feature2D::detect(image, *kp, mask);
		det->Feature2D::compute(image, *kp, des);

	}
	else if (detector == Orb)
	{
		det = ORB::create();
		det->Feature2D::detect(image, *kp, mask);
		det->Feature2D::compute(image, *kp, des);
	}


	return des;
}



vector<vector<DMatch>> Methods::match_features(Mat des1, Mat des2, bool matching, int detector, bool sorting, int k)
/***************************************************************************************
	Match features from two images

	Arguments :
		des1 -- list of the keypoint descriptors in the first image
		des2 -- list of the keypoint descriptors in the second image
		matching-- (bool)can be 'BF' for Brute Force or 'FLANN'
		detector-- (int)can be 'Sift or 'Orb'
		sort-- (bool)whether to sort matches by distance.Default is True
		k-- (int)number of neighbors to match to each feature.

	Returns:
		matches -- list of matched features from two images.Each match[i] is k or less
		matches for the same query descriptor
***************************************************************************************/
{
	vector<vector<DMatch>> matches;
	clock_t start = clock();

	if (matching == BF)
	{
		BFMatcher matcher;
		if (detector == Sift)
		{
			matcher.BFMatcher::create(NORM_L2, false);
		}
		else if (detector == Orb)
		{ 
			matcher.BFMatcher::create(NORM_HAMMING2, false);
		}

		matcher.BFMatcher::knnMatch(des1, des2, matches, k);
	}
	else if (matching == FLANN)
	{

		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		matcher->knnMatch(des1, des2, matches, 2);
	}
	clock_t end = clock();

	printf("\tTime to match keypoints using %s: %lld ms\n\n", (matching == BF) ? "BF" : "FLANN", end - start);
	return matches;
}


vector<vector<DMatch>> Methods::orbKNNmatch(Mat des1, Mat des2)
{
	static const uchar tab[] =
	{
		0, 1, 1, 1, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		1, 2, 2, 2, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4,
		2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4, 2, 3, 3, 3, 3, 4, 4, 4, 3, 4, 4, 4, 3, 4, 4, 4
	};
	vector<vector<DMatch>> matches;
	int knn = 2;
	const int IMGIDX_SHIFT = 18;
	const int IMGIDX_ONE = (1 << IMGIDX_SHIFT); // 2^18 = 262144

	matches.reserve(des1.rows); //matches.row = 400

	Mat dist, nidx;

	int dtype = CV_32S;

	dist.create(des1.rows, knn, dtype);
	nidx.create(dist.size(), CV_32S);

	dist = Scalar::all((double)INT_MAX);
	nidx = Scalar::all(-1);

	for (int i = 0; i < des1.rows; i++)
	{
		for (int y = 0; y < des2.rows; y++)
		{
			int d = 0;

			for (int z = 0; z < des2.cols; z++)
				d += tab[des1.at<uchar>(i, z) ^ des2.at<uchar>(y, z)];

			if (d < dist.at<int>(i, knn - 1))
			{
				nidx.at<int>(i, 1) = nidx.at<int>(i, 0);
				dist.at<int>(i, 1) = dist.at<int>(i, 0);
				nidx.at<int>(i, 0) = y;
				dist.at<int>(i, 0) = d;
			}
		}
	}


	Mat temp;
	dist.convertTo(temp, CV_32F);
	dist = temp;

	for (int qIdx = 0; qIdx < des1.rows; qIdx++)
	{
		const float* distptr = dist.ptr<float>(qIdx);
		const int* nidxptr = nidx.ptr<int>(qIdx);

		matches.push_back(vector<DMatch>());
		vector<DMatch>& mq = matches.back();
		mq.reserve(knn);

		for (int k = 0; k < nidx.cols; k++)
		{
			if (nidxptr[k] < 0)
				break;
			mq.push_back(DMatch(qIdx, nidxptr[k] & (IMGIDX_ONE - 1),
				nidxptr[k] >> IMGIDX_SHIFT, distptr[k]));
		}

	}
	return matches;
}


void Methods::visualize_matches(Mat image1, vector<KeyPoint> kp1, Mat image2, vector<KeyPoint> kp2, vector<vector<DMatch>> match)
/***************************************************************************************
	Visualize corresponding matches in two images

	Arguments :
		image1 -- the first image in a matched image pair
		kp1 -- list of the keypoints in the first image
		image2 -- the second image in a matched image pair
		kp2 -- list of the keypoints in the second image
		match -- list of matched features from the pair of images

	Returns :
		image_matches -- an image showing the corresponding matches on both image1 and
		image2 or None if you don't use this function

***************************************************************************************/
{
	Mat image_matches;
	drawMatches(image1, kp1, image2, kp2, match, image_matches);
	imshow("image matches", image_matches);
	waitKey();
	destroyWindow("image matches");
	system("cls");
}


vector<vector<DMatch>> Methods::filter_matches_distance(vector<vector<DMatch>> matches, float dist_threshold)
/***************************************************************************************
	Filter matched features from two images by distance between the best matches

	Arguments :
		match -- list of matched features from two images
		dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)

	Returns :
		filtered_match -- list of good matches, satisfying the distance threshold

***************************************************************************************/
{
	vector<vector<DMatch>> filtered_match;
	for (int m = 0; m < matches.size(); m++)
	{

		if (matches[m][0].distance <= dist_threshold * matches[m][1].distance)
		{
			vector<DMatch> match_i;
			match_i.push_back(matches[m][0]);
			filtered_match.push_back(match_i);
		}
	}
	return filtered_match;
}





void Methods::estimate_motion(vector<vector<DMatch>> match, vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat k, Mat depth1, int max_depth,
	Mat& rmat, Mat& tvec, Mat& image1_points, Mat& image2_points)
	/***************************************************************************************
		Estimate camera motion from a pair of subsequent image frames

		Arguments :
			match -- list of matched features from the pair of images
			kp1 -- list of the keypoints in the first image
			kp2 -- list of the keypoints in the second image
			k -- camera intrinsic calibration matrix
			depth1 -- Depth map of the first frame.Set to None to use Essential Matrix
					decomposition
			max_depth -- Threshold of depth to ignore matched features. 3000 is default

		Returns (call by reference) :
			rmat -- estimated 3x3 rotation matrix
			tvec -- estimated 3x1 translation vector
			image1_points -- matched feature pixel coordinates in the first image.
			image1_points[i] = [u, v]->pixel coordinates of i - th match
			image2_points -- matched feature pixel coordinates in the second image.
			image2_points[i] = [u, v]->pixel coordinates of i - th match
	***************************************************************************************/
{
	Mat image1_points__ = Mat(0, 2, CV_32F);
	image1_points = Mat(0, 2, CV_32F);
	Mat image2_points__ = Mat(0, 2, CV_32F);
	image2_points = Mat(0, 2, CV_32F);
	Mat rvec;// , rvec_test, rmat_test, tvec_test;
	Mat distCoef = Mat::zeros(1, 5, CV_32F);

	for (int m = 0; m < match.size(); m++)
	{
		image1_points__.push_back(kp1[match[m][0].queryIdx].pt);
		image2_points__.push_back(kp2[match[m][0].trainIdx].pt);
	}

	if (!depth1.empty())
	{
		float cx = k.at<float>(0, 2);
		float cy = k.at<float>(1, 2);
		float fx = k.at<float>(0, 0);
		float fy = k.at<float>(1, 1);
		Mat object_points = Mat::zeros(0, 3, CV_32F);

		for (int i = 0; i < image1_points__.rows; i++)
		{

			float u = image1_points__.at<float>(i, 0);
			float v = image1_points__.at<float>(i, 1);
			float z = depth1.at<float>((int)v, (int)u);

			if (z > max_depth)
			{
				continue;
			}


			float x = z * (u - cx) / fx;
			float y = z * (v - cy) / fy;


			Mat vec = Mat(1, 3, CV_32F);
			vec.at<float>(0, 0) = x;
			vec.at<float>(0, 1) = y;
			vec.at<float>(0, 2) = z;

			object_points.push_back(vec);
			image1_points.push_back(image1_points__.row(i));
			image2_points.push_back(image2_points__.row(i));
		}
		//__solvePnPRansac__(object_points, image2_points, k, distCoef, rvec, tvec,
			//false, 100, 8.0, 0.99, SOLVEPNP_ITERATIVE);

		Mat inlier;
		cv::solvePnPRansac(object_points, image2_points, k, distCoef, rvec, tvec,
			false, 1000000000, 8, 0.999999, inlier, SOLVEPNP_ITERATIVE);

		printf("\n======================== # object point: %d ===========================\n", object_points.rows);
		printf("\n======================== # inlier: %d ===========================\n", inlier.rows);

		rmat = Mat::eye(3, 3, CV_32F);
		//rmat_test = Mat::eye(3, 3, CV_32F);
		Rodrigues(rvec, rmat);
		//Rodrigues(rvec_test, rmat_test);

		//cout << "\n\n\nmanual rmat: " << endl << rmat << endl;
		//cout << "test rmat: " << endl << rmat_test << endl;
		//cout << "manual tvect: " << endl << tvec << endl;
		//cout << "test tvect: " << endl << tvec_test << endl;
	}
}


vector<Mat> Methods::visual_odometry(Dataset_Handler handler, int detector, bool matching,
	float filter_match_distance, bool stereo_matcher, int subset, Mat mask)
	/***************************************************************************************
		Function to perform visual odometry on a sequence from the KITTI visual odometry
		dataset.
		Takes as input a Dataset_Handler object and optional parameters.

		Arguments:
			handler -- Dataset_Handler object instance
			detector -- (str) can be 'Sift' or 'Orb'.
			matching -- (str) can be 'BF' for Brute Force or 'FLANN'.
			filter_match_distance -- (float) value for ratio test on matched features.
									Default is None.
			stereo_matcher -- (str) can be 'BM' (faster) or 'SGBM' (more accurate).
			mask -- (array) mask to reduce feature search area to where depth information
						available.
			subset -- (int) number of frames to compute. Defaults to None to compute
							all frames.

		Returns:
			trajectory -- Array of shape Nx3x4 of estimated poses of vehicle for each
						computed frame.
	***************************************************************************************/
{
	int num_frames;

	printf("Generating disparities with Stereo %s\n", stereo_matcher ? "SGBM" : "BM");
	printf("Detecting features with %s and matching with %s\n", (detector == Sift) ? "SIFT" : (detector == Orb) ? "ORB" : "SURF",
		matching ? "BF" : "FLANN");

	if (filter_match_distance)
		printf("Filtering feature matches at threshold of %f * distance\n", filter_match_distance);

	if (subset)
		num_frames = subset;
	else
		num_frames = handler.num_frames;

	Mat T_tot = Mat::eye(4, 4, CV_64F);


	vector<Mat> trajectory(num_frames);
	Rect rect(0, 0, 4, 3);
	T_tot(rect).copyTo(trajectory[0]);

	int imwidth = handler.imwidth;
	int imheight = handler.imheight;

	Mat k_left, r_left, t_left;
	decompose_Projection_Matrix(handler.P0, &k_left, &r_left, &t_left);

	Mat image_plus1 = imread(handler.left_image_files[0], IMREAD_GRAYSCALE);
	Mat image_left, image_right;
	Mat depth;

	for (int i = 0; i < num_frames - 1; i++)
	{
		printf("Computing frame %d\n", i + 1);
		clock_t start = clock();
		image_left = image_plus1;
		image_right = imread(handler.right_image_files[i], IMREAD_GRAYSCALE);
		image_plus1 = imread(handler.left_image_files[i + 1], IMREAD_GRAYSCALE);

		depth = stereo_2_depth(image_left, image_right, handler.P0, handler.P1, stereo_matcher, false, true);


		vector<KeyPoint> kp0, kp1;
		Mat des0 = extract_features(image_left, detector, mask, &kp0);
		Mat des1 = extract_features(image_plus1, detector, mask, &kp1);

		vector<vector<DMatch>> matches_unfilt = match_features(des0, des1, matching, detector, false, 2);

		vector<vector<DMatch>> matches;
		if (filter_match_distance)
			matches = filter_matches_distance(matches_unfilt, filter_match_distance);
		else
			matches = matches_unfilt;

		//visualize_matches(image_left, kp0, image_right, kp1, matches);

		Mat rmat, tvec, img1_points, img2_points;
		estimate_motion(matches, kp0, kp1, k_left, depth, 3000, rmat, tvec, img1_points, img2_points);

		Mat T_mat;
		Mat I4 = Mat::eye(4, 4, CV_64F);
		hconcat(rmat, tvec, T_mat);
		vconcat(T_mat, I4.row(3), T_mat);
		T_tot = T_tot * T_mat.inv();
		T_tot(rect).copyTo(trajectory[i + 1]);
		clock_t end = clock();

		printf("Time to compute frame %d: %lld ms\n\n", i + 1, end - start);
	}

	return trajectory;
}




template <typename FLOAT>
void computeTiltProjectionMatrix(FLOAT tauX,
	FLOAT tauY,
	Matx33d* matTilt = 0,
	Matx33d* dMatTiltdTauX = 0,
	Matx33d* dMatTiltdTauY = 0,
	Matx33d* invMatTilt = 0)
	/*Matx<FLOAT, 3, 3>* matTilt = 0,
	Matx<FLOAT, 3, 3>* dMatTiltdTauX = 0,
	Matx<FLOAT, 3, 3>* dMatTiltdTauY = 0,
	Matx<FLOAT, 3, 3>* invMatTilt = 0)*/
{
	FLOAT cTauX = cos(tauX);
	FLOAT sTauX = sin(tauX);
	FLOAT cTauY = cos(tauY);
	FLOAT sTauY = sin(tauY);
	Matx<FLOAT, 3, 3> matRotX = Matx<FLOAT, 3, 3>(1, 0, 0, 0, cTauX, sTauX, 0, -sTauX, cTauX);
	Matx<FLOAT, 3, 3> matRotY = Matx<FLOAT, 3, 3>(cTauY, 0, -sTauY, 0, 1, 0, sTauY, 0, cTauY);
	Matx<FLOAT, 3, 3> matRotXY = matRotY * matRotX;
	Matx<FLOAT, 3, 3> matProjZ = Matx<FLOAT, 3, 3>(matRotXY(2, 2), 0, -matRotXY(0, 2), 0, matRotXY(2, 2), -matRotXY(1, 2), 0, 0, 1);
	if (matTilt)
	{
		// Matrix for trapezoidal distortion of tilted image sensor
		*matTilt = matProjZ * matRotXY;
	}
	if (dMatTiltdTauX)
	{
		// Derivative with respect to tauX
		Matx<FLOAT, 3, 3> dMatRotXYdTauX = matRotY * Matx<FLOAT, 3, 3>(0, 0, 0, 0, -sTauX, cTauX, 0, -cTauX, -sTauX);
		Matx<FLOAT, 3, 3> dMatProjZdTauX = Matx<FLOAT, 3, 3>(dMatRotXYdTauX(2, 2), 0, -dMatRotXYdTauX(0, 2),
			0, dMatRotXYdTauX(2, 2), -dMatRotXYdTauX(1, 2), 0, 0, 0);
		*dMatTiltdTauX = (matProjZ * dMatRotXYdTauX) + (dMatProjZdTauX * matRotXY);
	}
	if (dMatTiltdTauY)
	{
		// Derivative with respect to tauY
		Matx<FLOAT, 3, 3> dMatRotXYdTauY = Matx<FLOAT, 3, 3>(-sTauY, 0, -cTauY, 0, 0, 0, cTauY, 0, -sTauY) * matRotX;
		Matx<FLOAT, 3, 3> dMatProjZdTauY = Matx<FLOAT, 3, 3>(dMatRotXYdTauY(2, 2), 0, -dMatRotXYdTauY(0, 2),
			0, dMatRotXYdTauY(2, 2), -dMatRotXYdTauY(1, 2), 0, 0, 0);
		*dMatTiltdTauY = (matProjZ * dMatRotXYdTauY) + (dMatProjZdTauY * matRotXY);
	}
	if (invMatTilt)
	{
		FLOAT inv = 1. / matRotXY(2, 2);
		Matx<FLOAT, 3, 3> invMatProjZ = Matx<FLOAT, 3, 3>(inv, 0, inv * matRotXY(0, 2), 0, inv, inv * matRotXY(1, 2), 0, 0, 1);
		*invMatTilt = matRotXY.t() * invMatProjZ;
	}
}

int cvRodrigues2(const CvMat* src, CvMat* dst, CvMat* jacobian)
{
	double J[27] = { 0 };
	CvMat matJ = cvMat(3, 9, CV_64F, J);

	if (!CV_IS_MAT(src))
		CV_Error(!src ? CV_StsNullPtr : CV_StsBadArg, "Input argument is not a valid matrix");

	if (!CV_IS_MAT(dst))
		CV_Error(!dst ? CV_StsNullPtr : CV_StsBadArg,
			"The first output argument is not a valid matrix");

	int depth = CV_MAT_DEPTH(src->type);
	int elem_size = CV_ELEM_SIZE(depth);

	if (depth != CV_32F && depth != CV_64F)
		CV_Error(CV_StsUnsupportedFormat, "The matrices must have 32f or 64f data type");

	if (!CV_ARE_DEPTHS_EQ(src, dst))
		CV_Error(CV_StsUnmatchedFormats, "All the matrices must have the same data type");

	if (jacobian)
	{
		if (!CV_IS_MAT(jacobian))
			CV_Error(CV_StsBadArg, "Jacobian is not a valid matrix");

		if (!CV_ARE_DEPTHS_EQ(src, jacobian) || CV_MAT_CN(jacobian->type) != 1)
			CV_Error(CV_StsUnmatchedFormats, "Jacobian must have 32fC1 or 64fC1 datatype");

		if ((jacobian->rows != 9 || jacobian->cols != 3) &&
			(jacobian->rows != 3 || jacobian->cols != 9))
			CV_Error(CV_StsBadSize, "Jacobian must be 3x9 or 9x3");
	}

	if (src->cols == 1 || src->rows == 1)
	{
		int step = src->rows > 1 ? src->step / elem_size : 1;

		if (src->rows + src->cols * CV_MAT_CN(src->type) - 1 != 3)
			CV_Error(CV_StsBadSize, "Input matrix must be 1x3, 3x1 or 3x3");

		if (dst->rows != 3 || dst->cols != 3 || CV_MAT_CN(dst->type) != 1)
			CV_Error(CV_StsBadSize, "Output matrix must be 3x3, single-channel floating point matrix");

		Point3d r;
		if (depth == CV_32F)
		{
			r.x = src->data.fl[0];
			r.y = src->data.fl[step];
			r.z = src->data.fl[step * 2];
		}
		else
		{
			r.x = src->data.db[0];
			r.y = src->data.db[step];
			r.z = src->data.db[step * 2];
		}

		double theta = norm(r);

		if (theta < DBL_EPSILON)
		{
			cvSetIdentity(dst);

			if (jacobian)
			{
				memset(J, 0, sizeof(J));
				J[5] = J[15] = J[19] = -1;
				J[7] = J[11] = J[21] = 1;
			}
		}
		else
		{
			double c = cos(theta);
			double s = sin(theta);
			double c1 = 1. - c;
			double itheta = theta ? 1. / theta : 0.;

			r *= itheta;

			Matx33d rrt(r.x * r.x, r.x * r.y, r.x * r.z, r.x * r.y, r.y * r.y, r.y * r.z, r.x * r.z, r.y * r.z, r.z * r.z);
			Matx33d r_x(0, -r.z, r.y,
				r.z, 0, -r.x,
				-r.y, r.x, 0);

			// R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
			Matx33d R = c * Matx33d::eye() + c1 * rrt + s * r_x;

			Mat(R).convertTo(cvarrToMat(dst), dst->type);

			if (jacobian)
			{
				const double I[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
				double drrt[] = { r.x + r.x, r.y, r.z, r.y, 0, 0, r.z, 0, 0,
								  0, r.x, 0, r.x, r.y + r.y, r.z, 0, r.z, 0,
								  0, 0, r.x, 0, 0, r.y, r.x, r.y, r.z + r.z };
				double d_r_x_[] = { 0, 0, 0, 0, 0, -1, 0, 1, 0,
									0, 0, 1, 0, 0, 0, -1, 0, 0,
									0, -1, 0, 1, 0, 0, 0, 0, 0 };
				for (int i = 0; i < 3; i++)
				{
					double ri = i == 0 ? r.x : i == 1 ? r.y : r.z;
					double a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
					double a3 = (c - s * itheta) * ri, a4 = s * itheta;
					for (int k = 0; k < 9; k++)
						J[i * 9 + k] = a0 * I[k] + a1 * rrt.val[k] + a2 * drrt[i * 9 + k] +
						a3 * r_x.val[k] + a4 * d_r_x_[i * 9 + k];
				}
			}
		}
	}
	else if (src->cols == 3 && src->rows == 3)
	{
		Matx33d U, Vt;
		Vec3d W;
		double theta, s, c;
		int step = dst->rows > 1 ? dst->step / elem_size : 1;

		if ((dst->rows != 1 || dst->cols * CV_MAT_CN(dst->type) != 3) &&
			(dst->rows != 3 || dst->cols != 1 || CV_MAT_CN(dst->type) != 1))
			CV_Error(CV_StsBadSize, "Output matrix must be 1x3 or 3x1");

		Matx33d R = cvarrToMat(src);

		if (!checkRange(R, true, NULL, -100, 100))
		{
			cvZero(dst);
			if (jacobian)
				cvZero(jacobian);
			return 0;
		}

		SVD::compute(R, W, U, Vt);
		R = U * Vt;

		Point3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

		s = std::sqrt((r.x * r.x + r.y * r.y + r.z * r.z) * 0.25);
		c = (R(0, 0) + R(1, 1) + R(2, 2) - 1) * 0.5;
		c = c > 1. ? 1. : c < -1. ? -1. : c;
		theta = acos(c);

		if (s < 1e-5)
		{
			double t;

			if (c > 0)
				r = Point3d(0, 0, 0);
			else
			{
				t = (R(0, 0) + 1) * 0.5;
				r.x = std::sqrt(MAX(t, 0.));
				t = (R(1, 1) + 1) * 0.5;
				r.y = std::sqrt(MAX(t, 0.)) * (R(0, 1) < 0 ? -1. : 1.);
				t = (R(2, 2) + 1) * 0.5;
				r.z = std::sqrt(MAX(t, 0.)) * (R(0, 2) < 0 ? -1. : 1.);
				if (fabs(r.x) < fabs(r.y) && fabs(r.x) < fabs(r.z) && (R(1, 2) > 0) != (r.y * r.z > 0))
					r.z = -r.z;
				theta /= norm(r);
				r *= theta;
			}

			if (jacobian)
			{
				memset(J, 0, sizeof(J));
				if (c > 0)
				{
					J[5] = J[15] = J[19] = -0.5;
					J[7] = J[11] = J[21] = 0.5;
				}
			}
		}
		else
		{
			double vth = 1 / (2 * s);

			if (jacobian)
			{
				double t, dtheta_dtr = -1. / s;
				// var1 = [vth;theta]
				// var = [om1;var1] = [om1;vth;theta]
				double dvth_dtheta = -vth * c / s;
				double d1 = 0.5 * dvth_dtheta * dtheta_dtr;
				double d2 = 0.5 * dtheta_dtr;
				// dvar1/dR = dvar1/dtheta*dtheta/dR = [dvth/dtheta; 1] * dtheta/dtr * dtr/dR
				double dvardR[5 * 9] =
				{
					0, 0, 0, 0, 0, 1, 0, -1, 0,
					0, 0, -1, 0, 0, 0, 1, 0, 0,
					0, 1, 0, -1, 0, 0, 0, 0, 0,
					d1, 0, 0, 0, d1, 0, 0, 0, d1,
					d2, 0, 0, 0, d2, 0, 0, 0, d2
				};
				// var2 = [om;theta]
				double dvar2dvar[] =
				{
					vth, 0, 0, r.x, 0,
					0, vth, 0, r.y, 0,
					0, 0, vth, r.z, 0,
					0, 0, 0, 0, 1
				};
				double domegadvar2[] =
				{
					theta, 0, 0, r.x * vth,
					0, theta, 0, r.y * vth,
					0, 0, theta, r.z * vth
				};

				CvMat _dvardR = cvMat(5, 9, CV_64FC1, dvardR);
				CvMat _dvar2dvar = cvMat(4, 5, CV_64FC1, dvar2dvar);
				CvMat _domegadvar2 = cvMat(3, 4, CV_64FC1, domegadvar2);
				double t0[3 * 5];
				CvMat _t0 = cvMat(3, 5, CV_64FC1, t0);

				cvMatMul(&_domegadvar2, &_dvar2dvar, &_t0);
				cvMatMul(&_t0, &_dvardR, &matJ);

				// transpose every row of matJ (treat the rows as 3x3 matrices)
				CV_SWAP(J[1], J[3], t); CV_SWAP(J[2], J[6], t); CV_SWAP(J[5], J[7], t);
				CV_SWAP(J[10], J[12], t); CV_SWAP(J[11], J[15], t); CV_SWAP(J[14], J[16], t);
				CV_SWAP(J[19], J[21], t); CV_SWAP(J[20], J[24], t); CV_SWAP(J[23], J[25], t);
			}

			vth *= theta;
			r *= vth;
		}

		if (depth == CV_32F)
		{
			dst->data.fl[0] = (float)r.x;
			dst->data.fl[step] = (float)r.y;
			dst->data.fl[step * 2] = (float)r.z;
		}
		else
		{
			dst->data.db[0] = r.x;
			dst->data.db[step] = r.y;
			dst->data.db[step * 2] = r.z;
		}
	}
	else
	{
		CV_Error(CV_StsBadSize, "Input matrix must be 1x3 or 3x1 for a rotation vector, or 3x3 for a rotation matrix");
	}

	if (jacobian)
	{
		if (depth == CV_32F)
		{
			if (jacobian->rows == matJ.rows)
				cvConvert(&matJ, jacobian);
			else
			{
				float Jf[3 * 9];
				CvMat _Jf = cvMat(matJ.rows, matJ.cols, CV_32FC1, Jf);
				cvConvert(&matJ, &_Jf);
				cvTranspose(&_Jf, jacobian);
			}
		}
		else if (jacobian->rows == matJ.rows)
			cvCopy(&matJ, jacobian);
		else
			cvTranspose(&matJ, jacobian);
	}

	return 1;
}




static void cvProjectPoints2Internal(const CvMat* objectPoints,
	const CvMat* r_vec,
	const CvMat* t_vec,
	const CvMat* A,
	const CvMat* distCoeffs,
	CvMat* imagePoints, CvMat* dpdr CV_DEFAULT(NULL),
	CvMat* dpdt CV_DEFAULT(NULL), CvMat* dpdf CV_DEFAULT(NULL),
	CvMat* dpdc CV_DEFAULT(NULL), CvMat* dpdk CV_DEFAULT(NULL),
	CvMat* dpdo CV_DEFAULT(NULL),
	double aspectRatio CV_DEFAULT(0))
{
	Ptr<CvMat> matM, _m;
	Ptr<CvMat> _dpdr, _dpdt, _dpdc, _dpdf, _dpdk;
	Ptr<CvMat> _dpdo;

	int i, j, count;
	int calc_derivatives;
	const CvPoint3D64f* M;
	CvPoint2D64f* m;
	double r[3], R[9], dRdr[27], t[3], a[9], k[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 }, fx, fy, cx, cy;
	Matx33d matTilt = Matx33d::eye();
	Matx33d dMatTiltdTauX(0, 0, 0, 0, 0, 0, 0, -1, 0);
	Matx33d dMatTiltdTauY(0, 0, 0, 0, 0, 0, 1, 0, 0);
	CvMat _r, _t, _a = cvMat(3, 3, CV_64F, a), _k;
	CvMat matR = cvMat(3, 3, CV_64F, R), _dRdr = cvMat(3, 9, CV_64F, dRdr);
	double* dpdr_p = 0, * dpdt_p = 0, * dpdk_p = 0, * dpdf_p = 0, * dpdc_p = 0;
	double* dpdo_p = 0;
	int dpdr_step = 0, dpdt_step = 0, dpdk_step = 0, dpdf_step = 0, dpdc_step = 0;
	int dpdo_step = 0;
	bool fixedAspectRatio = aspectRatio > FLT_EPSILON;

	if (!CV_IS_MAT(objectPoints) || !CV_IS_MAT(r_vec) ||
		!CV_IS_MAT(t_vec) || !CV_IS_MAT(A) ||
		/*!CV_IS_MAT(distCoeffs) ||*/ !CV_IS_MAT(imagePoints))
		CV_Error(CV_StsBadArg, "One of required arguments is not a valid matrix");

	int total = objectPoints->rows * objectPoints->cols * CV_MAT_CN(objectPoints->type);
	if (total % 3 != 0)
	{
		//we have stopped support of homogeneous coordinates because it cause ambiguity in interpretation of the input data
		CV_Error(CV_StsBadArg, "Homogeneous coordinates are not supported");
	}
	count = total / 3;

	if (CV_IS_CONT_MAT(objectPoints->type) &&
		(CV_MAT_DEPTH(objectPoints->type) == CV_32F || CV_MAT_DEPTH(objectPoints->type) == CV_64F) &&
		((objectPoints->rows == 1 && CV_MAT_CN(objectPoints->type) == 3) ||
			(objectPoints->rows == count && CV_MAT_CN(objectPoints->type) * objectPoints->cols == 3) ||
			(objectPoints->rows == 3 && CV_MAT_CN(objectPoints->type) == 1 && objectPoints->cols == count)))
	{
		matM.reset(cvCreateMat(objectPoints->rows, objectPoints->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(objectPoints->type))));
		cvConvert(objectPoints, matM);
	}
	else
	{
		//        matM = cvCreateMat( 1, count, CV_64FC3 );
		//        cvConvertPointsHomogeneous( objectPoints, matM );
		CV_Error(CV_StsBadArg, "Homogeneous coordinates are not supported");
	}

	if (CV_IS_CONT_MAT(imagePoints->type) &&
		(CV_MAT_DEPTH(imagePoints->type) == CV_32F || CV_MAT_DEPTH(imagePoints->type) == CV_64F) &&
		((imagePoints->rows == 1 && CV_MAT_CN(imagePoints->type) == 2) ||
			(imagePoints->rows == count && CV_MAT_CN(imagePoints->type) * imagePoints->cols == 2) ||
			(imagePoints->rows == 2 && CV_MAT_CN(imagePoints->type) == 1 && imagePoints->cols == count)))
	{
		_m.reset(cvCreateMat(imagePoints->rows, imagePoints->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(imagePoints->type))));
		cvConvert(imagePoints, _m);
	}
	else
	{
		//        _m = cvCreateMat( 1, count, CV_64FC2 );
		CV_Error(CV_StsBadArg, "Homogeneous coordinates are not supported");
	}

	M = (CvPoint3D64f*)matM->data.db;
	m = (CvPoint2D64f*)_m->data.db;

	if ((CV_MAT_DEPTH(r_vec->type) != CV_64F && CV_MAT_DEPTH(r_vec->type) != CV_32F) ||
		(((r_vec->rows != 1 && r_vec->cols != 1) ||
			r_vec->rows * r_vec->cols * CV_MAT_CN(r_vec->type) != 3) &&
			((r_vec->rows != 3 && r_vec->cols != 3) || CV_MAT_CN(r_vec->type) != 1)))
		CV_Error(CV_StsBadArg, "Rotation must be represented by 1x3 or 3x1 "
			"floating-point rotation vector, or 3x3 rotation matrix");

	if (r_vec->rows == 3 && r_vec->cols == 3)
	{
		_r = cvMat(3, 1, CV_64FC1, r);
		cvRodrigues2(r_vec, &_r, 0);
		cvRodrigues2(&_r, &matR, &_dRdr);
		cvCopy(r_vec, &matR);
	}
	else
	{
		_r = cvMat(r_vec->rows, r_vec->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(r_vec->type)), r);
		cvConvert(r_vec, &_r);
		cvRodrigues2(&_r, &matR, &_dRdr);
	}

	if ((CV_MAT_DEPTH(t_vec->type) != CV_64F && CV_MAT_DEPTH(t_vec->type) != CV_32F) ||
		(t_vec->rows != 1 && t_vec->cols != 1) ||
		t_vec->rows * t_vec->cols * CV_MAT_CN(t_vec->type) != 3)
		CV_Error(CV_StsBadArg,
			"Translation vector must be 1x3 or 3x1 floating-point vector");

	_t = cvMat(t_vec->rows, t_vec->cols, CV_MAKETYPE(CV_64F, CV_MAT_CN(t_vec->type)), t);
	cvConvert(t_vec, &_t);

	if ((CV_MAT_TYPE(A->type) != CV_64FC1 && CV_MAT_TYPE(A->type) != CV_32FC1) ||
		A->rows != 3 || A->cols != 3)
		CV_Error(CV_StsBadArg, "Intrinsic parameters must be 3x3 floating-point matrix");

	cvConvert(A, &_a);
	fx = a[0]; fy = a[4];
	cx = a[2]; cy = a[5];

	if (fixedAspectRatio)
		fx = fy * aspectRatio;

	if (distCoeffs)
	{

		_k = cvMat(distCoeffs->rows, distCoeffs->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(distCoeffs->type)), k);
		cvConvert(distCoeffs, &_k);
		if (k[12] != 0 || k[13] != 0)
		{
			computeTiltProjectionMatrix(k[12], k[13],
				&matTilt, &dMatTiltdTauX, &dMatTiltdTauY);
		}
	}

	if (dpdr)
	{
		if (!CV_IS_MAT(dpdr) ||
			(CV_MAT_TYPE(dpdr->type) != CV_32FC1 &&
				CV_MAT_TYPE(dpdr->type) != CV_64FC1) ||
			dpdr->rows != count * 2 || dpdr->cols != 3)
			CV_Error(CV_StsBadArg, "dp/drot must be 2Nx3 floating-point matrix");

		if (CV_MAT_TYPE(dpdr->type) == CV_64FC1)
		{
			_dpdr.reset(cvCloneMat(dpdr));
		}
		else
			_dpdr.reset(cvCreateMat(2 * count, 3, CV_64FC1));
		dpdr_p = _dpdr->data.db;
		dpdr_step = _dpdr->step / sizeof(dpdr_p[0]);
	}

	if (dpdt)
	{
		if (!CV_IS_MAT(dpdt) ||
			(CV_MAT_TYPE(dpdt->type) != CV_32FC1 &&
				CV_MAT_TYPE(dpdt->type) != CV_64FC1) ||
			dpdt->rows != count * 2 || dpdt->cols != 3)
			CV_Error(CV_StsBadArg, "dp/dT must be 2Nx3 floating-point matrix");

		if (CV_MAT_TYPE(dpdt->type) == CV_64FC1)
		{
			_dpdt.reset(cvCloneMat(dpdt));
		}
		else
			_dpdt.reset(cvCreateMat(2 * count, 3, CV_64FC1));
		dpdt_p = _dpdt->data.db;
		dpdt_step = _dpdt->step / sizeof(dpdt_p[0]);
	}

	if (dpdf)
	{
		if (!CV_IS_MAT(dpdf) ||
			(CV_MAT_TYPE(dpdf->type) != CV_32FC1 && CV_MAT_TYPE(dpdf->type) != CV_64FC1) ||
			dpdf->rows != count * 2 || dpdf->cols != 2)
			CV_Error(CV_StsBadArg, "dp/df must be 2Nx2 floating-point matrix");

		if (CV_MAT_TYPE(dpdf->type) == CV_64FC1)
		{
			_dpdf.reset(cvCloneMat(dpdf));
		}
		else
			_dpdf.reset(cvCreateMat(2 * count, 2, CV_64FC1));
		dpdf_p = _dpdf->data.db;
		dpdf_step = _dpdf->step / sizeof(dpdf_p[0]);
	}

	if (dpdc)
	{
		if (!CV_IS_MAT(dpdc) ||
			(CV_MAT_TYPE(dpdc->type) != CV_32FC1 && CV_MAT_TYPE(dpdc->type) != CV_64FC1) ||
			dpdc->rows != count * 2 || dpdc->cols != 2)
			CV_Error(CV_StsBadArg, "dp/dc must be 2Nx2 floating-point matrix");

		if (CV_MAT_TYPE(dpdc->type) == CV_64FC1)
		{
			_dpdc.reset(cvCloneMat(dpdc));
		}
		else
			_dpdc.reset(cvCreateMat(2 * count, 2, CV_64FC1));
		dpdc_p = _dpdc->data.db;
		dpdc_step = _dpdc->step / sizeof(dpdc_p[0]);
	}

	if (dpdk)
	{
		if (!CV_IS_MAT(dpdk) ||
			(CV_MAT_TYPE(dpdk->type) != CV_32FC1 && CV_MAT_TYPE(dpdk->type) != CV_64FC1) ||
			dpdk->rows != count * 2 || (dpdk->cols != 14 && dpdk->cols != 12 && dpdk->cols != 8 && dpdk->cols != 5 && dpdk->cols != 4 && dpdk->cols != 2))
			CV_Error(CV_StsBadArg, "dp/df must be 2Nx14, 2Nx12, 2Nx8, 2Nx5, 2Nx4 or 2Nx2 floating-point matrix");

		if (!distCoeffs)
			CV_Error(CV_StsNullPtr, "distCoeffs is NULL while dpdk is not");

		if (CV_MAT_TYPE(dpdk->type) == CV_64FC1)
		{
			_dpdk.reset(cvCloneMat(dpdk));
		}
		else
			_dpdk.reset(cvCreateMat(dpdk->rows, dpdk->cols, CV_64FC1));
		dpdk_p = _dpdk->data.db;
		dpdk_step = _dpdk->step / sizeof(dpdk_p[0]);
	}

	if (dpdo)
	{
		if (!CV_IS_MAT(dpdo) || (CV_MAT_TYPE(dpdo->type) != CV_32FC1
			&& CV_MAT_TYPE(dpdo->type) != CV_64FC1)
			|| dpdo->rows != count * 2 || dpdo->cols != count * 3)
			CV_Error(CV_StsBadArg, "dp/do must be 2Nx3N floating-point matrix");

		if (CV_MAT_TYPE(dpdo->type) == CV_64FC1)
		{
			_dpdo.reset(cvCloneMat(dpdo));
		}
		else
			_dpdo.reset(cvCreateMat(2 * count, 3 * count, CV_64FC1));
		cvZero(_dpdo);
		dpdo_p = _dpdo->data.db;
		dpdo_step = _dpdo->step / sizeof(dpdo_p[0]);
	}

	calc_derivatives = dpdr || dpdt || dpdf || dpdc || dpdk || dpdo;

	for (i = 0; i < count; i++)
	{
		double X = M[i].x, Y = M[i].y, Z = M[i].z;
		double x = R[0] * X + R[1] * Y + R[2] * Z + t[0];
		double y = R[3] * X + R[4] * Y + R[5] * Z + t[1];
		double z = R[6] * X + R[7] * Y + R[8] * Z + t[2];
		double r2, r4, r6, a1, a2, a3, cdist, icdist2;
		double xd, yd, xd0, yd0, invProj;
		Vec3d vecTilt;
		Vec3d dVecTilt;
		Matx22d dMatTilt;
		Vec2d dXdYd;

		double z0 = z;
		z = z ? 1. / z : 1;
		x *= z; y *= z;

		r2 = x * x + y * y;
		r4 = r2 * r2;
		r6 = r4 * r2;
		a1 = 2 * x * y;
		a2 = r2 + 2 * x * x;
		a3 = r2 + 2 * y * y;
		cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;
		icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
		xd0 = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
		yd0 = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;

		// additional distortion by projecting onto a tilt plane
		vecTilt = matTilt * Vec3d(xd0, yd0, 1);
		invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
		xd = invProj * vecTilt(0);
		yd = invProj * vecTilt(1);

		m[i].x = xd * fx + cx;
		m[i].y = yd * fy + cy;

		if (calc_derivatives)
		{
			if (dpdc_p)
			{
				dpdc_p[0] = 1; dpdc_p[1] = 0; // dp_xdc_x; dp_xdc_y
				dpdc_p[dpdc_step] = 0;
				dpdc_p[dpdc_step + 1] = 1;
				dpdc_p += dpdc_step * 2;
			}

			if (dpdf_p)
			{
				if (fixedAspectRatio)
				{
					dpdf_p[0] = 0; dpdf_p[1] = xd * aspectRatio; // dp_xdf_x; dp_xdf_y
					dpdf_p[dpdf_step] = 0;
					dpdf_p[dpdf_step + 1] = yd;
				}
				else
				{
					dpdf_p[0] = xd; dpdf_p[1] = 0;
					dpdf_p[dpdf_step] = 0;
					dpdf_p[dpdf_step + 1] = yd;
				}
				dpdf_p += dpdf_step * 2;
			}
			for (int row = 0; row < 2; ++row)
				for (int col = 0; col < 2; ++col)
					dMatTilt(row, col) = matTilt(row, col) * vecTilt(2)
					- matTilt(2, col) * vecTilt(row);
			double invProjSquare = (invProj * invProj);
			dMatTilt *= invProjSquare;
			if (dpdk_p)
			{
				dXdYd = dMatTilt * Vec2d(x * icdist2 * r2, y * icdist2 * r2);
				dpdk_p[0] = fx * dXdYd(0);
				dpdk_p[dpdk_step] = fy * dXdYd(1);
				dXdYd = dMatTilt * Vec2d(x * icdist2 * r4, y * icdist2 * r4);
				dpdk_p[1] = fx * dXdYd(0);
				dpdk_p[dpdk_step + 1] = fy * dXdYd(1);
				if (_dpdk->cols > 2)
				{
					dXdYd = dMatTilt * Vec2d(a1, a3);
					dpdk_p[2] = fx * dXdYd(0);
					dpdk_p[dpdk_step + 2] = fy * dXdYd(1);
					dXdYd = dMatTilt * Vec2d(a2, a1);
					dpdk_p[3] = fx * dXdYd(0);
					dpdk_p[dpdk_step + 3] = fy * dXdYd(1);
					if (_dpdk->cols > 4)
					{
						dXdYd = dMatTilt * Vec2d(x * icdist2 * r6, y * icdist2 * r6);
						dpdk_p[4] = fx * dXdYd(0);
						dpdk_p[dpdk_step + 4] = fy * dXdYd(1);

						if (_dpdk->cols > 5)
						{
							dXdYd = dMatTilt * Vec2d(
								x * cdist * (-icdist2) * icdist2 * r2, y * cdist * (-icdist2) * icdist2 * r2);
							dpdk_p[5] = fx * dXdYd(0);
							dpdk_p[dpdk_step + 5] = fy * dXdYd(1);
							dXdYd = dMatTilt * Vec2d(
								x * cdist * (-icdist2) * icdist2 * r4, y * cdist * (-icdist2) * icdist2 * r4);
							dpdk_p[6] = fx * dXdYd(0);
							dpdk_p[dpdk_step + 6] = fy * dXdYd(1);
							dXdYd = dMatTilt * Vec2d(
								x * cdist * (-icdist2) * icdist2 * r6, y * cdist * (-icdist2) * icdist2 * r6);
							dpdk_p[7] = fx * dXdYd(0);
							dpdk_p[dpdk_step + 7] = fy * dXdYd(1);
							if (_dpdk->cols > 8)
							{
								dXdYd = dMatTilt * Vec2d(r2, 0);
								dpdk_p[8] = fx * dXdYd(0); //s1
								dpdk_p[dpdk_step + 8] = fy * dXdYd(1); //s1
								dXdYd = dMatTilt * Vec2d(r4, 0);
								dpdk_p[9] = fx * dXdYd(0); //s2
								dpdk_p[dpdk_step + 9] = fy * dXdYd(1); //s2
								dXdYd = dMatTilt * Vec2d(0, r2);
								dpdk_p[10] = fx * dXdYd(0);//s3
								dpdk_p[dpdk_step + 10] = fy * dXdYd(1); //s3
								dXdYd = dMatTilt * Vec2d(0, r4);
								dpdk_p[11] = fx * dXdYd(0);//s4
								dpdk_p[dpdk_step + 11] = fy * dXdYd(1); //s4
								if (_dpdk->cols > 12)
								{
									dVecTilt = dMatTiltdTauX * Vec3d(xd0, yd0, 1);
									dpdk_p[12] = fx * invProjSquare * (
										dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
									dpdk_p[dpdk_step + 12] = fy * invProjSquare * (
										dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
									dVecTilt = dMatTiltdTauY * Vec3d(xd0, yd0, 1);
									dpdk_p[13] = fx * invProjSquare * (
										dVecTilt(0) * vecTilt(2) - dVecTilt(2) * vecTilt(0));
									dpdk_p[dpdk_step + 13] = fy * invProjSquare * (
										dVecTilt(1) * vecTilt(2) - dVecTilt(2) * vecTilt(1));
								}
							}
						}
					}
				}
				dpdk_p += dpdk_step * 2;
			}

			if (dpdt_p)
			{
				double dxdt[] = { z, 0, -x * z }, dydt[] = { 0, z, -y * z };
				for (j = 0; j < 3; j++)
				{
					double dr2dt = 2 * x * dxdt[j] + 2 * y * dydt[j];
					double dcdist_dt = k[0] * dr2dt + 2 * k[1] * r2 * dr2dt + 3 * k[4] * r4 * dr2dt;
					double dicdist2_dt = -icdist2 * icdist2 * (k[5] * dr2dt + 2 * k[6] * r2 * dr2dt + 3 * k[7] * r4 * dr2dt);
					double da1dt = 2 * (x * dydt[j] + y * dxdt[j]);
					double dmxdt = (dxdt[j] * cdist * icdist2 + x * dcdist_dt * icdist2 + x * cdist * dicdist2_dt +
						k[2] * da1dt + k[3] * (dr2dt + 4 * x * dxdt[j]) + k[8] * dr2dt + 2 * r2 * k[9] * dr2dt);
					double dmydt = (dydt[j] * cdist * icdist2 + y * dcdist_dt * icdist2 + y * cdist * dicdist2_dt +
						k[2] * (dr2dt + 4 * y * dydt[j]) + k[3] * da1dt + k[10] * dr2dt + 2 * r2 * k[11] * dr2dt);
					dXdYd = dMatTilt * Vec2d(dmxdt, dmydt);
					dpdt_p[j] = fx * dXdYd(0);
					dpdt_p[dpdt_step + j] = fy * dXdYd(1);
				}
				dpdt_p += dpdt_step * 2;
			}

			if (dpdr_p)
			{
				double dx0dr[] =
				{
					X * dRdr[0] + Y * dRdr[1] + Z * dRdr[2],
					X * dRdr[9] + Y * dRdr[10] + Z * dRdr[11],
					X * dRdr[18] + Y * dRdr[19] + Z * dRdr[20]
				};
				double dy0dr[] =
				{
					X * dRdr[3] + Y * dRdr[4] + Z * dRdr[5],
					X * dRdr[12] + Y * dRdr[13] + Z * dRdr[14],
					X * dRdr[21] + Y * dRdr[22] + Z * dRdr[23]
				};
				double dz0dr[] =
				{
					X * dRdr[6] + Y * dRdr[7] + Z * dRdr[8],
					X * dRdr[15] + Y * dRdr[16] + Z * dRdr[17],
					X * dRdr[24] + Y * dRdr[25] + Z * dRdr[26]
				};
				for (j = 0; j < 3; j++)
				{
					double dxdr = z * (dx0dr[j] - x * dz0dr[j]);
					double dydr = z * (dy0dr[j] - y * dz0dr[j]);
					double dr2dr = 2 * x * dxdr + 2 * y * dydr;
					double dcdist_dr = (k[0] + 2 * k[1] * r2 + 3 * k[4] * r4) * dr2dr;
					double dicdist2_dr = -icdist2 * icdist2 * (k[5] + 2 * k[6] * r2 + 3 * k[7] * r4) * dr2dr;
					double da1dr = 2 * (x * dydr + y * dxdr);
					double dmxdr = (dxdr * cdist * icdist2 + x * dcdist_dr * icdist2 + x * cdist * dicdist2_dr +
						k[2] * da1dr + k[3] * (dr2dr + 4 * x * dxdr) + (k[8] + 2 * r2 * k[9]) * dr2dr);
					double dmydr = (dydr * cdist * icdist2 + y * dcdist_dr * icdist2 + y * cdist * dicdist2_dr +
						k[2] * (dr2dr + 4 * y * dydr) + k[3] * da1dr + (k[10] + 2 * r2 * k[11]) * dr2dr);
					dXdYd = dMatTilt * Vec2d(dmxdr, dmydr);
					dpdr_p[j] = fx * dXdYd(0);
					dpdr_p[dpdr_step + j] = fy * dXdYd(1);
				}
				dpdr_p += dpdr_step * 2;
			}

			if (dpdo_p)
			{
				double dxdo[] = { z * (R[0] - x * z * z0 * R[6]),
								  z * (R[1] - x * z * z0 * R[7]),
								  z * (R[2] - x * z * z0 * R[8]) };
				double dydo[] = { z * (R[3] - y * z * z0 * R[6]),
								  z * (R[4] - y * z * z0 * R[7]),
								  z * (R[5] - y * z * z0 * R[8]) };
				for (j = 0; j < 3; j++)
				{
					double dr2do = 2 * x * dxdo[j] + 2 * y * dydo[j];
					double dr4do = 2 * r2 * dr2do;
					double dr6do = 3 * r4 * dr2do;
					double da1do = 2 * y * dxdo[j] + 2 * x * dydo[j];
					double da2do = dr2do + 4 * x * dxdo[j];
					double da3do = dr2do + 4 * y * dydo[j];
					double dcdist_do
						= k[0] * dr2do + k[1] * dr4do + k[4] * dr6do;
					double dicdist2_do = -icdist2 * icdist2
						* (k[5] * dr2do + k[6] * dr4do + k[7] * dr6do);
					double dxd0_do = cdist * icdist2 * dxdo[j]
						+ x * icdist2 * dcdist_do + x * cdist * dicdist2_do
						+ k[2] * da1do + k[3] * da2do + k[8] * dr2do
						+ k[9] * dr4do;
					double dyd0_do = cdist * icdist2 * dydo[j]
						+ y * icdist2 * dcdist_do + y * cdist * dicdist2_do
						+ k[2] * da3do + k[3] * da1do + k[10] * dr2do
						+ k[11] * dr4do;
					dXdYd = dMatTilt * Vec2d(dxd0_do, dyd0_do);
					dpdo_p[i * 3 + j] = fx * dXdYd(0);
					dpdo_p[dpdo_step + i * 3 + j] = fy * dXdYd(1);
				}
				dpdo_p += dpdo_step * 2;
			}
		}
	}

	if (_m != imagePoints)
		cvConvert(_m, imagePoints);

	if (_dpdr != dpdr)
		cvConvert(_dpdr, dpdr);

	if (_dpdt != dpdt)
		cvConvert(_dpdt, dpdt);

	if (_dpdf != dpdf)
		cvConvert(_dpdf, dpdf);

	if (_dpdc != dpdc)
		cvConvert(_dpdc, dpdc);

	if (_dpdk != dpdk)
		cvConvert(_dpdk, dpdk);

	if (_dpdo != dpdo)
		cvConvert(_dpdo, dpdo);
}


void cvProjectPoints2(const CvMat* objectPoints,
	const CvMat* r_vec,
	const CvMat* t_vec,
	const CvMat* A,
	const CvMat* distCoeffs,
	CvMat* imagePoints, CvMat* dpdr,
	CvMat* dpdt, CvMat* dpdf,
	CvMat* dpdc, CvMat* dpdk,
	double aspectRatio)
{
	cvProjectPoints2Internal(objectPoints, r_vec, t_vec, A, distCoeffs, imagePoints, dpdr, dpdt,
		dpdf, dpdc, dpdk, NULL, aspectRatio);
}

void cvConvertPointsHomogeneous(const CvMat* _src, CvMat* _dst)
{
	cv::Mat src = cv::cvarrToMat(_src), dst = cv::cvarrToMat(_dst);
	const cv::Mat dst0 = dst;

	int d0 = src.channels() > 1 ? src.channels() : MIN(src.cols, src.rows);

	if (src.channels() == 1 && src.cols > d0)
		cv::transpose(src, src);

	int d1 = dst.channels() > 1 ? dst.channels() : MIN(dst.cols, dst.rows);

	if (d0 == d1)
		src.copyTo(dst);
	else if (d0 < d1)
		cv::convertPointsToHomogeneous(src, dst);
	else
		cv::convertPointsFromHomogeneous(src, dst);

	bool tflag = dst0.channels() == 1 && dst0.cols > d1;
	dst = dst.reshape(dst0.channels(), (tflag ? dst0.cols : dst0.rows));

	if (tflag)
	{
		CV_Assert(dst.rows == dst0.cols && dst.cols == dst0.rows);
		if (dst0.type() == dst.type())
			transpose(dst, dst0);
		else
		{
			transpose(dst, dst);
			dst.convertTo(dst0, dst0.type());
		}
	}
	else
	{
		CV_Assert(dst.size() == dst0.size());
		if (dst.data != dst0.data)
			dst.convertTo(dst0, dst0.type());
	}
}


static void cvUndistortPointsInternal(const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
	const CvMat* _distCoeffs,
	const CvMat* matR, const CvMat* matP, cv::TermCriteria criteria)
{
	double A[3][3], RR[3][3], k[14] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	CvMat matA = cvMat(3, 3, CV_64F, A), _Dk;
	CvMat _RR = cvMat(3, 3, CV_64F, RR);
	cv::Matx33d invMatTilt = cv::Matx33d::eye();
	cv::Matx33d matTilt = cv::Matx33d::eye();


	cvConvert(_cameraMatrix, &matA);


	if (_distCoeffs)
	{
		_Dk = cvMat(_distCoeffs->rows, _distCoeffs->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(_distCoeffs->type)), k);

		cvConvert(_distCoeffs, &_Dk);
		if (k[12] != 0 || k[13] != 0)
		{

			computeTiltProjectionMatrix<double>(k[12], k[13], NULL, NULL, NULL, &invMatTilt);
			computeTiltProjectionMatrix<double>(k[12], k[13], &matTilt, NULL, NULL);
		}
	}

	cvSetIdentity(&_RR);

	if (matP)
	{
		double PP[3][3];
		CvMat _P3x3, _PP = cvMat(3, 3, CV_64F, PP);
		cvConvert(cvGetCols(matP, &_P3x3, 0, 3), &_PP);
		cvMatMul(&_PP, &_RR, &_RR);
	}

	const CvPoint2D32f* srcf = (const CvPoint2D32f*)_src->data.ptr;
	const CvPoint2D64f* srcd = (const CvPoint2D64f*)_src->data.ptr;
	CvPoint2D32f* dstf = (CvPoint2D32f*)_dst->data.ptr;
	CvPoint2D64f* dstd = (CvPoint2D64f*)_dst->data.ptr;
	int stype = CV_MAT_TYPE(_src->type);
	int dtype = CV_MAT_TYPE(_dst->type);
	int sstep = _src->rows == 1 ? 1 : _src->step / CV_ELEM_SIZE(stype);
	int dstep = _dst->rows == 1 ? 1 : _dst->step / CV_ELEM_SIZE(dtype);

	double fx = A[0][0];
	double fy = A[1][1];
	double ifx = 1. / fx;
	double ify = 1. / fy;
	double cx = A[0][2];
	double cy = A[1][2];

	int n = _src->rows + _src->cols - 1;
	for (int i = 0; i < n; i++)
	{
		double x, y, x0 = 0, y0 = 0, u, v;
		if (stype == CV_32FC2)
		{
			x = srcf[i * sstep].x;
			y = srcf[i * sstep].y;
		}
		else
		{
			x = srcd[i * sstep].x;
			y = srcd[i * sstep].y;
		}
		u = x; v = y;
		x = (x - cx) * ifx;
		y = (y - cy) * ify;

		if (_distCoeffs) {
			// compensate tilt distortion
			cv::Vec3d vecUntilt = invMatTilt * cv::Vec3d(x, y, 1);
			double invProj = vecUntilt(2) ? 1. / vecUntilt(2) : 1;
			x0 = x = invProj * vecUntilt(0);
			y0 = y = invProj * vecUntilt(1);

			double error = DBL_MAX;
			// compensate distortion iteratively

			for (int j = 0; ; j++)
			{
				if ((criteria.type & cv::TermCriteria::COUNT) && j >= criteria.maxCount)
					break;
				if ((criteria.type & cv::TermCriteria::EPS) && error < criteria.epsilon)
					break;
				double r2 = x * x + y * y;
				double icdist = (1 + ((k[7] * r2 + k[6]) * r2 + k[5]) * r2) / (1 + ((k[4] * r2 + k[1]) * r2 + k[0]) * r2);
				if (icdist < 0)  // test: undistortPoints.regression_14583
				{
					x = (u - cx) * ifx;
					y = (v - cy) * ify;
					break;
				}
				double deltaX = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x) + k[8] * r2 + k[9] * r2 * r2;
				double deltaY = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y + k[10] * r2 + k[11] * r2 * r2;
				x = (x0 - deltaX) * icdist;
				y = (y0 - deltaY) * icdist;

				if (criteria.type & cv::TermCriteria::EPS)
				{
					double r4, r6, a1, a2, a3, cdist, icdist2;
					double xd, yd, xd0, yd0;
					cv::Vec3d vecTilt;

					r2 = x * x + y * y;
					r4 = r2 * r2;
					r6 = r4 * r2;
					a1 = 2 * x * y;
					a2 = r2 + 2 * x * x;
					a3 = r2 + 2 * y * y;
					cdist = 1 + k[0] * r2 + k[1] * r4 + k[4] * r6;
					icdist2 = 1. / (1 + k[5] * r2 + k[6] * r4 + k[7] * r6);
					xd0 = x * cdist * icdist2 + k[2] * a1 + k[3] * a2 + k[8] * r2 + k[9] * r4;
					yd0 = y * cdist * icdist2 + k[2] * a3 + k[3] * a1 + k[10] * r2 + k[11] * r4;

					vecTilt = matTilt * cv::Vec3d(xd0, yd0, 1);
					invProj = vecTilt(2) ? 1. / vecTilt(2) : 1;
					xd = invProj * vecTilt(0);
					yd = invProj * vecTilt(1);

					double x_proj = xd * fx + cx;
					double y_proj = yd * fy + cy;

					error = sqrt(pow(x_proj - u, 2) + pow(y_proj - v, 2));
				}
			}
		}

		double xx = RR[0][0] * x + RR[0][1] * y + RR[0][2];
		double yy = RR[1][0] * x + RR[1][1] * y + RR[1][2];
		double ww = 1. / (RR[2][0] * x + RR[2][1] * y + RR[2][2]);
		x = xx * ww;
		y = yy * ww;

		if (dtype == CV_32FC2)
		{
			dstf[i * dstep].x = (float)x;
			dstf[i * dstep].y = (float)y;
		}
		else
		{
			dstd[i * dstep].x = x;
			dstd[i * dstep].y = y;
		}
	}
}


void __undistortPoints(InputArray _src, OutputArray _dst,
	InputArray _cameraMatrix,
	InputArray _distCoeffs,
	InputArray _Rmat,
	InputArray _Pmat)
{
	TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER, 5, 0.01);

	Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
	Mat distCoeffs = _distCoeffs.getMat(), R = _Rmat.getMat(), P = _Pmat.getMat();

	int npoints = src.checkVector(2), depth = src.depth();
	if (npoints < 0)
		src = src.t();
	npoints = src.checkVector(2);
	CV_Assert(npoints >= 0 && src.isContinuous() && (depth == CV_32F || depth == CV_64F));

	if (src.cols == 2)
		src = src.reshape(2);

	_dst.create(npoints, 1, CV_MAKETYPE(depth, 2), -1, true);
	Mat dst = _dst.getMat();

	CvMat _csrc = cvMat(src), _cdst = cvMat(dst), _ccameraMatrix = cvMat(cameraMatrix);
	CvMat matR, matP, _cdistCoeffs, * pR = 0, * pP = 0, * pD = 0;
	if (!R.empty())
		pR = &(matR = cvMat(R));
	if (!P.empty())
		pP = &(matP = cvMat(P));
	if (!distCoeffs.empty())
		pD = &(_cdistCoeffs = cvMat(distCoeffs));
	cvUndistortPointsInternal(&_csrc, &_cdst, &_ccameraMatrix, pD, pR, pP, criteria);
}







void cvUndistortPoints(const CvMat* _src, CvMat* _dst, const CvMat* _cameraMatrix,
	const CvMat* _distCoeffs,
	const CvMat* matR, const CvMat* matP)
{
	cvUndistortPointsInternal(_src, _dst, _cameraMatrix, _distCoeffs, matR, matP,
		cv::TermCriteria(cv::TermCriteria::COUNT, 5, 0.01));
}


int cvFindHomography(const CvMat* _src, const CvMat* _dst, CvMat* __H, int method,
	double ransacReprojThreshold, CvMat* _mask, int maxIters,
	double confidence)
{
	cv::Mat src = cv::cvarrToMat(_src), dst = cv::cvarrToMat(_dst);

	if (src.channels() == 1 && (src.rows == 2 || src.rows == 3) && src.cols > 3)
		cv::transpose(src, src);
	if (dst.channels() == 1 && (dst.rows == 2 || dst.rows == 3) && dst.cols > 3)
		cv::transpose(dst, dst);

	if (maxIters < 0)
		maxIters = 0;
	if (maxIters > 2000)
		maxIters = 2000;

	if (confidence < 0)
		confidence = 0;
	if (confidence > 1)
		confidence = 1;

	const cv::Mat H = cv::cvarrToMat(__H), mask = cv::cvarrToMat(_mask);
	cv::Mat H0 = cv::findHomography(src, dst, method, ransacReprojThreshold,
		_mask ? cv::_OutputArray(mask) : cv::_OutputArray(), maxIters,
		confidence);

	if (H0.empty())
	{
		cv::Mat Hz = cv::cvarrToMat(__H);
		Hz.setTo(cv::Scalar::all(0));
		return 0;
	}
	H0.convertTo(H, H.type());
	return 1;
}





void cvFindExtrinsicCameraParams2(const CvMat* objectPoints,
	const CvMat* imagePoints, const CvMat* A,
	const CvMat* distCoeffs, CvMat* rvec, CvMat* tvec,
	int useExtrinsicGuess)
{
	const int max_iter = 20;
	Ptr<CvMat> matM, _Mxy, _m, _mn, matL;

	int i, count;
	double a[9], ar[9] = { 1,0,0,0,1,0,0,0,1 }, R[9];
	double MM[9] = { 0 }, U[9] = { 0 }, V[9] = { 0 }, W[3] = { 0 };
	cv::Scalar Mc;
	double param[6] = { 0 };
	CvMat matA = cvMat(3, 3, CV_64F, a);
	CvMat _Ar = cvMat(3, 3, CV_64F, ar);
	CvMat matR = cvMat(3, 3, CV_64F, R);
	CvMat _r = cvMat(3, 1, CV_64F, param);
	CvMat _t = cvMat(3, 1, CV_64F, param + 3);
	CvMat _Mc = cvMat(1, 3, CV_64F, Mc.val);
	CvMat _MM = cvMat(3, 3, CV_64F, MM);
	CvMat matU = cvMat(3, 3, CV_64F, U);
	CvMat matV = cvMat(3, 3, CV_64F, V);
	CvMat matW = cvMat(3, 1, CV_64F, W);
	CvMat _param = cvMat(6, 1, CV_64F, param);
	CvMat _dpdr, _dpdt;

	CV_Assert(CV_IS_MAT(objectPoints) && CV_IS_MAT(imagePoints) &&
		CV_IS_MAT(A) && CV_IS_MAT(rvec) && CV_IS_MAT(tvec));

	count = MAX(objectPoints->cols, objectPoints->rows);
	matM.reset(cvCreateMat(1, count, CV_64FC3));
	_m.reset(cvCreateMat(1, count, CV_64FC2));

	cvConvertPointsHomogeneous(objectPoints, matM);
	cvConvertPointsHomogeneous(imagePoints, _m);
	cvConvert(A, &matA);

	CV_Assert((CV_MAT_DEPTH(rvec->type) == CV_64F || CV_MAT_DEPTH(rvec->type) == CV_32F) &&
		(rvec->rows == 1 || rvec->cols == 1) && rvec->rows * rvec->cols * CV_MAT_CN(rvec->type) == 3);

	CV_Assert((CV_MAT_DEPTH(tvec->type) == CV_64F || CV_MAT_DEPTH(tvec->type) == CV_32F) &&
		(tvec->rows == 1 || tvec->cols == 1) && tvec->rows * tvec->cols * CV_MAT_CN(tvec->type) == 3);

	CV_Assert((count >= 4) || (count == 3 && useExtrinsicGuess)); // it is unsafe to call LM optimisation without an extrinsic guess in the case of 3 points. This is because there is no guarantee that it will converge on the correct solution.

	_mn.reset(cvCreateMat(1, count, CV_64FC2));
	_Mxy.reset(cvCreateMat(1, count, CV_64FC2));

	// normalize image points
	// (unapply the intrinsic matrix transformation and distortion)
	cvUndistortPoints(_m, _mn, &matA, distCoeffs, 0, &_Ar);

	if (useExtrinsicGuess)
	{
		CvMat _r_temp = cvMat(rvec->rows, rvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
		CvMat _t_temp = cvMat(tvec->rows, tvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);
		cvConvert(rvec, &_r_temp);
		cvConvert(tvec, &_t_temp);
	}
	else
	{
		Mc = cvAvg(matM);
		cvReshape(matM, matM, 1, count);
		cvMulTransposed(matM, &_MM, 1, &_Mc);
		cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);

		// initialize extrinsic parameters
		if (W[2] / W[1] < 1e-3)
		{
			// a planar structure case (all M's lie in the same plane)
			double tt[3], h[9], h1_norm, h2_norm;
			CvMat* R_transform = &matV;
			CvMat T_transform = cvMat(3, 1, CV_64F, tt);
			CvMat matH = cvMat(3, 3, CV_64F, h);
			CvMat _h1, _h2, _h3;

			if (V[2] * V[2] + V[5] * V[5] < 1e-10)
				cvSetIdentity(R_transform);

			if (cvDet(R_transform) < 0)
				cvScale(R_transform, R_transform, -1);

			cvGEMM(R_transform, &_Mc, -1, 0, 0, &T_transform, CV_GEMM_B_T);

			for (i = 0; i < count; i++)
			{
				const double* Rp = R_transform->data.db;
				const double* Tp = T_transform.data.db;
				const double* src = matM->data.db + i * 3;
				double* dst = _Mxy->data.db + i * 2;

				dst[0] = Rp[0] * src[0] + Rp[1] * src[1] + Rp[2] * src[2] + Tp[0];
				dst[1] = Rp[3] * src[0] + Rp[4] * src[1] + Rp[5] * src[2] + Tp[1];
			}

			cvFindHomography(_Mxy, _mn, &matH, 0, 3, 0, 2000, 0.995);

			if (cvCheckArr(&matH, CV_CHECK_QUIET))
			{
				cvGetCol(&matH, &_h1, 0);
				_h2 = _h1; _h2.data.db++;
				_h3 = _h2; _h3.data.db++;
				h1_norm = std::sqrt(h[0] * h[0] + h[3] * h[3] + h[6] * h[6]);
				h2_norm = std::sqrt(h[1] * h[1] + h[4] * h[4] + h[7] * h[7]);

				cvScale(&_h1, &_h1, 1. / MAX(h1_norm, DBL_EPSILON));
				cvScale(&_h2, &_h2, 1. / MAX(h2_norm, DBL_EPSILON));
				cvScale(&_h3, &_t, 2. / MAX(h1_norm + h2_norm, DBL_EPSILON));
				cvCrossProduct(&_h1, &_h2, &_h3);

				cvRodrigues2(&matH, &_r, 0);
				cvRodrigues2(&_r, &matH, 0);
				cvMatMulAdd(&matH, &T_transform, &_t, &_t);
				cvMatMul(&matH, R_transform, &matR);
			}
			else
			{
				cvSetIdentity(&matR);
				cvZero(&_t);
			}

			cvRodrigues2(&matR, &_r, 0);
		}
		else
		{
			// non-planar structure. Use DLT method
			CV_CheckGE(count, 6, "DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences.");
			double* L;
			double LL[12 * 12], LW[12], LV[12 * 12], sc;
			CvMat _LL = cvMat(12, 12, CV_64F, LL);
			CvMat _LW = cvMat(12, 1, CV_64F, LW);
			CvMat _LV = cvMat(12, 12, CV_64F, LV);
			CvMat _RRt, _RR, _tt;
			CvPoint3D64f* M = (CvPoint3D64f*)matM->data.db;
			CvPoint2D64f* mn = (CvPoint2D64f*)_mn->data.db;

			matL.reset(cvCreateMat(2 * count, 12, CV_64F));
			L = matL->data.db;

			for (i = 0; i < count; i++, L += 24)
			{
				double x = -mn[i].x, y = -mn[i].y;
				L[0] = L[16] = M[i].x;
				L[1] = L[17] = M[i].y;
				L[2] = L[18] = M[i].z;
				L[3] = L[19] = 1.;
				L[4] = L[5] = L[6] = L[7] = 0.;
				L[12] = L[13] = L[14] = L[15] = 0.;
				L[8] = x * M[i].x;
				L[9] = x * M[i].y;
				L[10] = x * M[i].z;
				L[11] = x;
				L[20] = y * M[i].x;
				L[21] = y * M[i].y;
				L[22] = y * M[i].z;
				L[23] = y;
			}

			cvMulTransposed(matL, &_LL, 1);
			cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
			_RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
			cvGetCols(&_RRt, &_RR, 0, 3);
			cvGetCol(&_RRt, &_tt, 3);
			if (cvDet(&_RR) < 0)
				cvScale(&_RRt, &_RRt, -1);
			sc = cvNorm(&_RR);
			CV_Assert(fabs(sc) > DBL_EPSILON);
			cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
			cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
			cvScale(&_tt, &_t, cvNorm(&matR) / sc);
			cvRodrigues2(&matR, &_r, 0);
		}
	}

	cvReshape(matM, matM, 3, 1);
	cvReshape(_mn, _mn, 2, 1);

	// refine extrinsic parameters using iterative algorithm
	CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, FLT_EPSILON), true);
	cvCopy(&_param, solver.param);

	for (;;)
	{
		CvMat* matJ = 0, * _err = 0;
		const CvMat* __param = 0;
		bool proceed = solver.update(__param, matJ, _err);
		cvCopy(__param, &_param);
		if (!proceed || !_err)
			break;
		cvReshape(_err, _err, 2, 1);
		if (matJ)
		{
			cvGetCols(matJ, &_dpdr, 0, 3);
			cvGetCols(matJ, &_dpdt, 3, 6);
			cvProjectPoints2(matM, &_r, &_t, &matA, distCoeffs,
				_err, &_dpdr, &_dpdt, 0, 0, 0, 0);
		}
		else
		{
			cvProjectPoints2(matM, &_r, &_t, &matA, distCoeffs,
				_err, 0, 0, 0, 0, 0, 0);
		}
		cvSub(_err, _m, _err);
		cvReshape(_err, _err, 1, 2 * count);
	}
	cvCopy(solver.param, &_param);

	_r = cvMat(rvec->rows, rvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
	_t = cvMat(tvec->rows, tvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);

	cvConvert(&_r, rvec);
	cvConvert(&_t, tvec);
}



int __solvePnPGeneric__(InputArray _opoints, InputArray _ipoints,
	InputArray _cameraMatrix, InputArray _distCoeffs,
	OutputArrayOfArrays _rvecs, OutputArrayOfArrays _tvecs, SolvePnPMethod flags,
	InputArray _rvec, InputArray _tvec,
	OutputArray reprojectionError)
{

	Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
	int npoints = max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));

	opoints = opoints.reshape(3, npoints);
	ipoints = ipoints.reshape(2, npoints);



	Mat cameraMatrix0 = _cameraMatrix.getMat();
	Mat distCoeffs0 = _distCoeffs.getMat();
	Mat cameraMatrix = Mat_<double>(cameraMatrix0);
	Mat distCoeffs = Mat_<double>(distCoeffs0);

	vector<Mat> vec_rvecs, vec_tvecs;
	if (flags == SOLVEPNP_EPNP)
	{
		Mat undistortedPoints;
		__undistortPoints(ipoints, undistortedPoints, cameraMatrix, distCoeffs, noArray(), noArray());
		epnp PnP(cameraMatrix, opoints, undistortedPoints);

		Mat rvec, tvec, R;
		PnP.compute_pose(R, tvec);
		Rodrigues(R, rvec);

		vec_rvecs.push_back(rvec);
		vec_tvecs.push_back(tvec);
	}
	else if (flags == SOLVEPNP_ITERATIVE)
	{
		Mat rvec, tvec;
		rvec.create(3, 1, CV_64FC1);
		tvec.create(3, 1, CV_64FC1);

		CvMat c_objectPoints = cvMat(opoints), c_imagePoints = cvMat(ipoints);
		CvMat c_cameraMatrix = cvMat(cameraMatrix), c_distCoeffs = cvMat(distCoeffs);
		CvMat c_rvec = cvMat(rvec), c_tvec = cvMat(tvec);
		cvFindExtrinsicCameraParams2(&c_objectPoints, &c_imagePoints, &c_cameraMatrix,
			(c_distCoeffs.rows && c_distCoeffs.cols) ? &c_distCoeffs : 0,
			&c_rvec, &c_tvec, false);

		vec_rvecs.push_back(rvec);
		vec_tvecs.push_back(tvec);
	}

	CV_Assert(vec_rvecs.size() == vec_tvecs.size());

	int solutions = static_cast<int>(vec_rvecs.size());

	int depthRot = _rvecs.fixedType() ? _rvecs.depth() : CV_64F;
	int depthTrans = _tvecs.fixedType() ? _tvecs.depth() : CV_64F;
	_rvecs.create(solutions, 1, CV_MAKETYPE(depthRot, _rvecs.fixedType() && _rvecs.kind() == _InputArray::STD_VECTOR ? 3 : 1));
	_tvecs.create(solutions, 1, CV_MAKETYPE(depthTrans, _tvecs.fixedType() && _tvecs.kind() == _InputArray::STD_VECTOR ? 3 : 1));

	for (int i = 0; i < solutions; i++)
	{
		Mat rvec0, tvec0;
		if (depthRot == CV_64F)
			rvec0 = vec_rvecs[i];
		else
			vec_rvecs[i].convertTo(rvec0, depthRot);

		if (depthTrans == CV_64F)
			tvec0 = vec_tvecs[i];
		else
			vec_tvecs[i].convertTo(tvec0, depthTrans);

		if (_rvecs.fixedType() && _rvecs.kind() == _InputArray::STD_VECTOR)
		{
			Mat rref = _rvecs.getMat_();

			if (_rvecs.depth() == CV_32F)
				rref.at<Vec3f>(0, i) = Vec3f(rvec0.at<float>(0, 0), rvec0.at<float>(1, 0), rvec0.at<float>(2, 0));
			else
				rref.at<Vec3d>(0, i) = Vec3d(rvec0.at<double>(0, 0), rvec0.at<double>(1, 0), rvec0.at<double>(2, 0));
		}
		else
		{
			_rvecs.getMatRef(i) = rvec0;
		}

		if (_tvecs.fixedType() && _tvecs.kind() == _InputArray::STD_VECTOR)
		{

			Mat tref = _tvecs.getMat_();

			if (_tvecs.depth() == CV_32F)
				tref.at<Vec3f>(0, i) = Vec3f(tvec0.at<float>(0, 0), tvec0.at<float>(1, 0), tvec0.at<float>(2, 0));
			else
				tref.at<Vec3d>(0, i) = Vec3d(tvec0.at<double>(0, 0), tvec0.at<double>(1, 0), tvec0.at<double>(2, 0));
		}
		else
		{
			_tvecs.getMatRef(i) = tvec0;
		}
	}

	if (reprojectionError.needed())
	{
		int type = (reprojectionError.fixedType() || !reprojectionError.empty())
			? reprojectionError.type()
			: (max(_ipoints.depth(), _opoints.depth()) == CV_64F ? CV_64F : CV_32F);

		reprojectionError.create(solutions, 1, type);
		CV_CheckType(reprojectionError.type(), type == CV_32FC1 || type == CV_64FC1,
			"Type of reprojectionError must be CV_32FC1 or CV_64FC1!");

		Mat objectPoints, imagePoints;
		if (opoints.depth() == CV_32F)
		{
			opoints.convertTo(objectPoints, CV_64F);
		}
		else
		{
			objectPoints = opoints;
		}
		if (ipoints.depth() == CV_32F)
		{
			ipoints.convertTo(imagePoints, CV_64F);
		}
		else
		{
			imagePoints = ipoints;
		}

		for (size_t i = 0; i < vec_rvecs.size(); i++)
		{
			vector<Point2d> projectedPoints;
			projectPoints(objectPoints, vec_rvecs[i], vec_tvecs[i], cameraMatrix, distCoeffs, projectedPoints);
			double rmse = norm(Mat(projectedPoints, false), imagePoints, NORM_L2) / sqrt(2 * projectedPoints.size());

			Mat err = reprojectionError.getMat();
			if (type == CV_32F)
			{
				err.at<float>(static_cast<int>(i)) = static_cast<float>(rmse);
			}
			else
			{
				err.at<double>(static_cast<int>(i)) = rmse;
			}
		}
	}

	return solutions;
}




bool __solvePnP__(InputArray opoints, InputArray ipoints,
	InputArray cameraMatrix, InputArray distCoeffs,
	OutputArray rvec, OutputArray tvec, int flags)
{

	vector<Mat> rvecs, tvecs;
	int solutions = __solvePnPGeneric__(opoints, ipoints, cameraMatrix, distCoeffs, rvecs, tvecs, (SolvePnPMethod)flags, rvec, tvec, noArray());

	if (solutions > 0)
	{
		int rdepth = rvec.empty() ? CV_64F : rvec.depth();
		int tdepth = tvec.empty() ? CV_64F : tvec.depth();
		rvecs[0].convertTo(rvec, rdepth);
		tvecs[0].convertTo(tvec, tdepth);
	}

	return solutions > 0;
}


bool Methods::__solvePnPRansac__(Mat opoints, Mat ipoints,
	Mat cameraMatrix, Mat distCoeffs,
	Mat& rvec, Mat& tvec, bool useExtrinsicGuess,
	int iterationsCount, float reprojectionError, double confidence,
	int flags)
{
	int npoints = opoints.rows;

	rvec = Mat(3, 1, CV_64FC1);
	tvec = Mat(3, 1, CV_64FC1);

	int modelPoints = 5;
	int ransac_kernel_method = SOLVEPNP_EPNP;


	double param1 = reprojectionError;                // reprojection error
	double param2 = confidence;                       // confidence
	int param3 = iterationsCount;                     // number maximum iterations

	Mat _local_model(3, 2, CV_64FC1);
	Mat _mask_local_inliers(1, opoints.rows, CV_8UC1);

	// call Ransac
	//int result = __createRANSACPointSetRegistrator__(cb, modelPoints,
		//param1, param2, param3)->__run__(opoints, ipoints, _local_model, _mask_local_inliers);


	bool result = false;
	{
		OutputArray _model = _local_model;
		OutputArray _mask = _mask_local_inliers;
		Mat err, mask, model, bestModel, ms1, ms2;

		int iter, niters = MAX(iterationsCount, 1);
		int d1 = opoints.cols;
		int count = opoints.rows, maxGoodCount = 0;

		RNG rng((uint64)-1);

		if (count < modelPoints)
			return false;

		Mat bestMask0, bestMask;

		_mask.create(count, 1, CV_8U, -1, true);
		bestMask0 = bestMask = _mask.getMat();

		for (iter = 0; iter < niters; iter++)
		{
			int i, nmodels;

			//bool found = __getSubset__(m1, m2, ms1, ms2, rng, 10000);
			{
				cv::AutoBuffer<int> _idx(modelPoints);
				int* idx = _idx.data();

				const int d2 = ipoints.channels();

				int esz1 = (int)opoints.elemSize1() * d1;
				int esz2 = (int)ipoints.elemSize1() * d2;
				esz1 /= sizeof(int);
				esz2 /= sizeof(int);

				const int* m1ptr = opoints.ptr<int>();
				const int* m2ptr = ipoints.ptr<int>();

				ms1.create(modelPoints, 1, CV_MAKETYPE(opoints.depth(), d1));
				ms2.create(modelPoints, 1, CV_MAKETYPE(ipoints.depth(), d2));


				int* ms1ptr = ms1.ptr<int>();
				int* ms2ptr = ms2.ptr<int>();

				for (int iters = 0; iters < 1000; ++iters)
				{
					int i;

					for (i = 0; i < modelPoints; ++i)
					{
						int idx_i;

						for (idx_i = rng.uniform(0, count);
							std::find(idx, idx + i, idx_i) != idx + i;
							idx_i = rng.uniform(0, count))
						{
						}

						idx[i] = idx_i;

						for (int k = 0; k < esz1; ++k)
							ms1ptr[i * esz1 + k] = m1ptr[idx_i * esz1 + k];

						for (int k = 0; k < esz2; ++k)
							ms2ptr[i * esz2 + k] = m2ptr[idx_i * esz2 + k];
					}
					break;
				}
			}
			//nmodels = cb->__runKernel__(ms1, ms2, model);
			{
				cout << "ransacing~~~ " << endl;
				nmodels = __solvePnP__(ms1, ms2, cameraMatrix, distCoeffs,
					rvec, tvec, ransac_kernel_method);

				Mat _local_model;
				hconcat(rvec, tvec, _local_model);
				_local_model.copyTo(model);
			}
			Size modelSize(model.cols, model.rows / nmodels);


			for (i = 0; i < nmodels; i++)
			{
				Mat model_i = model.rowRange(i * modelSize.height, (i + 1) * modelSize.height);
				//int goodCount = __findInliers__(m1, m2, model_i, err, mask, threshold);
				int goodCount;
				{
					//cb->__computeError__(m1, m2, model, err);
					{
						int i, count = opoints.checkVector(3);
						Mat _rvec = model_i.col(0);
						Mat _tvec = model_i.col(1);


						Mat projpoints(count, 2, CV_32FC1);
						projectPoints(opoints, _rvec, _tvec, cameraMatrix, distCoeffs, projpoints);

						const Point2f* ipoints_ptr = ipoints.ptr<Point2f>();
						const Point2f* projpoints_ptr = projpoints.ptr<Point2f>();

						err.create(count, 1, CV_32FC1);

						for (i = 0; i < count; ++i)
							err.ptr<float>()[i] = (float)norm(Matx21f(ipoints_ptr[i] - projpoints_ptr[i]), NORM_L2SQR);
					}
					mask.create(err.size(), CV_8U);

					const float* errptr = err.ptr<float>();
					uchar* maskptr = mask.ptr<uchar>();
					float t = (float)(reprojectionError * reprojectionError);
					int i, n = (int)err.total();
					goodCount = 0;
					for (i = 0; i < n; i++)
					{
						int f = errptr[i] <= t;
						maskptr[i] = (uchar)f;
						goodCount += f;
					}
				}

				if (goodCount > MAX(maxGoodCount, modelPoints - 1))
				{
					std::swap(mask, bestMask);
					model_i.copyTo(bestModel);
					maxGoodCount = goodCount;
					//niters = __RANSACUpdateNumIters__(confidence, (double)(count - goodCount) / count, modelPoints, niters);
					double ep = (double)(count - goodCount) / count;
					double p = confidence;
					{
						p = MAX(p, 0.);
						p = MIN(p, 1.);
						ep = MAX(ep, 0.);
						ep = MIN(ep, 1.);

						double num = MAX(1. - p, DBL_MIN);
						double denom = 1. - pow(1. - ep, modelPoints);
						if (denom < DBL_MIN)
						{
							result = false;
							goto label;
						}

						num = log(num);
						denom = log(denom);

						niters = denom >= 0 || -num >= niters * (-denom) ? niters : cvRound(num / denom);
					}
				}
			}
		}

		transpose(bestMask, bestMask0);
		bestModel.copyTo(_model);
		result = true;
	}
label:

	vector<Point3d> opoints_inliers;
	vector<Point2d> ipoints_inliers;
	opoints = opoints.reshape(3);
	ipoints = ipoints.reshape(2);
	opoints.convertTo(opoints_inliers, CV_64F);
	ipoints.convertTo(ipoints_inliers, CV_64F);

	const uchar* mask = _mask_local_inliers.ptr<uchar>();
	int npoints1;// = __compressElems__(&opoints_inliers[0], mask, 1, npoints);
	//(T* ptr, const uchar* mask, int mstep, int count)
	{
		int i, j;
		for (i = j = 0; i < npoints; i++)
			if (mask[i])
			{
				if (i > j)
					opoints_inliers[j] = opoints_inliers[i];
				j++;
			}
		npoints1 = j;
	}
	//__compressElems__(&ipoints_inliers[0], mask, 1, npoints);
	{
		int i, j;
		for (i = j = 0; i < npoints; i++)
			if (mask[i])
			{
				if (i > j)
					ipoints_inliers[j] = ipoints_inliers[i];
				j++;
			}
	}

	opoints_inliers.resize(npoints1);
	ipoints_inliers.resize(npoints1);
	result = __solvePnP__(opoints_inliers, ipoints_inliers, cameraMatrix,
		distCoeffs, rvec, tvec, flags) ? 1 : -1;

	return true;
}