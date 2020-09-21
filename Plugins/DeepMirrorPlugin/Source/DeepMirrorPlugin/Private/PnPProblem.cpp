/*
 * PnPProblem.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */
#include "PnPProblem.h"
#include <iostream>
#include <sstream>





 /* Functions for Möller-Trumbore intersection algorithm */
static cv::Point3f CROSS(cv::Point3f v1, cv::Point3f v2)
{
	cv::Point3f tmp_p;
	tmp_p.x = v1.y*v2.z - v1.z*v2.y;
	tmp_p.y = v1.z*v2.x - v1.x*v2.z;
	tmp_p.z = v1.x*v2.y - v1.y*v2.x;
	return tmp_p;
}

static double DOT(cv::Point3f v1, cv::Point3f v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

static cv::Point3f SUB(cv::Point3f v1, cv::Point3f v2)
{
	cv::Point3f tmp_p;
	tmp_p.x = v1.x - v2.x;
	tmp_p.y = v1.y - v2.y;
	tmp_p.z = v1.z - v2.z;
	return tmp_p;
}

/* End functions for Möller-Trumbore intersection algorithm */

// Function to get the nearest 3D point to the Ray origin
static cv::Point3f get_nearest_3D_point(std::vector<cv::Point3f> &points_list, cv::Point3f origin)
{
	cv::Point3f p1 = points_list[0];
	cv::Point3f p2 = points_list[1];

	double d1 = std::sqrt(std::pow(p1.x - origin.x, 2) + std::pow(p1.y - origin.y, 2) + std::pow(p1.z - origin.z, 2));
	double d2 = std::sqrt(std::pow(p2.x - origin.x, 2) + std::pow(p2.y - origin.y, 2) + std::pow(p2.z - origin.z, 2));

	if (d1 < d2)
	{
		return p1;
	}
	else
	{
		return p2;
	}
}

PnPProblem::PnPProblem()
{
	// TODO Auto-generated constructor stub
}

// Custom constructor given the intrinsic camera parameters

void PnPProblem::init(const double params[])
{
	A_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
	A_matrix_.at<double>(0, 0) = params[0];       //      [ fx   0  cx ]
	A_matrix_.at<double>(1, 1) = params[1];       //      [  0  fy  cy ]
	A_matrix_.at<double>(0, 2) = params[2];       //      [  0   0   1 ]
	A_matrix_.at<double>(1, 2) = params[3];
	A_matrix_.at<double>(2, 2) = 1;
	R_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // rotation matrix
	t_matrix_ = cv::Mat::zeros(3, 1, CV_64FC1);   // translation matrix
	P_matrix_ = cv::Mat::zeros(3, 4, CV_64FC1);   // rotation-translation matrix

	cv::Mat distCoeffs_ = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat rvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);

}


PnPProblem::~PnPProblem()
{
	// TODO Auto-generated destructor stub
}

void PnPProblem::set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix)
{
	// Rotation-Translation Matrix Definition
	P_matrix_.at<double>(0, 0) = R_matrix.at<double>(0, 0);
	P_matrix_.at<double>(0, 1) = R_matrix.at<double>(0, 1);
	P_matrix_.at<double>(0, 2) = R_matrix.at<double>(0, 2);
	P_matrix_.at<double>(1, 0) = R_matrix.at<double>(1, 0);
	P_matrix_.at<double>(1, 1) = R_matrix.at<double>(1, 1);
	P_matrix_.at<double>(1, 2) = R_matrix.at<double>(1, 2);
	P_matrix_.at<double>(2, 0) = R_matrix.at<double>(2, 0);
	P_matrix_.at<double>(2, 1) = R_matrix.at<double>(2, 1);
	P_matrix_.at<double>(2, 2) = R_matrix.at<double>(2, 2);
	P_matrix_.at<double>(0, 3) = t_matrix.at<double>(0);
	P_matrix_.at<double>(1, 3) = t_matrix.at<double>(1);
	P_matrix_.at<double>(2, 3) = t_matrix.at<double>(2);
}

// Estimate the pose given a list of 2D/3D correspondences and the method to use
bool PnPProblem::estimatePose(const std::vector<cv::Point3f> &list_points3d,
	const std::vector<cv::Point2f> &list_points2d,
	int flags)
{
	bool useExtrinsicGuess = false;

	// Pose estimation
	bool correspondence = cv::solvePnP(list_points3d, list_points2d, A_matrix_, distCoeffs_, rvec_, tvec_,
		useExtrinsicGuess, flags);

	// Transforms Rotation Vector to Matrix
	Rodrigues(rvec_, R_matrix_);
	t_matrix_ = tvec_;

	// Set projection matrix
	this->set_P_matrix(R_matrix_, t_matrix_);

	return correspondence;
}

// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use

void PnPProblem::estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
	const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
	int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
	float reprojectionError, double confidence)    // Ransac parameters
{
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector

	bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
	// initial approximations of the rotation and translation vectors

	cv::solvePnPRansac(list_points3d, list_points2d, A_matrix_, distCoeffs, rvec, tvec,
		useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
		inliers, flags);

	Rodrigues(rvec, R_matrix_); // converts Rotation Vector to Matrix
	t_matrix_ = tvec;           // set translation matrix

	this->set_P_matrix(R_matrix_, t_matrix_); // set rotation-translation matrix

}


std::vector<double> PnPProblem::mat_toEular(cv::Mat m, int order)
{


		// assumes the upper 3x3 of m is a pure rotation matrix (i.e, unscaled)
	//double m00 = rotationMatrix.at<double>(0, 0);
	//double m02 = rotationMatrix.at<double>(0, 2);
	//double m10 = rotationMatrix.at<double>(1, 0);
	//double m11 = rotationMatrix.at<double>(1, 1);
	//double m12 = rotationMatrix.at<double>(1, 2);
	//double m20 = rotationMatrix.at<double>(2, 0);
	//double m22 = rotationMatrix.at<double>(2, 2);

		const double m11 = m.at<double>(0,0), m12 = m.at<double>(0,1), m13 = m.at<double>(0,2);
		const double m21 = m.at<double>(1,0), m22 = m.at<double>(1,1), m23 = m.at<double>(1,2);
		const double m31 = m.at<double>(2,0), m32 = m.at<double>(2,1), m33 = m.at<double>(2,2);
		double x=0;
		double y=0;
		double z=0;

		//RotationOrders = [ 0'XYZ', 1'YZX', 2'ZXY', 3'XZY', 4'YXZ', 5'ZYX' ];
		switch (order) {

		case 0://'XYZ':

			y = FMath::Asin(FMath::FMath::Clamp(m13, -1.0, 1.0));

			if (FMath::Abs(m13) < 0.9999999) {

				x = FMath::Atan2(-m23, m33);
				z = FMath::Atan2(-m12, m11);

			}
			else {

				x = FMath::Atan2(m32, m22);
				z = 0;

			}

			break;

		case 1://'YXZ':

			x = FMath::Asin(-FMath::Clamp(m23, -1.0, 1.0));

			if (FMath::Abs(m23) < 0.9999999) {

				y = FMath::Atan2(m13, m33);
				z = FMath::Atan2(m21, m22);

			}
			else {

				y = FMath::Atan2(-m31, m11);
				z = 0;

			}

			break;

		case 2://'ZXY':

			x = FMath::Asin(FMath::Clamp(m32, -1.0, 1.0));

			if (FMath::Abs(m32) < 0.9999999) {

				y = FMath::Atan2(-m31, m33);
				z = FMath::Atan2(-m12, m22);

			}
			else {

				y = 0;
				z = FMath::Atan2(m21, m11);

			}

			break;

		case 3://'ZYX':

			y = FMath::Asin(-FMath::Clamp(m31, -1.0, 1.0));

			if (FMath::Abs(m31) < 0.9999999) {

				x = FMath::Atan2(m32, m33);
				z = FMath::Atan2(m21, m11);

			}
			else {

				x = 0;
				z = FMath::Atan2(-m12, m22);

			}

			break;

		case 4://'YZX':

			z = FMath::Asin(FMath::Clamp(m21, -1.0, 1.0));

			if (FMath::Abs(m21) < 0.9999999) {

				x = FMath::Atan2(-m23, m22);
				y = FMath::Atan2(-m31, m11);

			}
			else {

				x = 0;
				y = FMath::Atan2(m13, m33);

			}

			break;

		case 5://'XZY':

			z = FMath::Asin(-FMath::Clamp(m12, -1.0, 1.0));

			if (FMath::Abs(m12) < 0.9999999) {

				x = FMath::Atan2(m32, m22);
				y = FMath::Atan2(m13, m11);

			}
			else {

				x = FMath::Atan2(-m23, m33);
				y = 0;

			}

			break;
		}
		double degrees = 180.0 / CV_PI;
		//return  { x*degrees ,y*degrees ,z*degrees };
		return{ x*degrees,y*degrees,z*degrees };
}