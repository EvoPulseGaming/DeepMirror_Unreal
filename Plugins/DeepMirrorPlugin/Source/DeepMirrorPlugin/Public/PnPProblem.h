/*
 * PnPProblem.h
 *
 *  Created on: Mar 28, 2014
 *      Author: Edgar Riba
 */

#ifndef PNPPROBLEM_H_
#define PNPPROBLEM_H_

#include <iostream>

#include "OpenCV_Common.h"


class PnPProblem
{
public:
	PnPProblem();  // defualt constructor
	void init(const double params[]);  // custom constructor
	virtual ~PnPProblem();

	bool estimatePose(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, int flags);
	void estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d,
		int flags, cv::Mat &inliers,
		int iterationsCount, float reprojectionError, double confidence);

	std::vector<double> mat_toEular(cv::Mat m, int order);

	cv::Mat get_A_matrix()		const { return A_matrix_;	 }
	cv::Mat get_R_matrix()		const { return R_matrix_;	 }
	cv::Mat get_t_matrix()		const { return t_matrix_;	 }
	cv::Mat get_P_matrix()		const { return P_matrix_;	 }
	cv::Mat get_distCoeffs()	const { return distCoeffs_;	 }
	cv::Mat get_rvec()			const { return rvec_;		 }
	cv::Mat get_tvec()			const { return tvec_;		 }

	void set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix);

private:
	/** The calibration matrix */
	cv::Mat A_matrix_;
	/** The computed rotation matrix */
	cv::Mat R_matrix_;
	/** The computed translation matrix */
	cv::Mat t_matrix_;
	/** The computed projection matrix */
	cv::Mat P_matrix_;

	cv::Mat distCoeffs_;
	cv::Mat rvec_;
	cv::Mat tvec_;
};

#endif /* PNPPROBLEM_H_ */
