//#include "CameraCalibration.h"
//
//
////number of internal corners horizontally and the number of internal corners vertically
////numCornersHor - number of corners along width
////numCornersVer - number of corners along height
////numBoards - number of captures (# of calibrations)
//int CameraCalibration::Calibrate(cv::Mat image, int numBoards, int numCornersHor, int numCornersVer)
//{
//	int numSquares = numCornersHor * numCornersVer;
//	cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);
//
//
////What do these mean ? For those unfamiliar with C++, a "vector" is a list.This list contains items of the type mentioned within the angular brackets < >
////(it's called generic programming). So, we're creating a list of list of 3D points(Point3f) and a list of list of 2D points(Point2f).
//
////object_points is the physical position of the corners(in 3D space).This has to be measured by us.
////[write relationg between each list item and list's eh you get the point]
//
////image_points is the location of the corners on in the image(in 2 dimensions).Once the program has actual physical locations and locations on the image, 
////it can calculate the relation between the two.
//
////And because we'll use a chessboard, these points have a definite relations between them (they lie on straight lines and on squares). 
////So the "expected" - "actual" relation can be used to correct the distortions in the image.
//	std::vector<std::vector<cv::Point3f>> object_points;
//	std::vector<std::vector<cv::Point2f>> image_points;
//
////Next, we create a list of corners.This will temporarily hold the current snapshot's chessboard corners. 
////We also declare a variable that will keep a track of successfully capturing a chessboard and saving it into the lists we declared above. 
//
//	std::vector<cv::Point2f> corners;
//	int successes = 0;
//
//
//
//
////Next, we do a little hack with object_points.Ideally, it should contain the physical position of each corner.
////The most intuitive way would be to measure distances "from" the camera lens.That is, 
////the camera is the origin and the chessboard has been displaced.
//
////Usually, it's done the other way round. The chessboard is considered the origin of the world.
////So, it is the camera that is moving around, taking different shots of the camera. 
////So, you can set the chessboard on some place (like the XY plane, of ir you like, the XZ plane). 
//
////Mathematically, it makes no difference which convention you choose.
////But it's easier for us and computationally faster in the second case. We just assign a constant position to each vertex.
////And we do that next: 
//
//	std::vector<cv::Point3f> obj;
//	for (int j = 0; j < numSquares; j++)
//		obj.push_back(cv::Point3f(j / numCornersHor, j%numCornersHor, 0.0f));
//
////This creates a list of coordinates(0, 0, 0), (0, 1, 0), (0, 2, 0)...(1, 4, 0)... so on.Each corresponds to a particular vertex.
//
////An important point here is that you're essentially setting up the units of calibration. Suppose the squares in your chessboards were 30mm in size, and you supplied these coordinates as (0,0,0), (0, 30, 0), etc, you'd get all unknowns in millimeters.
//
////We're not really concerned with the physical dimensions, so we used a random unit system.
//
////Now, for the loop.As long as the number of successful entries has been less than the number required, we keep looping :
//
//
//	while (successes < numBoards)
//	{
////Next, we convert the image into a grayscale image :
//
//		cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
//
////And we're here. The key functions:
//
//			bool found = findChessboardCorners(image, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
//
//		if (found)
//		{
//			cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//			cv::drawChessboardCorners(gray_image, board_sz, corners, found);
//		}
//
////The findChessboardCorners does exactly what it says.It looks for board_sz sized corners in image.If it detects such a pattern, their pixel locations are stored in corners and found becomes non - zero.The flags in the last parameter are used to improve the chances of detecting the corners.Check the OpenCV documentation for details about the three flags that can be used.
//
////If corners are detected, they are further refined.Subpixel corners are calculated from the grayscale image.This is an iterative process, so you need to provide a termination criteria(number of iterations, amount of error allowed, etc).
//
////Also, if corners are detected, they're drawn onto the screen using the handy _drawChessboardCorners _function!
//
////Next we update the display the images and grab another frame.We also try to capture a key :
//
//		imshow("win1", image);
//		imshow("win2", gray_image);
//
//		//capture >> image;
//		//int key = waitKey(1);
//
////If escape is pressed, we quit.No questions asked.If corners were found and space bar was pressed, we store the results into the lists.And if we reach the required number of snaps, we break the while loop too :
//
////if escape return
//		//if pressed space:
//			image_points.push_back(corners);
//			object_points.push_back(obj);
//
//			printf("Snap stored!");
//
//			successes++;
//
//			if (successes >= numBoards)
//				break;
//		//}
//	//}
//	}
//
////Next, we get ready to do the calibration.We declare variables that will hold the unknowns :
//
//	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
//	cv::Mat distCoeffs;
//	std::vector<cv::Mat> rvecs;
//	std::vector<cv::Mat> tvecs;
//
////We modify the intrinsic matrix with whatever we do know.The camera's aspect ratio is 1 (that's usually the case... If not, change it as required).
//
//	intrinsic.ptr<float>(0)[0] = 1;
//	intrinsic.ptr<float>(1)[1] = 1;
//
////Elements(0, 0) and (1, 1) are the focal lengths along the X and Y axis.
//
//	//And finally, the calibration :
//
//	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
//
////After this statement, you'll have the intrinsic matrix, distortion coefficients and the rotation+translation vectors. The intrinsic matrix and distortion coefficients are a property of the camera and lens. So as long as you use the same lens (ie you don't change it, or change its focal length, like in zoom lenses etc) you can reuse them.In fact, you can save them to a file if you want and skip the entire chessboard circus!
//
////Note: The calibrateCamera function converts all matrices into 64F format even if you initialize it to 32F.Thanks to Michael Koval!
//
//
//
//	return 0;
//}
//
////Now that we have the distortion coefficients, we can undistort the images
//
//cv::Mat CameraCalibration::undistortimage(cv::Mat inFrame, cv::Mat intrinsic, cv::Mat distCoeffs) {
//	cv::Mat outFrame;
//
//		undistort(inFrame, outFrame, intrinsic, distCoeffs);
//
//		return outFrame;
//}
