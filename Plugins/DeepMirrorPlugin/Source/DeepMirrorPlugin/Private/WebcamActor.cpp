#include "WebcamActor.h"
#include "TimerManager.h"
#include "Engine/Texture2D.h"

#include "DeepMirrorPlugin.h"
#include "Interfaces/IPluginManager.h"
#include "OpenCV_Common.h"
#include "cmath"

#define DETECT_BUFFER_SIZE 0x20000

AWebcamActor::AWebcamActor()
{
	PrimaryActorTick.bCanEverTick = true;

	CameraID = 0;
	RefreshFPS = 15.f;
	RefreshTime = 1000000.0f;
	bDebugFaceLandmarks = true;
	VideoSize = FVector2D(0, 0);
	stream = cv::VideoCapture();
	frame = cv::Mat();
	gray = cv::Mat();
	FrameToLightSize = cv::Mat();
	LightEstimationFrame = cv::Mat();
	smallMat = cv::Mat();
	StreamIn.clear();
	inputBlob = cv::Mat();


	//FMemory::Free(&landmark_detector_shape_predictor);
	//landmark_detector_shape_predictor
	//FMemory::Free(&face_rect);
	detection = cv::Mat();
	detectionMat = cv::Mat();
	//FMemory::Free(&model_3D_compare_pts);
	//FMemory::Free(&image_landmarks_pts);
	//FMemory::Free(&reprojectdst);
	//FMemory::Free(&reprojectsrc);

	//FMemory::Free(&box_detector_face_net);

	//FMemory::Free(&shapes);
	//FMemory::Free(&shape);
	//FMemory::Free(&center);

	rotation_vector = cv::Mat();

	nose_end_point3D.empty();
	nose_end_point2D.empty();
	rotation_mat = cv::Mat();
	pose_mat = cv::Mat();
	euler_angle = cv::Mat();
	out_intrinsics = cv::Mat();
	out_rotation = cv::Mat();
	out_translation = cv::Mat();






}

AWebcamActor::~AWebcamActor()
{
	if (!bMemoryReleased)
	{
		isStreamOpen = false;

		stream.release();
		frame.release();
		gray.release();
		FrameToLightSize.release();
		LightEstimationFrame.release();
		smallMat.release();
		StreamIn.clear();
		inputBlob.release();

		//FMemory::Free(&landmark_detector_shape_predictor);

		//FMemory::Free(&face_rect);

		detection.release();
		detectionMat.release();
		model_3D_compare_pts.clear();
		//image_landmarks_pts.clear();
		reprojectdst.clear();
		reprojectsrc.clear();

		//FMemory::Free(&box_detector_face_net);

		shapes.clear();
		//FMemory::Free(&shape);
		//FMemory::Free(&center);


		rotation_vector.release();

		nose_end_point3D.clear();
		nose_end_point2D.clear();
		rotation_mat.release();
		pose_mat.release();
		euler_angle.release();
		out_intrinsics.release();
		out_rotation.release();
		out_translation.release();
	}
}




void AWebcamActor::BeginPlay()
{
	Super::BeginPlay();

	bMemoryReleased = false;


	stream.open(CameraID);
	if (stream.grab() && stream.isOpened())
	{

		isStreamOpen = true;
		UpdateFrame();
		VideoSize = FVector2D(frame.cols, frame.rows);

		VideoTexture = UTexture2D::CreateTransient(VideoSize.X, VideoSize.Y);
		VideoTexture->UpdateResource();
		VideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, VideoSize.X, VideoSize.Y);
		CameraData.Init(FColor(0, 0, 0, 255), VideoSize.X * VideoSize.Y);

		xSizeLight = xSizeScale * VideoSize.X;
		ySizeLight = ySizeScale * VideoSize.Y;

		LightEstVideoTexture = UTexture2D::CreateTransient(xSizeLight, ySizeLight, PF_R8_UINT);
		LightEstVideoTexture->UpdateResource();
		LightEstVideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, xSizeLight, ySizeLight);
		EstimatedLights.Init(0, xSizeLight * ySizeLight);



		FString ContentDir = IPluginManager::Get().FindPlugin("DeepMirrorPlugin")->GetBaseDir();

		std::string proto = std::string(TCHAR_TO_UTF8(*ContentDir)) +"/Source/caffeemodels/deploy.prototxt";
		std::string caffe = std::string(TCHAR_TO_UTF8(*ContentDir)) +"/Source/caffeemodels/res10_300x300_ssd_iter_140000_fp16.caffemodel";



		box_detector_face_net = cv::dnn::readNetFromCaffe(proto, caffe);
		if (box_detector_face_net.empty())
		{
			std::cerr << "Can't load network by using the following files: " << std::endl;
			std::cerr << "prototxt:   " << proto << std::endl;
			std::cerr << "caffemodel: " << caffe << std::endl;
			exit(-1);
		}


		std::string ShapePath = std::string(TCHAR_TO_UTF8(*ContentDir)) + "/Source/caffeemodels/shape_predictor_68_face_landmarks.dat";

		StreamIn = std::ifstream(ShapePath, std::ios::binary);


		dlib::deserialize(landmark_detector_shape_predictor, StreamIn);


		//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
		//model_3D_compare_pts.push_back(cv::Point3f(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
		//model_3D_compare_pts.push_back(cv::Point3f(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
		//model_3D_compare_pts.push_back(cv::Point3f(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
		//model_3D_compare_pts.push_back(cv::Point3f(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
		//model_3D_compare_pts.push_back(cv::Point3f(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
		//model_3D_compare_pts.push_back(cv::Point3f(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
		//model_3D_compare_pts.push_back(cv::Point3f(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
		//model_3D_compare_pts.push_back(cv::Point3f(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
		//model_3D_compare_pts.push_back(cv::Point3f(2.005628, 1.409845, 6.165652));     //#55 nose left corner
		//model_3D_compare_pts.push_back(cv::Point3f(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
		//model_3D_compare_pts.push_back(cv::Point3f(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
		//model_3D_compare_pts.push_back(cv::Point3f(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
		//model_3D_compare_pts.push_back(cv::Point3f(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
		//model_3D_compare_pts.push_back(cv::Point3f(0.000000, -7.415691, 4.070434));    //#6 chin corner

		model_3D_compare_pts.push_back(cv::Point3f(0.0f, 0.0f, 60.f));               // Nose tip
		model_3D_compare_pts.push_back(cv::Point3f(0.0f, -330.0f, -65.0f));          // Chin
		model_3D_compare_pts.push_back(cv::Point3f(-225.0f, 170.0f, -135.0f));       // Left eye left corner
		model_3D_compare_pts.push_back(cv::Point3f(225.0f, 170.0f, -135.0f));        // Right eye right corner
		model_3D_compare_pts.push_back(cv::Point3f(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
		model_3D_compare_pts.push_back(cv::Point3f(150.0f, -150.0f, -125.0f));       // Right mouth corner


		//result
		pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
		euler_angle = cv::Mat(3, 1, CV_64FC1);

		//reproject 3D points world coordinate axis to verify result pose
		reprojectsrc.push_back(cv::Point3f(10.0, 10.0, 10.0));
		reprojectsrc.push_back(cv::Point3f(10.0, 10.0, -10.0));
		reprojectsrc.push_back(cv::Point3f(10.0, -10.0, -10.0));
		reprojectsrc.push_back(cv::Point3f(10.0, -10.0, 10.0));
		reprojectsrc.push_back(cv::Point3f(-10.0, 10.0, 10.0));
		reprojectsrc.push_back(cv::Point3f(-10.0, 10.0, -10.0));
		reprojectsrc.push_back(cv::Point3f(-10.0, -10.0, -10.0));
		reprojectsrc.push_back(cv::Point3f(-10.0, -10.0, 10.0));

		//reprojected 2D points

		reprojectdst.resize(8);

		//temp buf for decomposeProjectionMatrix()
		out_intrinsics = cv::Mat(3, 3, CV_64FC1);
		out_rotation = cv::Mat(3, 3, CV_64FC1);
		out_translation = cv::Mat(3, 1, CV_64FC1);

		// Do first frame
		//UpdateTexture();


		//kalman filter
		int nStates = 18;            // the number of states
		int nMeasurements = 6;       // the number of measured states
		int nInputs = 0;             // the number of action control
		double dt = 0.125;           // time between measurements (1/FPS)
		// instantiate Kalman Filter
		initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
		measurements = cv::Mat(nMeasurements, 1, CV_64FC1);
		measurements.setTo(cv::Scalar(0));





		// Generate fake camera Matrix
		//double focal_length = frame.cols;
		//cv::Point2f center = cv::Point2f(frame.cols / 2, frame.rows / 2);
		//cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
		//cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1); // Assuming no lens distortion

			// Intrinsic camera parameters: UVC WEBCAM
		double f = 55;                           // focal length in mm
		double sx = 22.3, sy = 14.9;             // sensor size
		double width = 640, height = 480;        // image size

		double params_WEBCAM[] = { width*f / sx,   // fx
								   height*f / sy,  // fy
								   width / 2,      // cx
								   height / 2 };    // cy

		pnp_detection.init(params_WEBCAM);
		pnp_detection_est.init(params_WEBCAM);




	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Camera error, stream wont open"))
	}

	GetWorldTimerManager().SetTimer(timerHandle, this, &AWebcamActor::CameraTimerTick, 0.08f, true, 3);

}



void AWebcamActor::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
	KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
	cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
	cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
	cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
				   /* DYNAMIC MODEL */
	//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
	//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
	//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
	// position
	KF.transitionMatrix.at<double>(0, 3) = dt;
	KF.transitionMatrix.at<double>(1, 4) = dt;
	KF.transitionMatrix.at<double>(2, 5) = dt;
	KF.transitionMatrix.at<double>(3, 6) = dt;
	KF.transitionMatrix.at<double>(4, 7) = dt;
	KF.transitionMatrix.at<double>(5, 8) = dt;
	KF.transitionMatrix.at<double>(0, 6) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(1, 7) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(2, 8) = 0.5*pow(dt, 2);
	// orientation
	KF.transitionMatrix.at<double>(9, 12) = dt;
	KF.transitionMatrix.at<double>(10, 13) = dt;
	KF.transitionMatrix.at<double>(11, 14) = dt;
	KF.transitionMatrix.at<double>(12, 15) = dt;
	KF.transitionMatrix.at<double>(13, 16) = dt;
	KF.transitionMatrix.at<double>(14, 17) = dt;
	KF.transitionMatrix.at<double>(9, 15) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(10, 16) = 0.5*pow(dt, 2);
	KF.transitionMatrix.at<double>(11, 17) = 0.5*pow(dt, 2);
	/* MEASUREMENT MODEL */
//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
	KF.measurementMatrix.at<double>(0, 0) = 1;  // x
	KF.measurementMatrix.at<double>(1, 1) = 1;  // y
	KF.measurementMatrix.at<double>(2, 2) = 1;  // z
	KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
	KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
	KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
}

void AWebcamActor::ComputeHeadDataTick()
{
	int scale_ratio = 1;


	if (stream.isOpened() && frame.empty() && !frame.data)
		return;

	if (frame.channels() == 4)
	{
		cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
	}


	// Convert frame to blob, and drop into Face Box Detector Netowrk
	cv::Mat blob, out;
	blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), (0, 0, 0), false, false);

	box_detector_face_net.setInput(blob);
	cv::Mat detection = box_detector_face_net.forward();
	cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

	// Check results and only take the most confident prediction
	float largest_conf = 0;
	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);

		if (confidence > .5)
		{
			// Get dimensions
			int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * (frame.cols / scale_ratio));
			int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * (frame.rows / scale_ratio));
			int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * (frame.cols / scale_ratio));
			int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * (frame.rows / scale_ratio));

			// Generate square dimensions
			int face_width = FMath::Max(x2 - x1, y2 - y1) / 2.8;
			int center_x = ((x2 + x1) / 2);
			int center_y = ((y2 + y1) / 2);

			if (run_count > 0)
			{
				// Average the center of the box with the Nose (Works better for landmark detection)
				center_x = int((center_x + prev_nose.x) / 2);
				center_y = int((center_y + prev_nose.y) / 2);
			}

			// Apply square dimensions
			dlib::point point_a(center_x - face_width, center_y - face_width);
			dlib::point point_b(center_x + face_width, center_y + face_width);
			dlib::rectangle new_face(point_a, point_b);

			if (confidence > largest_conf)
			{
				largest_conf = confidence;
				face_rect = new_face;
			}

		}
	}

	//Only run this if we detect something
	if (largest_conf > 0)
	{
		run_count += 1;

		// Run landmark detection
		cv::Mat half_frame(frame.rows / scale_ratio, frame.cols / scale_ratio, frame.type());
		cv::resize(frame, half_frame, half_frame.size(), cv::INTER_CUBIC);
		dlib::cv_image<dlib::bgr_pixel> dlib_image(half_frame);
		dlib::full_object_detection face_landmark;
		face_landmark = landmark_detector_shape_predictor(dlib_image, face_rect);

		// Store nose point
		prev_nose = cv::Point2f(face_landmark.part(34).x(), face_landmark.part(34).y());

		// Prepair face points for perspective solve
		std::vector<cv::Point2f> image_points;
		image_points.push_back(cv::Point2f(face_landmark.part(30).x() * scale_ratio, face_landmark.part(30).y() * scale_ratio));    // Nose tip
		image_points.push_back(cv::Point2f(face_landmark.part(8).x() * scale_ratio, face_landmark.part(8).y() * scale_ratio));      // Chin
		image_points.push_back(cv::Point2f(face_landmark.part(36).x() * scale_ratio, face_landmark.part(36).y() * scale_ratio));    // Left eye left corner
		image_points.push_back(cv::Point2f(face_landmark.part(45).x() * scale_ratio, face_landmark.part(45).y() * scale_ratio));    // Right eye right corner
		image_points.push_back(cv::Point2f(face_landmark.part(48).x() * scale_ratio, face_landmark.part(48).y() * scale_ratio));    // Left Mouth corner
		image_points.push_back(cv::Point2f(face_landmark.part(54).x() * scale_ratio, face_landmark.part(54).y() * scale_ratio));    // Right mouth corner



		// Output rotation and translation

		cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

		bool useExtrinsicGuess = false;

		// Solve for pose
		int pnpMethod = cv::SOLVEPNP_ITERATIVE;
		pnp_detection.estimatePose(model_3D_compare_pts, image_points, pnpMethod);

		// -- Step 5: Kalman Filter
		// GOOD MEASUREMENT

				// Get the measured translation
		cv::Mat translation_measured = pnp_detection.get_t_matrix();

		// Get the measured rotation
		cv::Mat rotation_measured = pnp_detection.get_R_matrix();

		// 0 , 1 roll
		// 2 yaw
		// 5 pitch
		// 3 translation?


		/* Head rotation as quat, 
		 0 and 1 seems to be roll and w/translation for quat, rolls inverted for the moment */
		QuatHeadRotation = FQuat(rotation_measured.at<double>(0)*-1, rotation_measured.at<double>(2) * -1, rotation_measured.at<double>(5), rotation_measured.at<double>(1));
		FQuat RotAdj = FQuat(0,0,-1,0);//probably find a way not to need this
		QuatHeadRotation = RotAdj* QuatHeadRotation;

		// fill the measurements vector
		fillMeasurements(measurements, translation_measured, rotation_measured);


		// update the Kalman filter with good measurements, otherwise with previous valid measurements
		cv::Mat translation_estimated(3, 1, CV_64FC1);
		cv::Mat rotation_estimated(3, 3, CV_64FC1);
		updateKalmanFilter(KF, measurements,
			translation_estimated, rotation_estimated);

		// -- Step 6: Set estimated projection matrix
		pnp_detection_est.set_P_matrix(rotation_estimated, translation_estimated);

		HeadLocation.X = -tvec.at<double>(0);
		HeadLocation.Y = -tvec.at<double>(1);
		HeadLocation.Z = -tvec.at<double>(2);

		HeadRotator.Pitch = -rotation_estimated.at<double>(0);
		HeadRotator.Yaw = -rotation_estimated.at<double>(1);
		HeadRotator.Roll = -rotation_estimated.at<double>(2);

		// Convert rotation to Matrix


		//cv::Mat identity = (cv::Mat_<double>(3, 3) <<
		//	1, 0, 0,
		//	0, -1, 0,
		//	0, 0, -1);

		//R_matrix = R_matrix * identity * R_matrix;
		//FMatrix ConvertRotationMatrix = FMatrix(
		//	FPlane(0, 0, 1, 0),//x to y
		//	FPlane(1, 0, 0, 0),//y to -z
		//	FPlane(0, -1, 0, 0),
		//	FPlane(0, 0, 0, 1));

		//FMatrix ViewRotationMatrix = FMatrix(
		//	FPlane(R_matrix.at<double>(0), R_matrix.at<double>(1), R_matrix.at<double>(2), 0),
		//	FPlane(R_matrix.at<double>(3), R_matrix.at<double>(4), R_matrix.at<double>(5), 0),
		//	FPlane(R_matrix.at<double>(6), R_matrix.at<double>(7), R_matrix.at<double>(8), 0),
		//	FPlane(0, 0, 0, 1));

		//FMatrix ConvertedMatrix = ConvertRotationMatrix * ViewRotationMatrix * ConvertRotationMatrix;


		//HeadRotator = ConvertedMatrix.Rotator();



		// Export transform
		//outFaces = TransformData(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
		//	R_matrix.at<double>(2, 0), R_matrix.at<double>(2, 1), R_matrix.at<double>(2, 2),
		//	R_matrix.at<double>(1, 0), R_matrix.at<double>(1, 1), R_matrix.at<double>(1, 2));



		//OPENCV =	right-handed	(+X: right,		+Y: down,	+Z: forward)
		//UE4	 =	left-handed		(+X: forward,	+Y: right,	+Z: up)
		//cheatcodes found here:
		//https://github.com/mrdoob/three.js/blob/dev/src/math/Euler.js
		//cv::hconcat(rotation_estimated, translation_estimated, pose_mat);
		//cv::hconcat(pnp_detection.get_R_matrix(), tvec, pose_mat);
		//cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle); //XYZ

		//HeadLocation.X = -tvec.at<double>(0);
		//HeadLocation.Y = -tvec.at<double>(1);
		//HeadLocation.Z = -tvec.at<double>(2);

		//HeadRotator.Pitch = -euler_angle.at<double>(0);
		//HeadRotator.Yaw = -euler_angle.at<double>(1);
		//HeadRotator.Roll = -euler_angle.at<double>(2);


	}
	// -- Step X: Draw pose and coordinate frame
	std::vector<cv::Point2f> pose_points2d;
	if (largest_conf > 0)// || displayFilteredPose)
	{
		cv::drawFrameAxes(frame, pnp_detection.get_A_matrix(), pnp_detection.get_distCoeffs(),
			pnp_detection.get_rvec(), pnp_detection.get_tvec(), 150, 2);
	}
	else
	{ //Can't enable until we move away from euler in the kalman filter.
		//cv::drawFrameAxes(frame, pnp_detection_est.get_A_matrix(), pnp_detection_est.get_distCoeffs(),
		//	pnp_detection_est.get_rvec(), pnp_detection_est.get_tvec(), 150, 2);
	}

}

void AWebcamActor::fillMeasurements(cv::Mat &measurements,
	const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
	// Convert rotation matrix to euler angles
	
	std::vector<double> measured_eulers = rot2euler(rotation_measured);
	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers[0];      // roll
	measurements.at<double>(4) = measured_eulers[1];      // pitch
	measurements.at<double>(5) = measured_eulers[2];      // yaw
}

// Converts a given Rotation Matrix to Euler angles
// Convention used is Y-Z-X Tait-Bryan angles
// Reference code implementation:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler/index.htm
std::vector<double> AWebcamActor::rot2euler(const cv::Mat & rotationMatrix)
{
	return pnp_detection.mat_toEular(rotationMatrix, rot2eular_Order);
}


// Converts a given Euler angles to Rotation Matrix
// Convention used is Y-Z-X Tait-Bryan angles
// Reference:
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
cv::Mat AWebcamActor::euler2rot(const cv::Mat & euler)
{
	cv::Mat rotationMatrix(3, 3, CV_64F);

	double bank = euler.at<double>(0);
	double attitude = euler.at<double>(1);
	double heading = euler.at<double>(2);

	// Assuming the angles are in radians.
	double ch = cos(heading);
	double sh = sin(heading);
	double ca = cos(attitude);
	double sa = sin(attitude);
	double cb = cos(bank);
	double sb = sin(bank);

	double m00, m01, m02, m10, m11, m12, m20, m21, m22;

	m00 = ch * ca;
	m01 = sh * sb - ch * sa*cb;
	m02 = ch * sa*sb + sh * cb;
	m10 = sa;
	m11 = ca * cb;
	m12 = -ca * sb;
	m20 = -sh * ca;
	m21 = sh * sa*cb + ch * sb;
	m22 = -sh * sa*sb + ch * cb;

	rotationMatrix.at<double>(0, 0) = m00;
	rotationMatrix.at<double>(0, 1) = m01;
	rotationMatrix.at<double>(0, 2) = m02;
	rotationMatrix.at<double>(1, 0) = m10;
	rotationMatrix.at<double>(1, 1) = m11;
	rotationMatrix.at<double>(1, 2) = m12;
	rotationMatrix.at<double>(2, 0) = m20;
	rotationMatrix.at<double>(2, 1) = m21;
	rotationMatrix.at<double>(2, 2) = m22;

	return rotationMatrix;
}

void AWebcamActor::updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
	cv::Mat &translation_estimated, cv::Mat &rotation_estimated)
{
	// First predict, to update the internal statePre variable
	cv::Mat prediction = KF.predict();
	// The "correct" phase that is going to use the predicted value and our measurement
	cv::Mat estimated = KF.correct(measurement);
	// Estimated translation
	translation_estimated.at<double>(0) = estimated.at<double>(0);
	translation_estimated.at<double>(1) = estimated.at<double>(1);
	translation_estimated.at<double>(2) = estimated.at<double>(2);
	// Estimated euler angles
	cv::Mat eulers_estimated(3, 1, CV_64F);
	eulers_estimated.at<double>(0) = estimated.at<double>(9);
	eulers_estimated.at<double>(1) = estimated.at<double>(10);
	eulers_estimated.at<double>(2) = estimated.at<double>(11);
	// Convert estimated quaternion to rotation matrix
	rotation_estimated = eulers_estimated;// euler2rot(eulers_estimated);
}




// Draw the 3D coordinate axes
void AWebcamActor::draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f> &list_points2d)
{
	// For circles
	const int lineType = 8;
	const int radius = 4;
	cv::Scalar red(0, 0, 255);
	cv::Scalar green(0, 255, 0);
	cv::Scalar blue(255, 0, 0);
	cv::Scalar black(0, 0, 0);

	cv::Point2i origin = list_points2d[0];
	cv::Point2i pointX = list_points2d[1];
	cv::Point2i pointY = list_points2d[2];
	cv::Point2i pointZ = list_points2d[3];

	drawArrow(image, origin, pointX, red, 25, 2, lineType, 0);
	drawArrow(image, origin, pointY, green, 25, 2, lineType, 0);
	drawArrow(image, origin, pointZ, blue, 25, 2, lineType, 0);
	cv::circle(image, origin, radius / 2, black, -1, lineType);
}

// Draw an arrow into the image
void AWebcamActor::drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
	//Draw the principle line
	cv::line(image, p, q, color, thickness, line_type, shift);

	//compute the angle alpha
	double angle = atan2((double)p.y - q.y, (double)p.x - q.x);
	//compute the coordinates of the first segment
	p.x = (int)(q.x + arrowMagnitude * cos(angle + CV_PI / 4));
	p.y = (int)(q.y + arrowMagnitude * sin(angle + CV_PI / 4));
	//Draw the first segment
	cv::line(image, p, q, color, thickness, line_type, shift);
	//compute the coordinates of the second segment
	p.x = (int)(q.x + arrowMagnitude * cos(angle - CV_PI / 4));
	p.y = (int)(q.y + arrowMagnitude * sin(angle - CV_PI / 4));
	//Draw the second segment
	cv::line(image, p, q, color, thickness, line_type, shift);
}


void AWebcamActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

void AWebcamActor::CameraTimerTick()
{
	RefreshTime += 0.08f;
	if (stream.isOpened() && RefreshTime >= 1.f / RefreshFPS)
	{
		stream.grab();
		RefreshTime -= 1.f / RefreshFPS;
		UpdateFrame();
		ComputeHeadDataTick(); //refresh rate is controlled above
		UpdateTexture();
	}
}

void AWebcamActor::UpdateFrame()
{
	if (stream.isOpened())
	{
		stream.retrieve(frame);
	}
	else
	{
		isStreamOpen = false;
	}
}


void AWebcamActor::UpdateTexture()
{
	if (VideoTexture != NULL && VideoTexture->Resource) {
		if (stream.isOpened() && frame.data)
		{
			for (int y = 0; y < VideoSize.Y; y++)
			{
				for (int x = 0; x < VideoSize.X; x++)
				{
					int i = x + (y * VideoSize.X);
					CameraData[i].B = frame.data[i * 3 + 0];
					CameraData[i].G = frame.data[i * 3 + 1];
					CameraData[i].R = frame.data[i * 3 + 2];
				}
			}

			UpdateTextureRegions(VideoTexture, (int32)0, (uint32)1, VideoUpdateTextureRegion, (uint32)(4 * VideoSize.X), (uint32)4, (uint8*)CameraData.GetData(), false);


			cv::resize(frame, FrameToLightSize, cv::Size(xSizeLight, ySizeLight), cv::INTER_LANCZOS4);

			//https://www.researchgate.box_detector_face_net/publication/274640792_Illumination_Estimation_Based_Color_to_Grayscale_Conversion_Algorithms

			//at gray(src.size(), CV_8UC1, Scalar(0));


			gray.cols = FrameToLightSize.cols;
			gray.rows = FrameToLightSize.rows;
			//assuming float (CV_32F)
			//make sure that the weights sum to 1 (or less) to avoid saturation.
			//on a BGR image you should get the same as BGR2GRAY.If you use(1 / 3.0, 1 / 3.0, 1 / 3.0) instead
			//you should get an average of the 3 channels.Adjust to your liking.
			cv::transform(FrameToLightSize, gray, cv::Matx13f(0.114, 0.587, 0.299));

			for (int y = 0; y < gray.rows; y++)
			{
				for (int x = 0; x < gray.cols; x++)
				{
					int i = x + (y * gray.cols);
					EstimatedLights[i] = gray.data[i];
				}
			}

			UpdateTextureRegions(LightEstVideoTexture, (int32)0, (uint32)1, LightEstVideoUpdateTextureRegion, (uint32)(1 * xSizeLight), (uint32)1, (uint8*)EstimatedLights.GetData(), false);

		}
	}

}

void AWebcamActor::UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData)
{
	if (Texture != NULL && Texture->Resource)
	{
		struct FUpdateTextureRegionsData
		{
			FTexture2DResource* Texture2DResource;
			int32 MipIndex;
			uint32 NumRegions;
			FUpdateTextureRegion2D* Regions;
			uint32 SrcPitch;
			uint32 SrcBpp;
			uint8* SrcData;
		};

		FUpdateTextureRegionsData* RegionData = new FUpdateTextureRegionsData;

		RegionData->Texture2DResource = (FTexture2DResource*)Texture->Resource;
		RegionData->MipIndex = MipIndex;
		RegionData->NumRegions = NumRegions;
		RegionData->Regions = Regions;
		RegionData->SrcPitch = SrcPitch;
		RegionData->SrcBpp = SrcBpp;
		RegionData->SrcData = SrcData;

		ENQUEUE_UNIQUE_RENDER_COMMAND_TWOPARAMETER(
			UpdateTextureRegionsData,
			FUpdateTextureRegionsData*, RegionData, RegionData,
			bool, bFreeData, bFreeData,
			{
			for (uint32 RegionIndex = 0; RegionIndex < RegionData->NumRegions; ++RegionIndex)
			{
				int32 CurrentFirstMip = RegionData->Texture2DResource->GetCurrentFirstMip();
				if (RegionData->MipIndex >= CurrentFirstMip)
				{
					RHIUpdateTexture2D(
						RegionData->Texture2DResource->GetTexture2DRHI(),
						RegionData->MipIndex - CurrentFirstMip,
						RegionData->Regions[RegionIndex],
						RegionData->SrcPitch,
						RegionData->SrcData
						+ RegionData->Regions[RegionIndex].SrcY * RegionData->SrcPitch
						+ RegionData->Regions[RegionIndex].SrcX * RegionData->SrcBpp
						);
				}
			}
			if (bFreeData)
			{
				FMemory::Free(RegionData->Regions);
				FMemory::Free(RegionData->SrcData);
			}
			delete RegionData;
			});
	}
}


void AWebcamActor::EndPlay(EEndPlayReason::Type reasonType)
{
	if (isStreamOpen)
	{
		GetWorldTimerManager().ClearTimer(timerHandle);
		isStreamOpen = false;

		stream.release();
		frame.release();
		gray.release();
		FrameToLightSize.release();
		LightEstimationFrame.release();
		smallMat.release();
		StreamIn.clear();
		inputBlob.release();

		//FMemory::Free(&landmark_detector_shape_predictor);

		//FMemory::Free(&face_rect);

		detection.release();
		detectionMat.release();
		model_3D_compare_pts.clear();
		//image_landmarks_pts.clear();
		reprojectdst.clear();
		reprojectsrc.clear();

		//FMemory::Free(&box_detector_face_net);

		shapes.clear();
		//FMemory::Free(&shape);
		//FMemory::Free(&center);


		rotation_vector.release();

		nose_end_point3D.clear();
		nose_end_point2D.clear();
		rotation_mat.release();
		pose_mat.release();
		euler_angle.release();
		out_intrinsics.release();
		out_rotation.release();
		out_translation.release();
	}

	bMemoryReleased = true;
}


