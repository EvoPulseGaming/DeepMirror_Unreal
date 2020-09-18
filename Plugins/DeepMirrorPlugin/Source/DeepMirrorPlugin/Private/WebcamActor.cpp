#include "WebcamActor.h"
#include "TimerManager.h"
#include "Engine/Texture2D.h"

#include "DeepMirrorPlugin.h"

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

	camera_matrix = cv::Mat();
	dist_coeffs = cv::Mat();
	rotation_vector = cv::Mat();
	translation_vector = cv::Mat();
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

		camera_matrix.release();
		dist_coeffs.release();
		rotation_vector.release();
		translation_vector.release();
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




		FString deploypath = FString::Printf(TEXT("S:/GitHub/DeepMirror_Unreal/Plugins/DeepMirrorPlugin/Source/caffeemodels/deploy.prototxt"));
		FString caffemodelpath = FString::Printf(TEXT("S:/GitHub/DeepMirror_Unreal/Plugins/DeepMirrorPlugin/Source/caffeemodels/res10_300x300_ssd_iter_140000_fp16.caffemodel"));


		std::string proto = "S:/GitHub/DeepMirror_Unreal/Plugins/DeepMirrorPlugin/Source/caffeemodels/deploy.prototxt";
		std::string caffe = "S:/GitHub/DeepMirror_Unreal/Plugins/DeepMirrorPlugin/Source/caffeemodels/res10_300x300_ssd_iter_140000_fp16.caffemodel";



		box_detector_face_net = cv::dnn::readNetFromCaffe(proto, caffe);
		if (box_detector_face_net.empty())
		{
			std::cerr << "Can't load network by using the following files: " << std::endl;
			std::cerr << "prototxt:   " << proto << std::endl;
			std::cerr << "caffemodel: " << caffe << std::endl;
			exit(-1);
		}


		FString ShapePath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("TestRes/shape_predictor_68_face_landmarks.dat"));

		StreamIn = std::ifstream(TCHAR_TO_UTF8(*ShapePath), std::ios::binary);


		dlib::deserialize(landmark_detector_shape_predictor, StreamIn);


		//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
		//model_3D_compare_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
		//model_3D_compare_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
		//model_3D_compare_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
		//model_3D_compare_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
		//model_3D_compare_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
		//model_3D_compare_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
		//model_3D_compare_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
		//model_3D_compare_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
		//model_3D_compare_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
		//model_3D_compare_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
		//model_3D_compare_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
		//model_3D_compare_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
		//model_3D_compare_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
		//model_3D_compare_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner

		model_3D_compare_pts.push_back(cv::Point3d(0.0f, 0.0f, 60.f));               // Nose tip
		model_3D_compare_pts.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
		model_3D_compare_pts.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
		model_3D_compare_pts.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
		model_3D_compare_pts.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
		model_3D_compare_pts.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner


		//result
		pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
		euler_angle = cv::Mat(3, 1, CV_64FC1);

		//reproject 3D points world coordinate axis to verify result pose
		reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
		reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
		reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
		reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
		reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
		reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
		reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
		reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

		//reprojected 2D points

		reprojectdst.resize(8);

		//temp buf for decomposeProjectionMatrix()
		out_intrinsics = cv::Mat(3, 3, CV_64FC1);
		out_rotation = cv::Mat(3, 3, CV_64FC1);
		out_translation = cv::Mat(3, 1, CV_64FC1);

		// Do first frame
		//UpdateTexture();
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Camera error, stream wont open"))
	}

	GetWorldTimerManager().SetTimer(timerHandle, this, &AWebcamActor::CameraTimerTick, 0.08f, true, 3);

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

void AWebcamActor::UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData)
{
	if (Texture->Resource)
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
	blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), (104, 117, 123), false, false);

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
		std::vector<cv::Point2d> image_points;
		image_points.push_back(cv::Point2d(face_landmark.part(30).x() * scale_ratio, face_landmark.part(30).y() * scale_ratio));    // Nose tip
		image_points.push_back(cv::Point2d(face_landmark.part(8).x() * scale_ratio, face_landmark.part(8).y() * scale_ratio));      // Chin
		image_points.push_back(cv::Point2d(face_landmark.part(36).x() * scale_ratio, face_landmark.part(36).y() * scale_ratio));    // Left eye left corner
		image_points.push_back(cv::Point2d(face_landmark.part(45).x() * scale_ratio, face_landmark.part(45).y() * scale_ratio));    // Right eye right corner
		image_points.push_back(cv::Point2d(face_landmark.part(48).x() * scale_ratio, face_landmark.part(48).y() * scale_ratio));    // Left Mouth corner
		image_points.push_back(cv::Point2d(face_landmark.part(54).x() * scale_ratio, face_landmark.part(54).y() * scale_ratio));    // Right mouth corner

		// Generate fake camera Matrix
		double focal_length = frame.cols;
		cv::Point2d center = cv::Point2d(frame.cols / 2, frame.rows / 2);
		cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
		cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

		// Output rotation and translation
		cv::Mat rotation_vector;
		cv::Mat translation_vector;
		cv::Mat rot_mat;

		// Solve for pose
		cv::solvePnP(model_3D_compare_pts, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

		// Convert rotation to Matrix
		cv::Rodrigues(rotation_vector, rot_mat);

		//cv::Mat identity = (cv::Mat_<double>(3, 3) <<
		//	1, 0, 0,
		//	0, -1, 0,
		//	0, 0, -1);

		//rot_mat = rot_mat * identity * rot_mat;
		//FMatrix ConvertRotationMatrix = FMatrix(
		//	FPlane(0, 0, 1, 0),//x to y
		//	FPlane(1, 0, 0, 0),//y to -z
		//	FPlane(0, -1, 0, 0),
		//	FPlane(0, 0, 0, 1));

		//FMatrix ViewRotationMatrix = FMatrix(
		//	FPlane(rot_mat.at<double>(0), rot_mat.at<double>(1), rot_mat.at<double>(2), 0),
		//	FPlane(rot_mat.at<double>(3), rot_mat.at<double>(4), rot_mat.at<double>(5), 0),
		//	FPlane(rot_mat.at<double>(6), rot_mat.at<double>(7), rot_mat.at<double>(8), 0),
		//	FPlane(0, 0, 0, 1));

		//FMatrix ConvertedMatrix = ConvertRotationMatrix * ViewRotationMatrix * ConvertRotationMatrix;


		//HeadRotator = ConvertedMatrix.Rotator();



		// Export transform
		//outFaces = TransformData(translation_vector.at<double>(0), translation_vector.at<double>(1), translation_vector.at<double>(2),
		//	rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2),
		//	rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2));



		//OPENCV =	right-handed	(+X: right,		+Y: down,	+Z: forward)
		//UE4	 =	left-handed		(+X: forward,	+Y: right,	+Z: up)


		cv::hconcat(rot_mat, translation_vector, pose_mat);
		cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle); //XYZ


		HeadRotator.Pitch = -euler_angle.at<double>(0);
		HeadRotator.Yaw = -euler_angle.at<double>(1);
		HeadRotator.Roll = -euler_angle.at<double>(2);


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

		camera_matrix.release();
		dist_coeffs.release();
		rotation_vector.release();
		translation_vector.release();
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


