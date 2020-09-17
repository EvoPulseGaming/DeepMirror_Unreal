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
	Frame = cv::Mat();
	gray = cv::Mat();
	FrameToLightSize = cv::Mat();
	LightEstimationFrame = cv::Mat();
	smallMat = cv::Mat();
	StreamIn.clear();
	inputBlob = cv::Mat();


	//FMemory::Free(&pose_model_shape_predictor);
	//pose_model_shape_predictor
	//FMemory::Free(&r);
	detection = cv::Mat();
	detectionMat = cv::Mat();
	//FMemory::Free(&object_compare_pts);
	//FMemory::Free(&image_pts);
	//FMemory::Free(&reprojectdst);
	//FMemory::Free(&reprojectsrc);

	//FMemory::Free(&net);

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
		Frame.release();
		gray.release();
		FrameToLightSize.release();
		LightEstimationFrame.release();
		smallMat.release();
		StreamIn.clear();
		inputBlob.release();

		//FMemory::Free(&pose_model_shape_predictor);

		//FMemory::Free(&r);

		detection.release();
		detectionMat.release();
		object_compare_pts.clear();
		image_pts.clear();
		reprojectdst.clear();
		reprojectsrc.clear();

		//FMemory::Free(&net);

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
		VideoSize = FVector2D(Frame.cols, Frame.rows);

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



		net = cv::dnn::readNetFromCaffe(proto, caffe);
		if (net.empty())
		{
			std::cerr << "Can't load network by using the following files: " << std::endl;
			std::cerr << "prototxt:   " << proto << std::endl;
			std::cerr << "caffemodel: " << caffe << std::endl;
			exit(-1);
		}


		FString ShapePath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("TestRes/shape_predictor_68_face_landmarks.dat"));

		StreamIn = std::ifstream(TCHAR_TO_UTF8(*ShapePath), std::ios::binary);


		dlib::deserialize(pose_model_shape_predictor, StreamIn);


		//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
		object_compare_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
		object_compare_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
		object_compare_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
		object_compare_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
		object_compare_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
		object_compare_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
		object_compare_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
		object_compare_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
		object_compare_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
		object_compare_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
		object_compare_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
		object_compare_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
		object_compare_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
		object_compare_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner


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
		stream.retrieve(Frame);
	}
	else
	{
		isStreamOpen = false;
	}
}


void AWebcamActor::UpdateTexture()
{
	if (stream.isOpened() && Frame.data)
	{
		for (int y = 0; y < VideoSize.Y; y++)
		{
			for (int x = 0; x < VideoSize.X; x++)
			{
				int i = x + (y * VideoSize.X);
				CameraData[i].B = Frame.data[i * 3 + 0];
				CameraData[i].G = Frame.data[i * 3 + 1];
				CameraData[i].R = Frame.data[i * 3 + 2];
			}
		}

		UpdateTextureRegions(VideoTexture, (int32)0, (uint32)1, VideoUpdateTextureRegion, (uint32)(4 * VideoSize.X), (uint32)4, (uint8*)CameraData.GetData(), false);


		cv::resize(Frame, FrameToLightSize, cv::Size(xSizeLight, ySizeLight), cv::INTER_LANCZOS4);

		//https://www.researchgate.net/publication/274640792_Illumination_Estimation_Based_Color_to_Grayscale_Conversion_Algorithms

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

	if (stream.isOpened() && Frame.empty() && !Frame.data)
		return;

	if (Frame.channels() == 4)
	{
		cv::cvtColor(Frame, Frame, cv::COLOR_BGRA2BGR);
	}

	float inScaleFactor = 1;
	float meanVal = 1;

	inputBlob = cv::dnn::blobFromImage(Frame, inScaleFactor, cv::Size(400, 300), meanVal, false, false);


	net.setInput(inputBlob, "data");
	detection = net.forward("detection_out");
	detectionMat = cv::Mat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());


	cv::resize(Frame, smallMat, cv::Size(), 1.0 / FACE_DOWNSAMPLE_RATIO, 1.0 / FACE_DOWNSAMPLE_RATIO);
	dlib::cv_image<dlib::bgr_pixel> cimg(Frame);
	dlib::cv_image<dlib::bgr_pixel> cimg_small(smallMat);


	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);
		float confidenceThreshold = 0.5f;
		if (confidence > confidenceThreshold)
		{
			int x1 = static_cast<int>(detectionMat.at<float>(i, 3) * Frame.cols);
			int y1 = static_cast<int>(detectionMat.at<float>(i, 4) * Frame.rows);
			int x2 = static_cast<int>(detectionMat.at<float>(i, 5) * Frame.cols);
			int y2 = static_cast<int>(detectionMat.at<float>(i, 6) * Frame.rows);

			dlib::rectangle r(
				static_cast<int>(detectionMat.at<float>(i, 3) * Frame.cols),
				static_cast<int>(detectionMat.at<float>(i, 4) * Frame.rows),
				static_cast<int>(detectionMat.at<float>(i, 5) * Frame.cols),
				static_cast<int>(detectionMat.at<float>(i, 6) * Frame.rows)
			);

			shape = pose_model_shape_predictor(cimg, r);



			shapes.push_back(shape);



			//Render square around face
			cv::rectangle(Frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2, 4);
			//draw features
			for (unsigned int i = 0; i < 68; ++i)
			{
				cv::circle(Frame, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), 2);
			}




			//fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
			//2D ref points(image coordinates), referenced from detected facial feature
			image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
			image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
			image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
			image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
			image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
			image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
			image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
			image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
			image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
			image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
			image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
			image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
			image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
			image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

			// Camera internals
			focal_length = Frame.cols; // Approximate focal length.

			center = cv::Point2d(Frame.cols / 2, Frame.rows / 2);

			camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);

			dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion


			// Solve for pose

			cv::solvePnP(object_compare_pts, image_pts, camera_matrix, dist_coeffs, rotation_vector, translation_vector);


			//rotation_vector is axis angle representation of rotation, which usually require 4 numbers, [v, theta], 
			//but v is required to be unit vector, and thus it's length is encoded as theta, reducing the required numbers to 3


			// Project a 3D point (0, 0, 1000.0) onto the image plane.
			// We use this to draw a line sticking out of the nose
			nose_end_point3D.push_back(cv::Point3d(0, 0, 1000.0));

			projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D, cv::noArray(), 1);


			HeadLocation.X = translation_vector.at<double>(0);
			HeadLocation.Y = translation_vector.at<double>(1);
			HeadLocation.Z = translation_vector.at<double>(2);

			//All this just for rotation
			cv::Rodrigues(rotation_vector, rotation_mat);
			//UE4 = X Forward, Z Up, Y Right
			

			cv::hconcat(rotation_mat, translation_vector, pose_mat);
			cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle); //XYZ




			
			//y_rot = asin(rotation_mat[2][0])
			//	x_rot = acos(rotation_mat[2][2] / math.cos(y_rot))
			//	z_rot = acos(rotation_mat[0][0] / math.cos(y_rot))
			//	y_rot_angle = y_rot * (180 / 3.1415)
			//	x_rot_angle = x_rot * (180 / 3.1415)
			//	z_rot_angle = z_rot * (180 / 3.1415)


			HeadRotator.Pitch = -euler_angle.at<double>(0);
			HeadRotator.Yaw = euler_angle.at<double>(1);
			HeadRotator.Roll = -euler_angle.at<double>(2);




			image_pts.clear();
			shapes.clear();
			nose_end_point3D.clear();


		}

	}

}

void AWebcamActor::EndPlay(EEndPlayReason::Type reasonType)
{
	if (isStreamOpen)
	{
		GetWorldTimerManager().ClearTimer(timerHandle);
		isStreamOpen = false;

		stream.release();
		Frame.release();
		gray.release();
		FrameToLightSize.release();
		LightEstimationFrame.release();
		smallMat.release();
		StreamIn.clear();
		inputBlob.release();

		//FMemory::Free(&pose_model_shape_predictor);

		//FMemory::Free(&r);

		detection.release();
		detectionMat.release();
		object_compare_pts.clear();
		image_pts.clear();
		reprojectdst.clear();
		reprojectsrc.clear();

		//FMemory::Free(&net);

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


