#include "WebcamActor.h"
#include "TimerManager.h"
#include "Engine/Texture2D.h"

#include "DeepMirrorPlugin.h"

#include "OpenCV_Common.h"

#define DETECT_BUFFER_SIZE 0x20000

AWebcamActor::AWebcamActor()
{
	PrimaryActorTick.bCanEverTick = true;

	CameraID = 0;
	RefreshFPS = 15.f;
	RefreshTime = 1000000.0f;
	bDebugFaceLandmarks = true;
	VideoSize = FVector2D(0, 0);
	Webcam = new cv::VideoCapture();

}

AWebcamActor::~AWebcamActor()
{
	if (!bMemoryReleased)
	{
		FMemory::Free(Size);
		FMemory::Free(Webcam);
	}
}




void AWebcamActor::BeginPlay()
{
	Super::BeginPlay();

	bMemoryReleased = false;

	Webcam->open(CameraID);
	if (Webcam->isOpened())
	{
		bCamOpen = true;
		UpdateFrame();
		VideoSize = FVector2D(Frame.cols, Frame.rows);
		Size = new cv::Size(VideoSize.X, VideoSize.Y);
		VideoTexture = UTexture2D::CreateTransient(VideoSize.X, VideoSize.Y);
		VideoTexture->UpdateResource();
		VideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, VideoSize.X, VideoSize.Y);

		CameraData.Init(FColor(0, 0, 0, 255), VideoSize.X * VideoSize.Y);

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


		SKIP_FRAMES = 2;
		FACE_DOWNSAMPLE_RATIO = 2;


		UpdateCount = 0;

		FString ShapePath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("TestRes/shape_predictor_68_face_landmarks.dat"));

		std::ifstream StreamIn(TCHAR_TO_UTF8(*ShapePath), std::ios::binary);

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

	


		////Init kalman
		//for (int i = 0; i < 68; i++)
		//{
		//	cv::KalmanFilter KF = cv::KalmanFilter(stateNum, measureNum, 0);
		//	cv::Mat measurement = cv::Mat::zeros(measureNum, 1, CV_32F);
		//	//A ??????
		//	KF.transitionMatrix = (cv::Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
		//		0, 1, 0, 1,
		//		0, 0, 1, 0,
		//		0, 0, 0, 1);
		//	//??????????B?????
		//	setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] ????
		//	setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));//Q?????????
		//	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));//R?????????
		//	setIdentity(KF.errorCovPost, cv::Scalar::all(1));//P???????????????????
		//	randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));//?????????
		//	KFs[i] = KF;
		//	measurements[i] = measurement;

		//}









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
	if (bCamOpen && RefreshTime >= 1.f / RefreshFPS)
	{
		RefreshTime -= 1.f / RefreshFPS;
		UpdateFrame();
		UpdateTexture();
		ComputeHeadDataTick();
		//Technically you should call ComputeHeadDataTick here, not manually in blueprints on timer
		//But manually allows us to run at a slower tick so we don't bog down the app while experimenting
	}
}

void AWebcamActor::UpdateFrame()
{
	if (bCamOpen)
	{
		Webcam->read(Frame);
	}
}

void AWebcamActor::UpdateTexture()
{
	if (bCamOpen && Frame.data)
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

		UpdateTextureRegions(VideoTexture, (int32)0, (uint32)1, VideoUpdateTextureRegion,
			(uint32)(4 * VideoSize.X), (uint32)4, (uint8*)CameraData.GetData(), false);
	}
}

void AWebcamActor::UpdateTextureRegions(UTexture2D * Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D * Regions,
	uint32 SrcPitch, uint32 SrcBpp, uint8 * SrcData, bool bFreeData)
{
	if (Texture->Resource)
	{
		FUpdateTextureRegionsData* RegionData = new FUpdateTextureRegionsData;

		RegionData->Texture2DResource = (FTexture2DResource*)Texture->Resource;
		RegionData->MipIndex = MipIndex;
		RegionData->NumRegions = NumRegions;
		RegionData->Regions = Regions;
		RegionData->SrcPitch = SrcPitch;
		RegionData->SrcBpp = SrcBpp;
		RegionData->SrcData = SrcData;

		ENQUEUE_UNIQUE_RENDER_COMMAND_TWOPARAMETER(UpdateTextureRegionsData, FUpdateTextureRegionsData*,
			RegionData, RegionData,
			bool, bFreeData, bFreeData,
			{

				for (uint32 regionIndex = 0; regionIndex < RegionData->NumRegions; ++regionIndex)
				{
					int32 currentFirstMip = RegionData->Texture2DResource->GetCurrentFirstMip();
					if (RegionData->MipIndex >= currentFirstMip)
					{
						RHIUpdateTexture2D(
							RegionData->Texture2DResource->GetTexture2DRHI(),
							RegionData->MipIndex - currentFirstMip,
							RegionData->Regions[regionIndex],
							RegionData->SrcPitch,
							RegionData->SrcData + RegionData->Regions[regionIndex].SrcY * RegionData->SrcPitch
							+ RegionData->Regions[regionIndex].SrcX * RegionData->SrcBpp
						);
					}
				}

			}
			if (bFreeData)
			{
				FMemory::Free(RegionData->Regions);
				FMemory::Free(RegionData->SrcData);
			}
			delete RegionData;
			);

	}
}

static cv::Rect dlibRectangleToOpenCV(dlib::rectangle r)
{
	return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
}

static dlib::rectangle openCVRectToDlib(cv::Rect r)
{
	return dlib::rectangle((long)r.tl().x, (long)r.tl().y, (long)r.br().x - 1, (long)r.br().y - 1);
}

void AWebcamActor::ComputeHeadDataTick()
{

	if (Frame.empty() && !Frame.data)
		return;

	if (Frame.channels() == 4)
	{
		cv::cvtColor(Frame, Frame, cv::COLOR_BGRA2BGR);
	}

	float inScaleFactor = 1;
	float meanVal = 1;

	  cv::Mat inputBlob = cv::dnn::blobFromImage(  Frame, inScaleFactor, cv::Size(         400,          300), meanVal, false, false);


	net.setInput(inputBlob, "data");
	cv::Mat detection = net.forward("detection_out");

	cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());


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

			cv::rectangle(Frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2, 4);
			dlib::rectangle r(
				static_cast<int>(detectionMat.at<float>(i, 3) * Frame.cols),
				static_cast<int>(detectionMat.at<float>(i, 4) * Frame.rows),
				static_cast<int>(detectionMat.at<float>(i, 5) * Frame.cols),
				static_cast<int>(detectionMat.at<float>(i, 6) * Frame.rows)
			);

			shape = pose_model_shape_predictor(cimg, r);
			shapes.push_back(shape);




			//Render square around face
			//cv::rectangle(Frame, dlibRectangleToOpenCV(r), (1,0,1),5);





			//draw features
			for (unsigned int i = 0; i < 68; ++i)
			{

				cv::circle(Frame, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), 2);
			}


			//ProcessShapeWithKalman(shape);


			//calc Head Info
	//			CalcHeadInfo(shape);

			// Custom Face Render
	//			RenderFace(FinalFrame, shape);

			//fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
			//2D ref points(image coordinates), referenced from detected facial feature
			//std::vector<cv::Point2d> image_pts;
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
			//	//fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
			//image_pts.push_back(cv::Point2d(shape.part(30).x(), shape.part(30).y()));    // Nose tip
			//image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));      // Chin
			//image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y()));    // Left eye left corner
			//image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y()));    // Right eye right corner
			//image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y()));    // Left Mouth corner
			//image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y()));    // Right mouth corner


			double focal_length = Frame.cols;
			camera_matrix = get_camera_matrix(focal_length, cv::Point2d(Frame.cols / 2, Frame.rows / 2));


			dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);



			//calc pose
			//cv::solvePnPRansac(object_compare_pts, image_pts, camera_matrix, dist_coeffs, rotation_vec, translation_vec);
			cv::solvePnP(object_compare_pts, image_pts, camera_matrix, dist_coeffs, rotation_vec, translation_vec);
			//cv::solveP3P(object_compare_pts, image_pts, camera_matrix, dist_coeffs, rotation_vec, translation_vec);


		//Alt1
		//std::vector<cv::Point3d> nose_end_point3D;
		//std::vector<cv::Point2d> nose_end_point2D;
		//nose_end_point3D.push_back(cv::Point3d(0, 0, 1000.0));

		//cv::projectPoints(nose_end_point3D, rotation_vec, translation_vec, camera_matrix, dist_coeffs, nose_end_point2D);
		//Alt1 End

		//Alt2
		//reproject
		//cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, camera_matrix, dist_coeffs, reprojectdst);
		//Alt2 END
		//calc euler angle
			cv::Rodrigues(rotation_vec, rotation_mat);
			cv::hconcat(rotation_mat, translation_vec, pose_mat);
			cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

			image_pts.clear();
			shapes.clear();
			faces.clear();

			HeadRotator.Pitch = -euler_angle.at<double>(0);
			HeadRotator.Yaw = euler_angle.at<double>(1);
			HeadRotator.Roll = -euler_angle.at<double>(2);

			out_translation = rotation_mat.t() * translation_vec;

			HeadLocation.X = out_translation.at<double>(0);
			HeadLocation.Y = out_translation.at<double>(1);
			HeadLocation.Z = out_translation.at<double>(2);

		}

	}
	
}

cv::Mat AWebcamActor::get_camera_matrix(float focal_length, cv::Point2d center)
{
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	return camera_matrix;
}

void AWebcamActor::ProcessShapeWithKalman(const dlib::full_object_detection& shape)
{
	for (int i = 0; i < (int)(shape.num_parts()); i++)
	{
		cv::Mat prediction = KFs[i].predict();
		cv::Point predict_pt = cv::Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));
		measurements[i].at<float>(0) = (float)shape.part(i).x();
		measurements[i].at<float>(1) = (float)shape.part(i).y();
		KFs[i].correct(measurements[i]);
		facePoints[i] = predict_pt;
	}
}

void AWebcamActor::EndPlay(EEndPlayReason::Type reasonType)
{
	if (bCamOpen)
	{
		Webcam->release();
		bCamOpen = false;
	}

	FMemory::Free(Webcam);

	bMemoryReleased = true;
}


