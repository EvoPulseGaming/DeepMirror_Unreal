#include "WebcamActor.h"
#include "TimerManager.h"
#include "Engine/Texture2D.h"
#include "OpenCV_Common.h"

#define DETECT_BUFFER_SIZE 0x20000

AWebcamActor::AWebcamActor()
{
	PrimaryActorTick.bCanEverTick = true;

	CameraID = 0;
	RefreshFPS = 15.f;
	RefreshTime = 1000000.0f;
	bFaceDetect = true;
	bDebugFaceLandmarks = true;
	VideoSize = FVector2D(0, 0);
	Frame = new cv::Mat();
	GrayFrame = new cv::Mat();
	Webcam = new cv::VideoCapture();

}

AWebcamActor::~AWebcamActor()
{
	if (!bMemoryReleased)
	{
		FMemory::Free(Frame);
		FMemory::Free(GrayFrame);
		FMemory::Free(Size);
		FMemory::Free(Webcam);

		if (bFaceDetect)
		{
			FMemory::Free(pBuffer);
		}
	}
}

void AWebcamActor::BeginPlay()
{
	Super::BeginPlay();

	bMemoryReleased = false;

	if (bFaceDetect)
	{
		pResult = NULL;
		pBuffer = (unsigned char*)FMemory::Malloc(DETECT_BUFFER_SIZE);
	}

	Webcam->open(CameraID);
	if (Webcam->isOpened())
	{
		bCamOpen = true;
		UpdateFrame();
		VideoSize = FVector2D(Frame->cols, Frame->rows);
		Size = new cv::Size(VideoSize.X, VideoSize.Y);
		VideoTexture = UTexture2D::CreateTransient(VideoSize.X, VideoSize.Y);
		VideoTexture->UpdateResource();
		VideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, VideoSize.X, VideoSize.Y);

		CameraData.Init(FColor(0, 0, 0, 255), VideoSize.X * VideoSize.Y);










		UpdateCount = 0;


		//Load face detection and pose estimation models (dlib).
		detector = dlib::get_frontal_face_detector();

		//dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;


		FString ShapePath = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("TestRes/shape_predictor_68_face_landmarks.dat"));

		std::ifstream StreamIn(TCHAR_TO_UTF8(*ShapePath), std::ios::binary);

		dlib::deserialize(predictor, StreamIn);





		//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
		object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
		object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
		object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
		object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
		object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
		object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
		object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
		object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
		object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
		object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
		object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
		object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
		object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
		object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner


		//result
		cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
		cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

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
		cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
		cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
		cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

		//text on screen
		std::ostringstream outtext;



		//Init kalman
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
		UE_LOG(LogTemp, Error, TEXT("A CAMERA NAO ABRIU"))
	}

	GetWorldTimerManager().SetTimer(timerHandle, this, &AWebcamActor::CameraTimerTick, 0.08f, true);

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
		OnNextFrame();
	}
}

void AWebcamActor::UpdateFrame()
{
	if (bCamOpen)
	{
		Webcam->read(*Frame);
		//TODO: eu posso manipular o fame aqui		
		cv::cvtColor(*Frame, *GrayFrame, cv::COLOR_BGR2GRAY);
		if (bFaceDetect)
		{
			DetectFace();
		}
	}
}

void AWebcamActor::UpdateTexture()
{
	if (bCamOpen && Frame->data)
	{
		for (int y = 0; y < VideoSize.Y; y++)
		{
			for (int x = 0; x < VideoSize.X; x++)
			{
				int i = x + (y * VideoSize.X);
				CameraData[i].B = Frame->data[i * 3 + 0];
				CameraData[i].G = Frame->data[i * 3 + 1];
				CameraData[i].R = Frame->data[i * 3 + 2];
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

void AWebcamActor::ComputeHeadDataTick()
{

	// Grab a frame
	cv::Mat temp;
	*Webcam >> temp;







	std::vector<dlib::rectangle> faces;

	//detector = dlib::get_frontal_face_detector();

	if (temp.empty())
		return;

	//FinalFrame = Frame->m.getMat(cv::ACCESS_READ).clone();

	int FrameRate = 1;
	int OptimizeScale = 2;
	cv::Mat smallMat;
	cv::resize(temp, smallMat, cv::Size(), 1.0 / OptimizeScale, 1.0 / OptimizeScale);
	dlib::cv_image<dlib::bgr_pixel> cimg(temp);
	dlib::cv_image<dlib::bgr_pixel> cimg_small(smallMat);
	//Detect faces   
	if (UpdateCount++ % FrameRate == 0) {
		faces = detector(cimg_small);
		if (faces.size() == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("Can't dector face from cimg_small"));
			return;
		}
	}
	else
	{
		return;
	}










//	dlib::cv_image<dlib::bgr_pixel> cimg(temp);

	// Detect faces
//	std::vector<dlib::rectangle> faces = detector(cimg);

	// Find the pose of each face
	if (faces.size() > 0)
	{
		for (unsigned long i = 0; i < faces.size(); ++i) {
			dlib::rectangle r(
				(long)(faces[i].left()*OptimizeScale),
				(long)(faces[i].top()*OptimizeScale),
				(long)(faces[i].right()*OptimizeScale),
				(long)(faces[i].bottom()*OptimizeScale)
			);

			dlib::full_object_detection shape = predictor(cimg, r);


			//track features
			//dlib::full_object_detection shape = predictor(cimg, faces[0]);

			//draw features
			//for (unsigned int i = 0; i < 68; ++i)
			//{
			//	cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
			//}


			//ProcessShapeWithKalman(shape);


			//calc Head Info
//			CalcHeadInfo(shape);

			// Custom Face Render
//			RenderFace(FinalFrame, shape);

			//fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
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
			//image_pts.push_back(facePoints[17]); //#17 left brow left corner
			//image_pts.push_back(facePoints[21]); //#21 left brow right corner
			//image_pts.push_back(facePoints[22]); //#22 right brow left corner
			//image_pts.push_back(facePoints[26]); //#26 right brow right corner
			//image_pts.push_back(facePoints[36]); //#36 left eye left corner
			//image_pts.push_back(facePoints[39]); //#39 left eye right corner
			//image_pts.push_back(facePoints[42]); //#42 right eye left corner
			//image_pts.push_back(facePoints[45]); //#45 right eye right corner
			//image_pts.push_back(facePoints[31]); //#31 nose left corner
			//image_pts.push_back(facePoints[35]); //#35 nose right corner
			//image_pts.push_back(facePoints[48]); //#48 mouth left corner
			//image_pts.push_back(facePoints[54]); //#54 mouth right corner
			//image_pts.push_back(facePoints[57]); //#57 mouth central bottom corner
			//image_pts.push_back(facePoints[8]);   //#8 chin corner

			//calc pose
			cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);

			//reproject
			cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);


			//calc euler angle
			cv::Rodrigues(rotation_vec, rotation_mat);
			cv::hconcat(rotation_mat, translation_vec, pose_mat);
			cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

			image_pts.clear();

			HeadRotator.Pitch = -euler_angle.at<double>(0);
			HeadRotator.Yaw = euler_angle.at<double>(1);
			HeadRotator.Roll = -euler_angle.at<double>(2);

			HeadLocation.X = -out_translation.at<double>(0);
			HeadLocation.Y = out_translation.at<double>(1);
			HeadLocation.Z = -out_translation.at<double>(2);

		}

	}

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

void AWebcamActor::DetectFace()
{

}

void AWebcamActor::OnNextFrame_Implementation()
{
}

void AWebcamActor::EndPlay(EEndPlayReason::Type reasonType)
{
	if (bCamOpen)
	{
		Webcam->release();
		bCamOpen = false;
	}

	FMemory::Free(Frame);
	FMemory::Free(GrayFrame);
	FMemory::Free(Webcam);
	FMemory::Free(pBuffer);

	bMemoryReleased = true;
}


