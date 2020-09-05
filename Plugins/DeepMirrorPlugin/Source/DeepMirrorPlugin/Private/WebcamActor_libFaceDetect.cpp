//#include "WebcamActor_libFaceDetect.h"
//#include "TimerManager.h"
//#include "Engine/Texture2D.h"
//#include "facedetect-dll.h"
//
//#define DETECT_BUFFER_SIZE 0x20000
//
//AWebcamActor_libFaceDetect::AWebcamActor_libFaceDetect()
//{
//	PrimaryActorTick.bCanEverTick = true;
//
//	CameraID = 0;
//	RefreshFPS = 15.f;
//	RefreshTime = 1000000.0f;
//	bFaceDetect = true;
//	bDebugFaceLandmarks = true;
//	VideoSize = FVector2D(0, 0);
//	Frame = new cv::Mat();
//	GrayFrame = new cv::Mat();
//	Webcam = new cv::VideoCapture();
//		
//}
//
//AWebcamActor_libFaceDetect::~AWebcamActor_libFaceDetect()
//{
//	if (!bMemoryReleased) 
//	{
//		FMemory::Free(Frame);
//		FMemory::Free(GrayFrame);
//		FMemory::Free(Size);
//		FMemory::Free(Webcam);
//
//		if (bFaceDetect)
//		{
//			FMemory::Free(pBuffer);
//		}
//	}	
//}
//
//void AWebcamActor_libFaceDetect::BeginPlay()
//{
//	Super::BeginPlay();	
//
//	bMemoryReleased = false;
//
//	if (bFaceDetect)
//	{
//		pResult = NULL;
//		pBuffer = (unsigned char*)FMemory::Malloc(DETECT_BUFFER_SIZE);
//	}
//
//	Webcam->open(CameraID);
//	if (Webcam->isOpened())
//	{
//		bCamOpen = true;
//		UpdateFrame();
//		VideoSize = FVector2D(Frame->cols, Frame->rows);
//		Size = new cv::Size(VideoSize.X, VideoSize.Y);
//		VideoTexture = UTexture2D::CreateTransient(VideoSize.X, VideoSize.Y);
//		VideoTexture->UpdateResource();
//		VideoUpdateTextureRegion = new FUpdateTextureRegion2D(0, 0, 0, 0, VideoSize.X, VideoSize.Y);
//
//		CameraData.Init(FColor(0, 0, 0, 255), VideoSize.X * VideoSize.Y);
//		
//	}
//	else 
//	{
//		UE_LOG(LogTemp, Error, TEXT("A CAMERA NAO ABRIU"))
//	}
//
//	GetWorldTimerManager().SetTimer(timerHandle, this, &AWebcamActor_libFaceDetect::CameraTimerTick, 0.08f, true);
//}
//
//void AWebcamActor_libFaceDetect::Tick(float DeltaTime)
//{
//	Super::Tick(DeltaTime);
//}
//
//void AWebcamActor_libFaceDetect::CameraTimerTick()
//{
//	RefreshTime += 0.08f;
//	if (bCamOpen && RefreshTime >= 1.f / RefreshFPS)
//	{
//		RefreshTime -= 1.f / RefreshFPS;
//		UpdateFrame();
//		UpdateTexture();
//		OnNextFrame();
//	}
//}
//
//void AWebcamActor_libFaceDetect::UpdateFrame()
//{
//	if (bCamOpen)
//	{
//		Webcam->read(*Frame);
//		//TODO: eu posso manipular o fame aqui		
//		cv::cvtColor(*Frame, *GrayFrame, cv::COLOR_BGR2GRAY);
//		if (bFaceDetect) 
//		{
//			DetectFace();
//		}
//	}
//}
//
//void AWebcamActor_libFaceDetect::UpdateTexture()
//{
//	if (bCamOpen && Frame->data)
//	{
//		for (int y = 0; y < VideoSize.Y; y++)
//		{
//			for (int x = 0; x < VideoSize.X; x++)
//			{
//				int i = x + (y * VideoSize.X);
//				CameraData[i].B = Frame->data[i * 3 + 0];
//				CameraData[i].G = Frame->data[i * 3 + 1];
//				CameraData[i].R = Frame->data[i * 3 + 2];
//			}
//		}
//
//		UpdateTextureRegions(VideoTexture, (int32)0, (uint32)1, VideoUpdateTextureRegion, 
//			(uint32)(4 * VideoSize.X), (uint32)4, (uint8*)CameraData.GetData(), false);
//	}
//}
//
//void AWebcamActor_libFaceDetect::UpdateTextureRegions(UTexture2D * Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D * Regions, 
//	uint32 SrcPitch, uint32 SrcBpp, uint8 * SrcData, bool bFreeData)
//{
//	if (Texture->Resource)
//	{		
//		FUpdateTextureRegionsData* RegionData = new FUpdateTextureRegionsData;
//		
//		RegionData->Texture2DResource = (FTexture2DResource*)Texture->Resource;
//		RegionData->MipIndex = MipIndex;
//		RegionData->NumRegions = NumRegions;
//		RegionData->Regions = Regions;
//		RegionData->SrcPitch = SrcPitch;
//		RegionData->SrcBpp = SrcBpp;
//		RegionData->SrcData = SrcData;
//				
//		ENQUEUE_UNIQUE_RENDER_COMMAND_TWOPARAMETER(UpdateTextureRegionsData, FUpdateTextureRegionsData*,
//			RegionData, RegionData,
//			bool, bFreeData, bFreeData,
//			{
//
//				for (uint32 regionIndex = 0; regionIndex < RegionData->NumRegions; ++regionIndex)
//				{
//					int32 currentFirstMip = RegionData->Texture2DResource->GetCurrentFirstMip();
//					if (RegionData->MipIndex >= currentFirstMip)
//					{
//						RHIUpdateTexture2D(
//							RegionData->Texture2DResource->GetTexture2DRHI(),
//							RegionData->MipIndex - currentFirstMip,
//							RegionData->Regions[regionIndex],
//							RegionData->SrcPitch,
//							RegionData->SrcData + RegionData->Regions[regionIndex].SrcY * RegionData->SrcPitch
//							+ RegionData->Regions[regionIndex].SrcX * RegionData->SrcBpp
//						);
//					}
//				}
//
//			}
//			if (bFreeData)
//			{
//				FMemory::Free(RegionData->Regions);
//				FMemory::Free(RegionData->SrcData);
//			}
//			delete RegionData;
//		);
//		
//	}
//}
//
//void AWebcamActor_libFaceDetect::DetectFace()
//{
//	pResult = facedetect_multiview_reinforce(pBuffer, (unsigned char*)(GrayFrame->ptr(0)), GrayFrame->cols, GrayFrame->rows, (int)GrayFrame->step,
//		1.2f, 2, 48, 0, 1);
//
//	DetectedFaces.Empty();
//	
//	for (int i = 0; i < (pResult ? *pResult : 0); i++)
//	{
//		short* p = ((short*)(pResult + 1)) + 142 * i;
//		int x = p[0];
//		int y = p[1];
//		int w = p[2];
//		int h = p[3];
//		int neighbors = p[4];
//		int angle = p[5];
//						
//		FFaceDetected face = FFaceDetected(FVector4(x, y, w, h), neighbors, angle);
//		TArray<FVector2D> landMarks;
//
//		for (int j = 0; j < 68; j++)
//		{			
//			landMarks.Add(FVector2D(p[6 + 2 * j], p[6 + 2 * j + 1]));
//		
//			if (bDebugFaceLandmarks) 
//			{
//				cv::circle(*Frame, cv::Point((int)p[6 + 2 * j], (int)p[6 + 2 * j + 1]), 1, cv::Scalar(255, 0, 0));
//			}
//		}
//		face.LandMarks = landMarks;
//
//		if (bDebugFaceLandmarks)
//		{
//			cv::rectangle(*Frame, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 3);
//		}
//
//		DetectedFaces.Add(face);
//						
//	}
//					
//}
//
//void AWebcamActor_libFaceDetect::OnNextFrame_Implementation()
//{	
//}
//
//void AWebcamActor_libFaceDetect::EndPlay(EEndPlayReason::Type reasonType)
//{	
//	if (bCamOpen) 
//	{	
//		Webcam->release();
//		bCamOpen = false;
//	}
//
//	FMemory::Free(Frame);
//	FMemory::Free(GrayFrame);
//	FMemory::Free(Webcam);
//	FMemory::Free(pBuffer);
//		
//	bMemoryReleased = true;
//}
//
//
