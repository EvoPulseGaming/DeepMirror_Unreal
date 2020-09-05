//#pragma once
//
//#include "CoreMinimal.h"
//#include "GameFramework/Actor.h"
//#include "OpenCV_Common.h"
//#include "RHI.h"
//#include "Math/IntRect.h"
//#include "WebcamActor_libFaceDetect.generated.h"
//
//USTRUCT(BlueprintType)
//struct FFaceDetected
//{
//	GENERATED_BODY()
//	
//	FFaceDetected() {}
//	FFaceDetected(FVector4 faceRect, int32 neighbors, int32 angle) : FaceRect(faceRect), Neighbors(neighbors), Angle(angle) {}
//
//	UPROPERTY(BlueprintReadOnly, Category = "FaceDetection")
//		FVector4 FaceRect;
//
//	UPROPERTY(BlueprintReadOnly, Category = "FaceDetection")
//		int32 Neighbors;
//
//	UPROPERTY(BlueprintReadOnly, Category = "FaceDetection")
//		int32 Angle;
//
//	UPROPERTY(BlueprintReadOnly, Category = "FaceDetection")
//		TArray<FVector2D> LandMarks;
//		
//};
//
//UCLASS()
//class DEEPMIRRORPLUGIN_API AWebcamActor_libFaceDetect : public AActor
//{
//	GENERATED_BODY()
//	
//public:		
//	AWebcamActor_libFaceDetect();
//	~AWebcamActor_libFaceDetect();
//	virtual void Tick(float DeltaTime) override;
//
//	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Webcam")
//		int32 CameraID;
//
//	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
//		FVector2D VideoSize;
//
//	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Webcam")
//		float RefreshFPS;
//
//	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
//		float RefreshTime;
//
//	UPROPERTY(BlueprintReadWrite, Category = "WebcamTexture")
//		UTexture2D* VideoTexture;
//
//	UPROPERTY(BlueprintReadOnly, Category = "WebcamInternals")
//		bool bCamOpen;
//
//	UPROPERTY(BlueprintReadWrite, Category = "WebcamData")
//		TArray<FColor> CameraData;
//
//	UPROPERTY(EditAnywhere, Category = "FaceDetection")
//		bool bFaceDetect;
//
//	UPROPERTY(BlueprintReadOnly, Category = "FaceDetection")
//		TArray<FFaceDetected> DetectedFaces;
//
//	UPROPERTY(EditAnywhere, Category = "FaceDetection")
//		bool bDebugFaceLandmarks;
//
//	UFUNCTION(BlueprintNativeEvent, Category = "Webcam")
//		void OnNextFrame();
//	
//	virtual void EndPlay(EEndPlayReason::Type reasonType) override;
//
//protected:	
//	virtual void BeginPlay() override;
//		
//	cv::Mat* Frame;
//	cv::Mat* GrayFrame;
//	cv::VideoCapture* Webcam;
//	cv::Size* Size;
//
//	unsigned char* pBuffer;
//	int* pResult;
//	
//	FTimerHandle timerHandle;
//	FUpdateTextureRegion2D* VideoUpdateTextureRegion;
//
//	void UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData);
//	void UpdateFrame();
//	void CameraTimerTick();
//	void UpdateTexture();
//
//	void DetectFace();
//	
//private:
//	struct FUpdateTextureRegionsData
//	{
//		FTexture2DResource* Texture2DResource;
//		int32 MipIndex;
//		uint32 NumRegions;
//		FUpdateTextureRegion2D* Regions;
//		uint32 SrcPitch;
//		uint32 SrcBpp;
//		uint8 * SrcData;
//	};
//	
//	bool bMemoryReleased;
//
//};
