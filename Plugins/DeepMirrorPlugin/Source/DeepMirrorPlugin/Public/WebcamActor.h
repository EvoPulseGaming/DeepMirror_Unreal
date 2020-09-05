#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "OpenCV_Common.h"
#include "RHI.h"
#include "Math/IntRect.h"
#include "WebcamActor.generated.h"


UCLASS()
class DEEPMIRRORPLUGIN_API AWebcamActor : public AActor
{
	GENERATED_BODY()

public:
	AWebcamActor();
	~AWebcamActor();
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Webcam")
		int32 CameraID;

	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
		FVector2D VideoSize;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Webcam")
		float RefreshFPS;

	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
		float RefreshTime;

	UPROPERTY(BlueprintReadWrite, Category = "WebcamTexture")
		UTexture2D* VideoTexture;

	UPROPERTY(BlueprintReadOnly, Category = "WebcamInternals")
		bool bCamOpen;

	UPROPERTY(BlueprintReadWrite, Category = "WebcamData")
		TArray<FColor> CameraData;

	UPROPERTY(EditAnywhere, Category = "FaceDetection")
		bool bDebugFaceLandmarks;

	UPROPERTY(BlueprintReadOnly)
		FRotator HeadRotator;

	UPROPERTY(BlueprintReadOnly)
		FVector HeadLocation;

	UPROPERTY(EditAnywhere, Category = "FaceDetection")
	int SKIP_FRAMES = 2;
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
	int FACE_DOWNSAMPLE_RATIO = 4;
	//1.kalman filter setup  
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
	int stateNum = 4;
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
	int measureNum = 2;

	UFUNCTION(BlueprintCallable, Category = "Dlib|DetectFacePose")
		void ComputeHeadDataTick();

	virtual void EndPlay(EEndPlayReason::Type reasonType) override;

protected:
	virtual void BeginPlay() override;

	//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
	//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
	double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
	double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };
	//fill in cam intrinsics and distortion coefficients
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, K);

	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);


	int UpdateCount;

	cv::Mat Frame;
	cv::Mat* GrayFrame;
	cv::VideoCapture* Webcam;
	cv::Size* Size;
	dlib::frontal_face_detector detector;
	dlib::shape_predictor pose_model_shape_predictor;
	//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	std::vector<cv::Point3d> object_compare_pts;

	//2D ref points(image coordinates), referenced from detected facial feature
	std::vector<cv::Point2d> image_pts;
	//reprojected 2D points
	std::vector<cv::Point2d> reprojectdst;
	//reproject 3D points world coordinate axis to verify result pose
	std::vector<cv::Point3d> reprojectsrc;

	std::vector<dlib::full_object_detection> shapes;
	dlib::full_object_detection shape;
	std::vector<dlib::rectangle> faces;

	cv::Mat rotation_vec; //3 x 1
	cv::Mat rotation_mat;//3 x 3 R
	cv::Mat translation_vec;//3 x 1 T

	//cv::Mat temp;
	cv::Mat smallMat;

	//result
	cv::Mat pose_mat;							     //3 x 4 R | T
	cv::Mat euler_angle;

	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics;
	cv::Mat out_rotation;
	cv::Mat out_translation;

	//Filter
	cv::KalmanFilter KFs[68];
	cv::Mat measurements[68];
	cv::Point facePoints[68];


	FTimerHandle timerHandle;
	FUpdateTextureRegion2D* VideoUpdateTextureRegion;

	void UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData);
	void UpdateFrame();
	void CameraTimerTick();
	void UpdateTexture();

	cv::Mat get_camera_matrix(float focal_length, cv::Point2d center);

	void ProcessShapeWithKalman(const dlib::full_object_detection& shape);
private:
	struct FUpdateTextureRegionsData
	{
		FTexture2DResource* Texture2DResource;
		int32 MipIndex;
		uint32 NumRegions;
		FUpdateTextureRegion2D* Regions;
		uint32 SrcPitch;
		uint32 SrcBpp;
		uint8 * SrcData;
	};

	bool bMemoryReleased;

};
