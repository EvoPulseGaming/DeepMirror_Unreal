#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "OpenCV_Common.h"
#include "RHI.h"
#include "Math/IntRect.h"
#include "PnPProblem.h"
#include "WebcamActor.generated.h"

USTRUCT()
struct FEstimatedLightData
{
	GENERATED_BODY()

		UPROPERTY()
		TArray<float> LightValue;

	FEstimatedLightData()
	{
	}
};

UCLASS()
class DEEPMIRRORPLUGIN_API AWebcamActor : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AWebcamActor();
	// Called when garbage collected
	~AWebcamActor();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;




	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	// Called when actor destroyed
	virtual void EndPlay(EEndPlayReason::Type reasonType) override;

	// The device ID opened by the Video Stream
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = Webcam)
		int32 CameraID;

	// The rate at which the color data array and video texture is updated (in frames per second)
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Webcam")
		float RefreshFPS;

	// The refresh timer for updating stream frame
	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
		float RefreshTime;


	// OpenCV fields
	cv::Mat frame;
	cv::Mat gray;
	cv::Mat FrameToLightSize;
	cv::Mat LightEstimationFrame;
	cv::Mat smallMat;
	cv::VideoCapture stream;

	// OpenCV prototypes
	void UpdateFrame();
	void CameraTimerTick();
	void UpdateTexture();

	UFUNCTION(BlueprintCallable, Category = "Dlib|DetectFacePose")
		void ComputeHeadDataTick();

	// If the stream has succesfully opened yet
	UPROPERTY(BlueprintReadOnly, Category = Webcam)
		bool isStreamOpen;

	//Custom timer to handle updating the stream
	FTimerHandle timerHandle;

	// The videos width and height (width, height)
	UPROPERTY(BlueprintReadWrite, Category = "Webcam")
		FVector2D VideoSize;

	// The current video frame's corresponding texture
	UPROPERTY(BlueprintReadWrite, Category = "WebcamTexture")
		UTexture2D* VideoTexture;

	// The current video frame's corresponding texture
	UPROPERTY(BlueprintReadWrite, Category = "WebcamTexture")
		UTexture2D* LightEstVideoTexture;

	// The current color data array
	UPROPERTY(BlueprintReadWrite, Category = "WebcamData")
		TArray<FColor> CameraData;

	// The current grayscale data array
	UPROPERTY(BlueprintReadWrite, Category = "EstimatedLight")
		TArray<uint8> EstimatedLights;

	// The targeted resize width and height (width, height) for light estimation
	UPROPERTY(EditAnywhere, Category = "EstimatedLight")
		float xSizeScale = .01875;

	UPROPERTY(EditAnywhere, Category = "EstimatedLight")
		float ySizeScale = .025;

	//The width and height of light estimation texture
	UPROPERTY(BlueprintReadOnly, Category = "EstimatedLight")
		int xSizeLight = 0;

	UPROPERTY(BlueprintReadOnly, Category = "EstimatedLight")
		int ySizeLight = 0;

	//Draw debug markers on top of the stream
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
		bool bDebugFaceLandmarks;

	//relative rotation of detected face
	UPROPERTY(BlueprintReadOnly)
		FRotator HeadRotator;

	UPROPERTY(BlueprintReadOnly)
	FQuat QuatHeadRotation;

	//relative location of detected face
	UPROPERTY(BlueprintReadOnly)
		FVector HeadLocation;

	//How many frames to skip before attempting another detection
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
		int SKIP_FRAMES = 2;

	//How small to make the video for face detection (smaller = faster, less accurate)
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
		int FACE_DOWNSAMPLE_RATIO = 2;

	int run_count = 0;

	cv::Point2f prev_nose;



	//file reader for dnn network
	std::ifstream StreamIn;
	//for dnn face detection
	cv::Mat inputBlob;

	//face detection
	dlib::shape_predictor landmark_detector_shape_predictor;
	dlib::rectangle face_rect;
	std::vector<dlib::full_object_detection> shapes;
	dlib::full_object_detection shape;


	cv::Mat detection;
	cv::Mat detectionMat;





	//fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
	std::vector<cv::Point3f> model_3D_compare_pts;

	//2D ref points(image coordinates), referenced from detected facial feature
	//std::vector<cv::Point2f> image_landmarks_pts;
	//reprojected 2D points
	std::vector<cv::Point2f> reprojectdst;
	//reproject 3D points world coordinate axis to verify result pose
	std::vector<cv::Point3f> reprojectsrc;

	cv::dnn::Net box_detector_face_net;



	cv::Mat dist_coeffs;
	cv::Mat rotation_vector; // Rotation in axis-angle form
	cv::Mat translation_vector;
	std::vector<cv::Point3f> nose_end_point3D;
	std::vector<cv::Point2f> nose_end_point2D;


	cv::Mat rotation_mat;//3 x 3 R



	//result
	cv::Mat pose_mat;							     //3 x 4 R | T
	cv::Mat euler_angle;

	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics;
	cv::Mat out_rotation;
	cv::Mat out_translation;

	bool bMemoryReleased;

	//kalman filter
	double deltaT = 0.01, omega_w = 8, omega_u = 3.1623;
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;

	cv::Mat measurements;







	void fillMeasurements(cv::Mat & measurements, const cv::Mat & translation_measured, const cv::Mat & rotation_measured);

	//RotationOrders = [ 0'XYZ', 1'YZX', 2'ZXY', 3'XZY', 4'YXZ', 5'ZYX' ];
	UPROPERTY(EditAnywhere, Category = "FaceDetection")
		int rot2eular_Order = 3;

	std::vector<double>  rot2euler(const cv::Mat & rotationMatrix);

	cv::Mat euler2rot(const cv::Mat & euler);

	void updateKalmanFilter(cv::KalmanFilter & KF, cv::Mat & measurement, cv::Mat & translation_estimated, cv::Mat & rotation_estimated);
	

	void draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2f>& list_points2d);

	void drawArrow(cv::Mat image, cv::Point2i p, cv::Point2i q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift);

	void initKalmanFilter(cv::KalmanFilter & KF, int nStates, int nMeasurements, int nInputs, double dt);



	PnPProblem pnp_detection;
	PnPProblem pnp_detection_est;


protected:

	// Use this function to update the texture rects you want to change:
	// NOTE: There is a method called UpdateTextureRegions in UTexture2D but it is compiled WITH_EDITOR and is not marked as ENGINE_API so it cannot be linked
	// from plugins.
	// FROM: https://wiki.unrealengine.com/Dynamic_Textures
	void UpdateTextureRegions(UTexture2D* Texture, int32 MipIndex, uint32 NumRegions, FUpdateTextureRegion2D* Regions, uint32 SrcPitch, uint32 SrcBpp, uint8* SrcData, bool bFreeData);

	// Pointer to update texture region 2D struct
	FUpdateTextureRegion2D* VideoUpdateTextureRegion;
	FUpdateTextureRegion2D* LightEstVideoUpdateTextureRegion;
};
