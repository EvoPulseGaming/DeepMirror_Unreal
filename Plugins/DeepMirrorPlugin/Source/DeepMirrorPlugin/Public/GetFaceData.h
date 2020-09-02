// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "VideoCapture.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "OpenCV_Common.h"
#include "FaceFrameCalcThread.h"

//dlib
#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing.h" 

#include "GetFaceData.generated.h"

UCLASS()
class DEEPMIRRORPLUGIN_API AGetFaceData : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AGetFaceData();

	UPROPERTY(BlueprintReadOnly)
		float LeftEyeWeight;
	UPROPERTY(BlueprintReadOnly)
		float RightEyeWeight;
	UPROPERTY(BlueprintReadOnly)
		float MouseWeight;

	UPROPERTY(BlueprintReadOnly)
		FQuat HeadQuat;

	UPROPERTY(BlueprintReadOnly)
		FRotator HeadRotator;

	int UpdateCount;
	int FrameRate;
	int OptimizeScale;

	//1.kalman filter setup  
	const int stateNum = 4;
	const int measureNum = 2;

	cv::Mat* Frame;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// OpenCV fields
	UPROPERTY(BlueprintReadWrite, Category = "Dlib|DetectFacePose")
		UCVUMat* frame;

	cv::Mat FinalFrame;

	dlib::frontal_face_detector FrontDector;
	dlib::shape_predictor PoseModel;
	std::vector<dlib::rectangle> faces;


	void RenderFace(cv::Mat& img, const dlib::full_object_detection& shape);
	void ProcessShapeWithKalman(const dlib::full_object_detection& shape);
	void DrawPolyLine(cv::Mat &img, const dlib::full_object_detection& d, const int start, const int end, bool isClosed = false);
	void DrawCircle(cv::Mat &img, const dlib::full_object_detection& d, const float radius, cv::Scalar color);

	void CalcFaceFeature(const dlib::full_object_detection& shape);
	void CalcHeadInfo(const dlib::full_object_detection& shape);



	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = Dlib)
		int FaceCheckRate;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = Dlib)
		int OptimizeCount;



	UFUNCTION(BlueprintCallable, Category = "Dlib|DetectFacePose")
		void ComputeHeadDataTick(UCVUMat* in_frame);

	UFUNCTION(BlueprintCallable, Category = "Dlib|DetectFacePose")
		void Start();

	UFUNCTION(BlueprintCallable, Category = "Dlib|DetectFacePose")
		void Stop();
private:
	void InitDlib();

private:
	//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
	//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
	double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
	double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };
	//fill in cam intrinsics and distortion coefficients
	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);
	std::vector<cv::Point3d> object_pts;
	std::vector<cv::Point2d> image_pts;
	//result
	cv::Mat rotation_vec;                           //3 x 1
	cv::Mat rotation_mat;                           //3 x 3 R
	cv::Mat translation_vec;                        //3 x 1 T
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

	//reproject 3D points world coordinate axis to verify result pose
	std::vector<cv::Point3d> reprojectsrc;
	//reprojected 2D points
	std::vector<cv::Point2d> reprojectdst;
	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

	//Filter
	cv::KalmanFilter KFs[68];
	cv::Mat measurements[68];
	cv::Point facePoints[68];
};
