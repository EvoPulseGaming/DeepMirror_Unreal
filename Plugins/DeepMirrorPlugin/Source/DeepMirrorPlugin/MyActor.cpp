//// Fill out your copyright notice in the Description page of Project Settings.
//
//
//#include "MyActor.h"
//
//#include<opencv2/core.hpp>
//#include<opencv2/highgui.hpp>
//
//
//// Sets default values
//AMyActor::AMyActor()
//{
// 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
//	PrimaryActorTick.bCanEverTick = true;
//}
//
//// Called when the game starts or when spawned
//void AMyActor::BeginPlay()
//{
//	Super::BeginPlay();
//	//cv::Mat gImage = cv::Mat(512, 512, CV_8UC4);
//}
//
//// Called every frame
//void AMyActor::Tick(float DeltaTime)
//{
//	cv::Mat frame(512, 512, CV_8UC4);
//	frame.setTo(cv::Scalar(255, 0, 0, 255));
//
//	cv::imshow("haha", frame);
//
//	Super::Tick(DeltaTime);
//}
//
