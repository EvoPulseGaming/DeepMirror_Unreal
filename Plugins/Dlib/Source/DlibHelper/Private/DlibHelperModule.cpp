// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "IDlibHelperModule.h"
#include "Modules/ModuleManager.h" // for IMPLEMENT_MODULE()
#include "Interfaces/IPluginManager.h"
#include "HAL/PlatformProcess.h"

class FDlibHelperModule : public IDlibHelperModule
{
public:
	FDlibHelperModule();

public:
	//~ IModuleInterface interface
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	void* DlibDllHandle;
};

FDlibHelperModule::FDlibHelperModule()
	: DlibDllHandle(nullptr)
{}

void FDlibHelperModule::StartupModule()
{
	const FString PluginDir = IPluginManager::Get().FindPlugin(TEXT("Dlib"))->GetBaseDir();

#if WITH_DLIB
	const FString DlibBinPath = PluginDir / TEXT(PREPROCESSOR_TO_STRING(DLIB_PLATFORM_PATH));
	const FString DLLPath = DlibBinPath / TEXT(PREPROCESSOR_TO_STRING(DLIB_DLL_NAME));

	FPlatformProcess::PushDllDirectory(*DlibBinPath);
	DlibDllHandle = FPlatformProcess::GetDllHandle(*DLLPath);
	FPlatformProcess::PopDllDirectory(*DlibBinPath);
#endif
}

void FDlibHelperModule::ShutdownModule()
{
#if WITH_DLIB
	if (DlibDllHandle)
	{
		FPlatformProcess::FreeDllHandle(DlibDllHandle);
		DlibDllHandle = nullptr;
	}
#endif
}

IMPLEMENT_MODULE(FDlibHelperModule, DlibHelper);
