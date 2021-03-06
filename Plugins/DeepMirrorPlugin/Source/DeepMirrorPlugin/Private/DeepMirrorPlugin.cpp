// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.
#pragma once
#include "DeepMirrorPlugin.h"

#include "CoreMinimal.h"

#define LOCTEXT_NAMESPACE "FDeepMirrorPluginModule"
DEFINE_LOG_CATEGORY(DeepMirrorPlugin);

void FDeepMirrorPluginModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	
}

void FDeepMirrorPluginModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FDeepMirrorPluginModule, DeepMirrorPlugin)