// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#include "Dlib.h"
#include "Core.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"


#define LOCTEXT_NAMESPACE "FDlibModule"

void FDlibModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module

	// Get the base directory of this plugin
	
}

void FDlibModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.

	// Free the dll handle

}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FDlibModule, Dlib)
