// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"
#include "HAL/IConsoleManager.h" // for TAutoConsoleVariable<>

class IConsoleVariable;

/**
 * 
 */
class IDlibHelperModule : public IModuleInterface
{
public:
	/** Virtual destructor. */
	virtual ~IDlibHelperModule() {}
};

