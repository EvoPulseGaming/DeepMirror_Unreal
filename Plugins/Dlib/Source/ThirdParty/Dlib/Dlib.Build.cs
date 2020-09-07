// Copyright 1998-2019 Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class Dlib : ModuleRules
{
	public Dlib(ReadOnlyTargetRules Target) : base(Target)
	{
		//PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		//solve cannot use ‘throw’ with exceptions disabled
		bEnableExceptions = true;
		
		ShadowVariableWarningLevel = WarningLevel.Off;
		
		Type = ModuleType.External;

		if (Target.Platform == UnrealTargetPlatform.Win64 ||
			Target.Platform == UnrealTargetPlatform.Win32)
		{
			string PlatformDir = Target.Platform.ToString();
			string IncPath = Path.Combine(ModuleDirectory, "include");
			PublicSystemIncludePaths.Add(IncPath);

			string LibPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../Libraries", PlatformDir));

			string LibName = "dlib19.21.0";

			if (Target.Configuration == UnrealTargetConfiguration.Debug &&
				Target.bDebugBuildsActuallyUseDebugCRT)
			{
					LibName += "d";
			}

			PublicAdditionalLibraries.Add(Path.Combine(LibPath, LibName + ".lib"));

		PublicDefinitions.Add("DLIB_NO_GUI_SUPPORT");
		PublicDefinitions.Add("USE_AVX_INSTRUCTIONS");
		PublicDefinitions.Add("USE_SSE4_INSTRUCTIONS");
		
		PublicDefinitions.Add("WITH_DLIB=1");
		PublicDefinitions.Add("DLIB_PLATFORM_PATH=Binaries/ThirdParty/" + PlatformDir);

		}
		else // unsupported platform
		{
            PublicDefinitions.Add("WITH_DLIB=0");
		}

	}
}
