// Some copyright should be here...

using System.IO;
using UnrealBuildTool;

public class DeepMirrorPlugin : ModuleRules
{
	public DeepMirrorPlugin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		//for dlib
		//for c4458 c4459
		ShadowVariableWarningLevel = WarningLevel.Off;
		//for c4668  err
		bEnableUndefinedIdentifierWarnings = false;
		bEnableExceptions = true;



		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
                Path.Combine(ModuleDirectory, "Public"),
				Path.Combine(ModuleDirectory, "Classes"),
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
				Path.Combine(ModuleDirectory, "Private"),
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				// ... add other public dependencies that you statically link with here ...
				"Core",
				"RHI",
				"RenderCore",
				"OpenCV",
				"OpenCVHelper",
				"Dlib",
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"OpenCV",
				"OpenCVHelper"
				// ... add private dependencies that you statically link with here ...	
			}
			);

		PrivateIncludePathModuleNames.AddRange(
			new string[] 
			{
				"OpenCV",
				"OpenCVHelper",
				"Dlib",
			});

		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
