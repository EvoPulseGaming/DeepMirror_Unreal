// Some copyright should be here...

using System.IO;
using UnrealBuildTool;

public class DeepMirrorPlugin : ModuleRules
{
	private string LibFacePath
	{
		get
		{
			return Path.Combine(ModuleDirectory, "../libfacedetection");
		}
	}

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
				Path.Combine(ModuleDirectory, "../libfacedetection/include"),
			}
			);



		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
				Path.Combine(ModuleDirectory, "Private"),
				Path.Combine(ModuleDirectory, "../libfacedetection/include"),
				//Path.Combine(ModuleDirectory, "../Dlib"),
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
				"DlibHelper",
				"Projects", //to get the root dir of this plugin later in code
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
				"OpenCVHelper",
				"Dlib",
				"DlibHelper",
				// ... add private dependencies that you statically link with here ...	
			}
			);

		PrivateIncludePathModuleNames.AddRange(
			new string[] 
			{
				"OpenCV",
				"OpenCVHelper",
				"Dlib",
				"DlibHelper",
			});

		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);

		PublicIncludePaths.AddRange(
	new string[] {
					Path.Combine(LibFacePath, "include")
		}
	);

		//libfacedetection
		PublicLibraryPaths.Add(Path.Combine(LibFacePath, "lib", "Win64"));
		PublicAdditionalLibraries.Add(Path.Combine(LibFacePath, "lib", "Win64", "libfacedetect-x64.lib"));
		PublicDelayLoadDLLs.Add(Path.Combine(LibFacePath, "lib", "Win64", "libfacedetect-x64.dll"));
		PublicAdditionalLibraries.Add(Path.Combine(LibFacePath, "lib", "Win64", "libfacedetectcnn-x64.lib"));
		PublicDelayLoadDLLs.Add(Path.Combine(LibFacePath, "lib", "Win64", "libfacedetectcnn-x64.dll"));

		//DLIB_USE_CUDA
		//DLIB_USE_BLAS
	}
}
