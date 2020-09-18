@ECHO OFF
set dlib_ver=19.21
set dlib_url=http://dlib.net/files/dlib-%dlib_ver%.zip
set dlib_src=dlib-%dlib_ver%

for %%x in ("..\..\..\Binaries\ThirdParty") do set bin_path=%%~fx
for %%x in ("lib") do set lib_path=%%~fx

if not exist build md build

pushd build

IF NOT EXIST %dlib_src% (
    IF NOT EXIST %dlib_src%.zip (
        echo Downloading %dlib_url%...
        powershell -Command "(New-Object Net.WebClient).DownloadFile('%dlib_url%', '%dlib_src%.zip')"
    )
    echo Extracting %dlib_src%.zip...
    powershell -Command "Add-Type -AssemblyName System.IO.Compression.FileSystem; [System.IO.Compression.ZipFile]::ExtractToDirectory('%dlib_src%.zip', '.')"
)


echo Deleting existing build directories...
if exist x86 rd /s /q x86
if exist x64 rd /s /q x64

md x86
pushd x86

echo Configuring x86 build...
if "%1" == "-2019" (
	echo Compiling for 2019
	cmake -G "Visual Studio 16 2019" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 ..\%dlib_src%
	) else (
	echo Compiling for 2017
	cmake -G "Visual Studio 15 2017" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 ..\%dlib_src%
)


echo Building x86 Release build...
cmake.exe --build . --config Release --target INSTALL -- /m:4
echo Building x86 Debug build...
cmake.exe --build . --config Debug --target INSTALL -- /m:4

popd

md x64
pushd x64

echo Configuring x64 build...
if "%1" == "-2019" (
echo Compiling for 2019
cmake -G "Visual Studio 16 2019 Win64" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 ..\%dlib_src%
) else (
echo Compiling for 2017
cmake -G "Visual Studio 15 2017 Win64" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 ..\%dlib_src%
)

echo Building x64 Release build...
cmake.exe --build . --config Release --target INSTALL -- /m:4
echo Building x64 Debug build...
cmake.exe --build . --config Debug --target INSTALL -- /m:4

popd

popd

echo Cleaning up
:: Clean up destination directories
@ECHO ON
REM md %bin_path%\Win32
REM md %bin_path%\Win64

::Copy to plugin dir
md ..\..\..\Libraries\Win32
md ..\..\..\Libraries\Win64


if "%1" == "-2019" (
echo Compiling for 2019
move /y %lib_path%\dlib19.21.0_debug_32bit_msvc1924.lib ..\..\..\Libraries\Win32\dlib19.21.0d.lib
move /y %lib_path%\dlib19.21.0_release_32bit_msvc1924.lib ..\..\..\Libraries\Win32\dlib19.21.0.lib
move /y %lib_path%\dlib19.21.0_debug_64bit_msvc1924.lib ..\..\..\Libraries\Win64\dlib19.21.0d.lib
move /y %lib_path%\dlib19.21.0_release_64bit_msvc1924.lib ..\..\..\Libraries\Win64\dlib19.21.0.lib
) else (
echo Compiling for 2017
move /y %lib_path%\dlib19.21.0_debug_32bit_msvc1916.lib ..\..\..\Libraries\Win32\dlib19.21.0d.lib
move /y %lib_path%\dlib19.21.0_release_32bit_msvc1916.lib ..\..\..\Libraries\Win32\dlib19.21.0.lib
move /y %lib_path%\dlib19.21.0_debug_64bit_msvc1916.lib ..\..\..\Libraries\Win64\dlib19.21.0d.lib
move /y %lib_path%\dlib19.21.0_release_64bit_msvc1916.lib ..\..\..\Libraries\Win64\dlib19.21.0.lib
)

REM rd /s /q x86
REM rd /s /q x64
REM del /f OpenCV*.cmake

echo Done. Libs are in %lib_path%
