@ECHO OFF
set opencv_ver=4.4.0

set opencv_url=https://github.com/opencv/opencv/archive/%opencv_ver%.zip
set opencv_src=opencv-%opencv_ver%

set opencv_contrib_url=https://github.com/opencv/opencv_contrib/archive/%opencv_ver%.zip
set opencv_contrib_src=opencv_contrib-%opencv_ver%

for %%x in ("..\..\..\Binaries\ThirdParty") do set bin_path=%%~fx
for %%x in ("lib") do set lib_path=%%~fx

if not exist build md build

pushd build

IF NOT EXIST %opencv_src% (
    IF NOT EXIST %opencv_src%.zip (
        echo Downloading %opencv_url%...
        powershell -Command "(New-Object Net.WebClient).DownloadFile('%opencv_url%', '%opencv_src%.zip')"
    )
    echo Extracting %opencv_src%.zip...
    powershell -Command "Add-Type -AssemblyName System.IO.Compression.FileSystem; [System.IO.Compression.ZipFile]::ExtractToDirectory('%opencv_src%.zip', '.')"
)

IF NOT EXIST %opencv_contrib_src% (
    IF NOT EXIST %opencv_contrib_src%.zip (
        echo Downloading %opencv_contrib_url%...
        powershell -Command "(New-Object Net.WebClient).DownloadFile('%opencv_contrib_url%', '%opencv_contrib_src%.zip')"
    )
    echo Extracting %opencv_contrib_src%.zip...
    powershell -Command "Add-Type -AssemblyName System.IO.Compression.FileSystem; [System.IO.Compression.ZipFile]::ExtractToDirectory('%opencv_contrib_src%.zip', '.')"
)


echo Deleting existing build directories...
if exist x86 rd /s /q x86
if exist x64 rd /s /q x64

md x86
pushd x86

echo Configuring x86 build...
if "%1" == "-2019" (
	echo Compiling for 2019
	cmake -G "Visual Studio 16 2019" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 -DOPENCV_EXTRA_MODULES_PATH=..\%opencv_contrib_src%/modules ..\%opencv_src%
	) else (
	echo Compiling for 2017
	cmake -G "Visual Studio 15 2017" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 -DOPENCV_EXTRA_MODULES_PATH=..\%opencv_contrib_src%/modules ..\%opencv_src%
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
	cmake -G "Visual Studio 16 2019 Win64" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 -DOPENCV_EXTRA_MODULES_PATH=..\%opencv_contrib_src%/modules ..\%opencv_src%
	) else (
	echo Compiling for 2017
	cmake -G "Visual Studio 15 2017 Win64" -C %~dp0\cmake_options.txt -DCMAKE_INSTALL_PREFIX=%~dp0 -DOPENCV_EXTRA_MODULES_PATH=..\%opencv_contrib_src%/modules ..\%opencv_src%
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
md %bin_path%\Win32
md %bin_path%\Win64

md %lib_path%\Win32
md %lib_path%\Win64

::Copy to plugin dir
md ..\..\..\Libraries\Win32
md ..\..\..\Libraries\Win64

move /y x86\vc15\bin\*.* %bin_path%\Win32
move /y x64\vc15\bin\*.* %bin_path%\Win64
move /y x86\vc15\lib\*.lib ..\..\..\Libraries\Win32
move /y x64\vc15\lib\*.lib ..\..\..\Libraries\Win64

rd /s /q x86
rd /s /q x64
del /f OpenCV*.cmake

echo Done. Libs are in %lib_path%, Bins are in %bin_path%
