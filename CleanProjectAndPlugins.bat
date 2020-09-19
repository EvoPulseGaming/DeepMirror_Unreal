@echo off
setlocal
:PROMPT
set /p var=Are you sure you want to clean the project and all plugins?[Y/N]: 
if /I not %var%== Y exit

del /q/s *".sln"

rmdir /q/s ".vs"
rmdir /q/s "Build"
rmdir /q/s "Binaries"
rmdir /q/s "Intermediate"
rmdir /q/s "Saved"

rmdir /q/s "Plugins/DeepMirrorPlugin/Binaries"
rmdir /q/s "Plugins/DeepMirrorPlugin/Intermediate"

rmdir /q/s "Plugins/Dlib/Binaries"
rmdir /q/s "Plugins/Dlib/Libraries"
rmdir /q/s "Plugins/Dlib/Intermediate"

rmdir /q/s "Plugins/Dlib/Source/ThirdParty/Dlib/build/x64"
rmdir /q/s "Plugins/Dlib/Source/ThirdParty/Dlib/build/x86"
rmdir /q/s "Plugins/Dlib/Source/ThirdParty/Dlib/include"
rmdir /q/s "Plugins/Dlib/Source/ThirdParty/Dlib/lib"
rmdir /q/s "Plugins/Dlib/Source/ThirdParty/Dlib/build/dlib-19.21"

rmdir /q/s "Plugins/OpenCV/Binaries"
rmdir /q/s "Plugins/OpenCV/Libraries"
rmdir /q/s "Plugins/OpenCV/Intermediate"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/build/x64"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/build/x86"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/include"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/lib"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/etc"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/python"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/build/opencv-4.4.0"
rmdir /q/s "Plugins/OpenCV/Source/ThirdParty/OpenCV/build/opencv_contrib-4.4.0"

endlocal
