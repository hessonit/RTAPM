setlocal enabledelayedexpansion

cd libs
if not exist build mkdir build
cd build
if not exist vc140 mkdir vc140
cd ..
if not exist install mkdir install
cd install
if not exist vc140 mkdir vc140
cd ../scripts/
start /WAIT build_glfw_vc140.bat ^&^& exit
start /WAIT build_libjpeg-turbo_vc140.bat ^&^& exit
start /WAIT build_libfreenect2_vc140.bat ^&^& exit
start /WAIT build_opencv_vc140.bat ^&^& exit