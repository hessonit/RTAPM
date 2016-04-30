cd ../libfreenect2/depends
call install_libusb_vs2015.cmd
cd ../../../build/vc140
if not exist libfreenect2 mkdir libfreenect2
cd libfreenect2
call cmake -G "Visual Studio 14 2015 Win64" -C ../../../scripts/initial_cache_libfreenect2.cmake ../../../libfreenect2
call "%ProgramFiles(x86)%\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"
call msbuild /p:Configuration=Release INSTALL.vcxproj