if not exist build mkdir build
cd build
call cmake -G "Visual Studio 14 2015 Win64" ../
call "%ProgramFiles(x86)%\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"
call msbuild /p:Configuration=Release ALL_BUILD.vcxproj