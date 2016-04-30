cd ../build/vc140
if not exist glfw mkdir glfw
cd glfw
call cmake -G "Visual Studio 14 2015 Win64" -C ../../../scripts/initial_cache_glfw.cmake ../../../glfw
call "%ProgramFiles(x86)%\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"
call msbuild /p:Configuration=Release INSTALL.vcxproj