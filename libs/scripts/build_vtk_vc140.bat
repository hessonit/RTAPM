cd ../build/vc140
if not exist VTK mkdir VTK
cd VTK
call cmake -G "Visual Studio 14 2015 Win64" -C ../../../scripts/initial_cache_vtk.cmake ../../../VTK
call "%ProgramFiles(x86)%\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"
call msbuild /p:Configuration=Release INSTALL.vcxproj