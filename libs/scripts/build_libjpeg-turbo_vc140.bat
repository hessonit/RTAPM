cd ../build/vc140
if not exist libjpeg-turbo mkdir libjpeg-turbo
cd libjpeg-turbo
call cmake -G "Visual Studio 14 2015 Win64" -C ../../../scripts/initial_cache_libjpeg-turbo.cmake ../../../libjpeg-turbo -DNASM=C:\\Users\\Adam\\AppData\\Local\\bin\\NASM\\nasm.exe
call "%ProgramFiles(x86)%\Microsoft Visual Studio 14.0\Common7\Tools\VsDevCmd.bat"
call msbuild /p:Configuration=Release INSTALL.vcxproj