include( ${CMAKE_CURRENT_LIST_DIR}"/initial_compiler_flags_common.cmake" )
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libfreenect2 CACHE PATH "")

#set(BUILD_SHARED_LIBS 0 CACHE BOOL "")
set(BUILD_TESTING 0 CACHE BOOL "")
set(CMAKE_CXX_MP_FLAG 1 CACHE BOOL "")
set(BUILD_OPENNI2_DRIVER 0 CACHE BOOL "")
set(BUILD_EXAMPLES 1 CACHE BOOL "")
set(ENABLE_CUDA 0 CACHE BOOL "")
set(ENABLE_CXX11 1 CACHE BOOL "")
set(ENABLE_OPENCL 0 CACHE BOOL "")
set(ENABLE_OPENGL 1 CACHE BOOL "")
set(ENABLE_TEGRAJPEG 0 CACHE BOOL "")
set(TURBOJPEG_WORKS 1 CACHE BOOL "")

set(JPEG_LIBRARY ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libjpeg-turbo/lib/jpeg-static.lib CACHE FILEPATH "")
set(JPEG_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libjpeg-turbo/include CACHE PATH "")
set(TurboJPEG_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libjpeg-turbo/include CACHE PATH "")
set(TurboJPEG_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libjpeg-turbo/lib/turbojpeg-static.lib CACHE FILEPATH "")
set(TurboJPEG_DLL ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/libjpeg-turbo/bin/turbojpeg.dll CACHE FILEPATH "")
set(GLFW3_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/glfw/include CACHE PATH "")
set(GLFW3_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/glfw/lib/glfw3dll.lib CACHE FILEPATH "")
set(GLFW3_DLL ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/glfw/lib/glfw3.dll CACHE FILEPATH "")