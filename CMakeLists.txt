cmake_minimum_required(VERSION 2.6)
project(Niksa)



#set(PCL_DIR "${PROJECT_SOURCE_DIR}/libs/install/PCL/cmake/")

#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/libs/install/vc140/opencv")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(freenect2_DIR "${PROJECT_SOURCE_DIR}/libs/install/vc140/libfreenect2/lib/cmake/freenect2")
find_package(freenect2 REQUIRED)
set(freenect2_DLL ${PROJECT_SOURCE_DIR}/libs/install/vc140/libfreenect2/bin/freenect2.dll)
include_directories(${freenect2_INCLUDE_DIRS})
link_directories(${freenect2_LIBRARIES})

set(libjpegturbo_LIBRARIES "${PROJECT_SOURCE_DIR}/libs/install/vc140/libjpeg-turbo/lib/turbojpeg-static.lib")
link_directories(${libjpegturbo_LIBRARIES})

FIND_PACKAGE(OpenGL)

set(GLFW3_DIR "${PROJECT_SOURCE_DIR}/libs/install/vc140/glfw/include")
#FIND_PACKAGE(GLFW3)
#find_package(glfw REQUIRED)
set(GLFW3_DLL "${PROJECT_SOURCE_DIR}/libs/install/vc140/glfw/lib/glfw3.dll")
set(GLFW3_LIBRARY "${PROJECT_SOURCE_DIR}/libs/install/vc140/glfw/lib/glfw3dll.lib")
include_directories(${GLFW3_DIR})
link_directories(${GLFW3_LIBRARY})

set(libusb_LIBRARY "${PROJECT_SOURCE_DIR}/libs/libfreenect2/depends/libusb/MS64/dll/libusb-usbdk-1.0.lib")
link_directories(${libusb_LIBRARY})


set(VTK_DIR "${PROJECT_SOURCE_DIR}/libs/install/vc140/VTK/lib/cmake/vtk-6.3")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# C:\local\boost_1_60_0\lib64-msvc-14.0

# set(Boost_USE_STATIC_LIBS   ON)
set(BOOST_DIR "C:/local/boost_1_60_0")
# set(BOOST_DLL "C:/local/boost_1_60_0/lib64-msvc-14.0/")
set(BOOST_LIBRARIES "C:/local/boost_1_60_0/lib64-msvc-14.0/")
# linker input additional library directories
include_directories(${BOOST_DIR})
link_directories(${BOOST_LIBRARIES})

set(OGRE_DIR "${PROJECT_SOURCE_DIR}/libs/install/vc140/OGRE/include")
set(OGRE_DLL "${PROJECT_SOURCE_DIR}/libs/install/vc140/OGRE/bin/Release")
set(OGRE_LIBRARIES "${PROJECT_SOURCE_DIR}/libs/install/vc140/OGRE/lib/Release/OgreMain.lib")
include_directories(${OGRE_DIR})
link_directories(${OGRE_LIBRARIES})



set(common_libraries 
	${OpenCV_LIBRARIES}
#	${PCL_LIBRARIES} 
	${freenect2_LIBRARIES} 
	${VTK_LIBRARIES}
#	${libjpegturbo_LIBRARIES} 
	${GLFW3_LIBRARY} 
	${libusb_LIBRARY}
	${OGRE_LIBRARIES}
	${BOOST_LIBRARIES}

	)


#add_executable(capture_depth_frames src/capture_depth_frames.cpp)
#target_link_libraries(capture_depth_frames ${common_libraries})


include_directories("${PROJECT_SOURCE_DIR}/include")
add_executable(Niksa
	include/viewer.h
	src/viewer.cpp
	include/flextGL.h
	src/flextGL.cpp
	include/pt.h
	#include/k2g.h
	src/pt.cpp
	include/util.h
	src/util.cpp
	include/calibration.h
	src/calibration.cpp
	# include/VideoFaceDetector.h
	# src/VideoFaceDetector.cpp	
	include/Projector.h
	src/Projector.cpp
    include/intrinsics.h
    src/intrinsics.cpp
    include/CTObject.h
	src/CTObject.cpp
    include/objObject.h
	src/objObject.cpp
  
	src/Niksa.cpp
	)
target_link_libraries(Niksa ${common_libraries})
#add_executable(kinfu_preview src/kinfu_preview.cpp)
#target_link_libraries(kinfu_preview ${common_libraries})

# add_custom_command(TARGET Niksa POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy_if_different
        # "${PROJECT_SOURCE_DIR}/libs/irrlicht/bin/Win32-VisualStudio/Irrlicht.dll"
        # $<TARGET_FILE_DIR:Niksa>)

add_custom_command(TARGET Niksa POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/libs/install/vc140/VTK/bin/"
        $<TARGET_FILE_DIR:Niksa>)
		
add_custom_command(TARGET Niksa POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/libs/install/vc140/OGRE/bin/Release/"
        $<TARGET_FILE_DIR:Niksa>)

# add_custom_command(TARGET capture_depth_frames POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy_if_different
        # "${PROJECT_SOURCE_DIR}/libs/libfreenect2/depends/libusb/MS64/dll/libusb-usbdk-1.0.dll"
        # $<TARGET_FILE_DIR:capture_depth_frames>/libusb-1.0.dll)

# add_custom_command(TARGET capture_depth_frames POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy_if_different
        # ${freenect2_DLL}
        # $<TARGET_FILE_DIR:capture_depth_frames>)

# add_custom_command(TARGET capture_depth_frames POST_BUILD
    # COMMAND ${CMAKE_COMMAND} -E copy_if_different
        # ${GLFW3_DLL}
        # $<TARGET_FILE_DIR:capture_depth_frames>)