#CMAKE_INSTALL_PREFIX:PATH=C:/kinxplore/libs/install/vc120/VTK
#BUILD_TESTING:BOOL=0
#BUILD_SHARED_LIBS:BOOL=0
#VTK_Group_Views:BOOL=1
#VTK_USE_SYSTEM_GLEW:BOOL=0

include( ${CMAKE_CURRENT_LIST_DIR}"/initial_compiler_flags_common.cmake" )
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/../install/vc140/VTK CACHE PATH "")

set(BUILD_SHARED_LIBS 0 CACHE BOOL "")
set(BUILD_TESTS 0 CACHE BOOL "")
set(VTK_Group_Views 1 CACHE BOOL "")
set(VTK_USE_SYSTEM_GLEW 0 CACHE BOOL "")
