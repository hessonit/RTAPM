set(CMAKE_C_FLAGS_DEBUG "/D_DEBUG /MTd /Zi /Ob0 /Od /RTC1 /MP -DPSAPI_VERSION=1" CACHE STRING "")
set(CMAKE_C_FLAGS_RELEASE        "/MP /MT /O2 /Ob2 /D NDEBUG -DPSAPI_VERSION=1" CACHE STRING "")
set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MTd /Zi /Ob0 /Od /RTC1 /MP -DPSAPI_VERSION=1" CACHE STRING "")
set(CMAKE_CXX_FLAGS_RELEASE        "/MT /MT /O2 /Ob2 /D NDEBUG -DPSAPI_VERSION=1" CACHE STRING "")