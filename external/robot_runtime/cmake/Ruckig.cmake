# Local Community functionality only: never enable the cloud waypoint client.
include(FetchContent)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ruckig
    URL https://codeload.github.com/pantor/ruckig/tar.gz/refs/tags/v0.15.3
    URL_HASH SHA256=949c8b0b3227cf3f71a9750126b2952ba3a7af43e3be85505a853ebd17671166
)
FetchContent_MakeAvailable(ruckig)
include(${CMAKE_CURRENT_LIST_DIR}/RuckigProfileFix.cmake)
