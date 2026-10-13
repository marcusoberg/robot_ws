include(FetchContent)
# SML is a single-header library. Avoid downloading repository benchmarks/assets.
FetchContent_Declare(robot_sml
    URL https://raw.githubusercontent.com/boost-ext/sml/v1.2.0/include/boost/sml.hpp
    URL_HASH SHA256=f67d3bf7a4681c93a88f7a04db55b42a99ce45c4687b2f40c0f2d99ecb16c849
    DOWNLOAD_NO_EXTRACT TRUE
    DOWNLOAD_NAME sml.hpp
    SOURCE_SUBDIR robot-header-only
)
FetchContent_MakeAvailable(robot_sml)
if(EXISTS "${robot_sml_SOURCE_DIR}/include/boost/sml.hpp")
    # A full local source override is also supported for offline builds.
    set(robot_sml_include "${robot_sml_SOURCE_DIR}/include")
else()
    set(robot_sml_include "${CMAKE_CURRENT_BINARY_DIR}/robot-sml/include")
    file(MAKE_DIRECTORY "${robot_sml_include}/boost")
    configure_file("${robot_sml_SOURCE_DIR}/sml.hpp" "${robot_sml_include}/boost/sml.hpp" COPYONLY)
endif()
add_library(robot_sml_headers INTERFACE)
target_include_directories(robot_sml_headers INTERFACE "$<BUILD_INTERFACE:${robot_sml_include}>" "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
