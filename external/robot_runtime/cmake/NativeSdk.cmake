# Share the same dependency installation across CLI and IDE build directories.
# Installation is explicit (scripts/setup_native.sh), never a configure side effect.
option(ROBOT_RUNTIME_USE_NATIVE_SDK "Use the pinned local native SDK when installed" ON)
if(DEFINED ENV{ROBOT_RUNTIME_NATIVE_ENV})
    set(native_sdk_default "$ENV{ROBOT_RUNTIME_NATIVE_ENV}")
else()
    set(native_sdk_default "${PROJECT_SOURCE_DIR}/.dependencies/native")
endif()
set(ROBOT_RUNTIME_NATIVE_SDK "${native_sdk_default}" CACHE PATH "Native SDK environment directory")
set(ROBOT_RUNTIME_NATIVE_SDK_ACTIVE FALSE)
if(ROBOT_RUNTIME_USE_NATIVE_SDK AND EXISTS "${ROBOT_RUNTIME_NATIVE_SDK}/bin/python")
    execute_process(
        COMMAND "${ROBOT_RUNTIME_NATIVE_SDK}/bin/python" -c
            "import sysconfig; print(sysconfig.get_path('purelib') + '/cmeel.prefix')"
        RESULT_VARIABLE sdk_result
        OUTPUT_VARIABLE sdk_prefix OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE sdk_error)
    if(NOT sdk_result EQUAL 0 OR NOT EXISTS "${sdk_prefix}/lib/cmake/pinocchio/pinocchioConfig.cmake")
        message(FATAL_ERROR "Incomplete native SDK at ${ROBOT_RUNTIME_NATIVE_SDK}. Run scripts/setup_native.sh.\n${sdk_error}")
    endif()
    # Set this before finding Eigen/Boost: Pinocchio must use its matching SDK headers.
    list(PREPEND CMAKE_PREFIX_PATH "${sdk_prefix}")
    set(ROBOT_RUNTIME_SDK_PYTHON "${ROBOT_RUNTIME_NATIVE_SDK}/bin/python")
    set(ROBOT_RUNTIME_NATIVE_SDK_ACTIVE TRUE)
    message(STATUS "Native SDK: ${ROBOT_RUNTIME_NATIVE_SDK}")
endif()

# Pinocchio's dependencies use Boost's package configuration with modern CMake.
set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
