# Local, build-directory-only correction for the pinned Ruckig 0.15.3 profile.
# Velocity profiles do not define pf; extrema must use the integrated endpoint.
# Also include stationary points in constant-acceleration (zero-jerk) phases.
# Never modify an externally supplied source checkout or silently patch another release.
set(profile_original "${ruckig_SOURCE_DIR}/include/ruckig/profile.hpp")
file(SHA256 "${profile_original}" profile_hash)
if(NOT profile_hash STREQUAL "1b7441140a33c8b301fea051e04d559f0b7441c06dd65c77af0626f74ccda571")
    message(FATAL_ERROR "Ruckig profile differs from pinned 0.15.3; review the local extrema fix")
endif()
file(READ "${profile_original}" profile_content)
string(REPLACE
    "if (pf < extrema.min) {\n            extrema.min = pf;"
    "if (p.back() < extrema.min) {\n            extrema.min = p.back();"
    profile_content "${profile_content}")
string(REPLACE
    "if (pf > extrema.max) {\n            extrema.max = pf;"
    "if (p.back() > extrema.max) {\n            extrema.max = p.back();"
    profile_content "${profile_content}")
string(REPLACE
    "check_position_extremum((-a + D_sqrt) / j, t_sum, t, p, v, a, j, ext);\n            }\n        }"
    "check_position_extremum((-a + D_sqrt) / j, t_sum, t, p, v, a, j, ext);\n            }\n        } else if (a != 0.0) {\n            check_position_extremum(-v / a, t_sum, t, p, v, a, j, ext);\n        }"
    profile_content "${profile_content}")
set(profile_overlay "${CMAKE_CURRENT_BINARY_DIR}/ruckig-fixed/include")
file(CONFIGURE OUTPUT "${profile_overlay}/ruckig/profile.hpp" CONTENT "${profile_content}" @ONLY)
target_include_directories(ruckig BEFORE PUBLIC "$<BUILD_INTERFACE:${profile_overlay}>")
# Ensure installed runtime headers see the same correction as the compiled dependency.
# Upstream installs the original header first. Force the reviewed overlay afterwards;
# a timestamp-only install comparison can otherwise keep the upstream file.
install(CODE "file(COPY_FILE \"${profile_overlay}/ruckig/profile.hpp\" \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/ruckig/profile.hpp\")")
