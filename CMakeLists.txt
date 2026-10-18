cmake_minimum_required(VERSION 3.26)
project(orbiedge-agent VERSION 0.1.0 LANGUAGES C CXX)

cmake_policy(SET CMP0048 NEW)

set(FPRIME_FRAMEWORK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/fprime" CACHE PATH "nasa/fprime checkout")
if(NOT EXISTS "${FPRIME_FRAMEWORK_PATH}/cmake/FPrime.cmake")
  message(FATAL_ERROR
    "The nasa/fprime submodule is not present at third_party/fprime. "
    "Run scripts/init_submodules.ps1. Do not unpack the framework over this tree.")
endif()

# F´ devel has no Windows platform. Configure this file on Linux or WSL.
# googletest is not in the depth-1 submodule, so leave BUILD_TESTING off.
set(FPRIME_INCLUDE_FRAMEWORK_CODE OFF)
find_package(FPrime REQUIRED PATHS "${FPRIME_FRAMEWORK_PATH}")
fprime_add_validation_flags()
add_subdirectory("${FPRIME_FRAMEWORK_PATH}" "F-Prime")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/TelemetryMonitor")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/FdirTypes")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/BusMonitor")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/FdirManager")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/AgentUplinkFilter")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/SimBridge")
add_fprime_subdirectory("${CMAKE_CURRENT_LIST_DIR}/Orbiedge/OrbiEdgeDeployment")

register_fprime_deployment(
  OrbiEdgeApp
  SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/Orbiedge/OrbiEdgeDeployment/Main.cpp"
  DEPENDS
    Orbiedge_OrbiEdgeDeployment
)
