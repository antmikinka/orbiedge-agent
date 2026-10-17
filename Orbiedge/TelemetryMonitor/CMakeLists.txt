register_fprime_module(
  AUTOCODER_INPUTS
    "${CMAKE_CURRENT_LIST_DIR}/TelemetryMonitor.fpp"
  SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/TelemetryMonitor.cpp"
  HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/TelemetryMonitor.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/BasiliskStatePacket.hpp"
)

find_path(ORB_ZMQ_INCLUDE_DIR NAMES zmq.h)
find_library(ORB_ZMQ_LIBRARY NAMES zmq libzmq)
if(NOT ORB_ZMQ_INCLUDE_DIR OR NOT ORB_ZMQ_LIBRARY)
  message(FATAL_ERROR
    "libzmq was not found. TelemetryMonitor links ZeroMQ and does not link Basilisk. "
    "On WSL or Ubuntu install libzmq3-dev. On Windows set CMAKE_PREFIX_PATH to a ZeroMQ install.")
endif()
target_include_directories(${FPRIME_CURRENT_MODULE} PUBLIC "${ORB_ZMQ_INCLUDE_DIR}")
target_link_libraries(${FPRIME_CURRENT_MODULE} PUBLIC "${ORB_ZMQ_LIBRARY}")
