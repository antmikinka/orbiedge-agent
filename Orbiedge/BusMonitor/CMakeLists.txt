register_fprime_module(
  AUTOCODER_INPUTS
    "${CMAKE_CURRENT_LIST_DIR}/BusMonitor.fpp"
  SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/BusMonitor.cpp"
  HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/BusMonitor.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/../FdirRules/FdirRules.hpp"
)

# No ZeroMQ. Do not FATAL_ERROR when libzmq is absent.
target_include_directories(${FPRIME_CURRENT_MODULE} PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/../.."
)
