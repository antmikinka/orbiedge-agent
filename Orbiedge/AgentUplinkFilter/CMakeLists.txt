register_fprime_module(
  AUTOCODER_INPUTS
    "${CMAKE_CURRENT_LIST_DIR}/AgentUplinkFilter.fpp"
  SOURCES
    "${CMAKE_CURRENT_LIST_DIR}/AgentUplinkFilter.cpp"
  HEADERS
    "${CMAKE_CURRENT_LIST_DIR}/AgentUplinkFilter.hpp"
    "${CMAKE_CURRENT_LIST_DIR}/../FdirRules/FdirRules.hpp"
  DEPENDS
    Orbiedge_BusMonitor
)

# No ZeroMQ. Do not FATAL_ERROR when libzmq is absent.
target_include_directories(${FPRIME_CURRENT_MODULE} PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/../.."
)
