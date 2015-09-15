cmake_minimum_required(VERSION 2.8.3)
project(ec_master_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(COMAN_ROOT      $ENV{COMAN_ROOT}     CACHE PATH "Base path of the COMAN_shared")
set(ENABLE_XENO  CACHE BOOL "OFF")


find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)

if(ENABLE_XENO AND RTNET_INCLUDE_DIR)
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
  set(RT_NRT_SRCS src/rt_ipc.cpp) 
else()
  set(ENABLE_RTNET  OFF)
  message("NOT Found RTNET")
endif()

set(SHARED_SRC ${CMAKE_CURRENT_SOURCE_DIR}/shared_src) 

include_directories("${SHARED_SRC}")
include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

add_subdirectory(basic)
add_subdirectory(joint_joy)
add_subdirectory(coman_walk)