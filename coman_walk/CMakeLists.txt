cmake_minimum_required(VERSION 2.8.3)
project(coman_walk)

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
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -L../../soem_1-3-0/build")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
  set(RT_NRT_SRCS src/rt_ipc.cpp) 
else()
  set(ENABLE_RTNET  OFF)
  message("NOT Found RTNET")
endif()

include_directories(include)
set(SHARED_SRC ${PROJECT_SOURCE_DIR}/../shared_src) 
include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

add_executable(coman_walk
ec_boards_walk.cpp
${SHARED_SRC}/test_common.cpp
main_walk.cpp
)

if(ENABLE_XENO)
     target_link_libraries(coman_walk walkman_master iitecat_iface soem coman_os_rt pthread)
else()
    target_link_libraries(coman_walk walkman_master iitecat_iface soem coman_os pthread)
    add_custom_command(TARGET coman_walk POST_BUILD COMMAND set_cap.sh coman_walk COMMENT set_cap)
endif()

install(TARGETS coman_walk RUNTIME DESTINATION bin)
