cmake_minimum_required(VERSION 2.8.3)
project(ec_master_test)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(COMAN_ROOT      $ENV{COMAN_ROOT}     CACHE PATH "Base path of the COMAN_shared")
set(ENABLE_XENO  CACHE BOOL "OFF")

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()

set(SHARED_SRC ${CMAKE_CURRENT_SOURCE_DIR}/shared_src) 

include_directories("${SHARED_SRC}")
include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

add_subdirectory(basic)
add_subdirectory(sine_joint)
add_subdirectory(joint_joy)
add_subdirectory(coman_walk)