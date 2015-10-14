cmake_minimum_required(VERSION 2.8.3)
project(ec_master_test)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)

set(CMAKE_CXX_FLAGS 	"${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(COMAN_ROOT      	$ENV{COMAN_ROOT}     CACHE PATH "Base path of the COMAN_shared")
set(ENABLE_XENO  	CACHE BOOL "OFF")
set(SHARED_SRC 		${CMAKE_CURRENT_SOURCE_DIR}/shared_src) 

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
    set(RT_SRC ${SHARED_SRC}/rt_ipc.cpp)
endif()

add_library(shared_utils SHARED
    ${SHARED_SRC}/ec_boards_base.cpp
    ${SHARED_SRC}/test_common.cpp
    ${RT_SRC}
)

install(DIRECTORY ${SHARED_SRC}/include/  DESTINATION include)
install(TARGETS shared_utils DESTINATION lib)

include_directories("${SHARED_SRC}/include")
link_directories("${SHARED_SRC}")

include_directories("${COMAN_ROOT}/include")
link_directories("${COMAN_ROOT}/lib")

set(COMMON_LIBS
    shared_utils
    walkman_master
    iitecat_iface
    soem
    pthread
    zmq
    jsoncpp
)

optional_build(basic 		examples/basic		ON)
optional_build(sine_joint 	examples/sine_joint	OFF)
optional_build(joint_joy 	examples/joint_joy	OFF)
optional_build(coman_walk	examples/coman_walk	OFF)
optional_build(joy_forward 	examples/joy_forward	OFF)
optional_build(spnav_forward 	examples/spnav_forward	OFF)
