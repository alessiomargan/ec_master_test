cmake_minimum_required(VERSION 2.8.3)
project(ec_master_test)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

set(CMAKE_CXX_FLAGS 	"${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(ENABLE_XENO  	CACHE BOOL "OFF")
set(SHARED_SRC 		${CMAKE_CURRENT_SOURCE_DIR}/shared_src) 


	
if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()


include_directories("${SHARED_SRC}/include")
#link_directories("${SHARED_SRC}")

set(COMMON_LIBS
    shared_utils
    walkman_master
    iitecat_iface
    soem
    pthread
    zmq
    jsoncpp
)

add_subdirectory(${SHARED_SRC})

optional_build(soem_test 	examples/soem_test	ON)

optional_build(basic 		examples/basic		ON)
optional_build(sine_joint 	examples/sine_joint	OFF)
optional_build(joint_joy 	examples/joint_joy	OFF)
optional_build(coman_walk	examples/coman_walk	OFF)
optional_build(coman_test	examples/coman_test	OFF)

optional_build(joy_forward 	other_bin/joy_forward	OFF)
optional_build(spnav_forward 	other_bin/spnav_forward	OFF)
optional_build(zmq_pub          other_bin/zmq_pub	OFF)
optional_build(spline_test      other_bin/spline_test	OFF)
optional_build(lpms_forward 	other_bin/lpms_forward	OFF)
