cmake_minimum_required(VERSION 2.8.3)
project(ec_master_test)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

#set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++17 -g")            # Debug
#set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG")  # Release
set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(ENABLE_XENO         CACHE BOOL "OFF")
set(SHARED_SRC          ${CMAKE_CURRENT_SOURCE_DIR}/shared_src) 

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

include_directories("${SHARED_SRC}/include")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

set(COMMON_LIBS
    ecat_test_utils${LIB_TYPE}
    ecat_master${LIB_TYPE}
    iitecat_iface${LIB_TYPE}
    soem${LIB_TYPE}
    ec_master_pb
    pthread
    zmq
    jsoncpp
    yaml-cpp
)

add_subdirectory(protobuf)
add_subdirectory(${SHARED_SRC})

optional_build(soem_test    examples/soem_test  ON)
optional_build(fw_update    examples/fw_update  OFF)
optional_build(flash_utils  examples/flash_utils OFF)

optional_build(basic        examples/basic      ON)
optional_build(sine_joint   examples/sine_joint OFF)
optional_build(joint_joy    examples/joint_joy  OFF)
#optional_build(coman_walk   examples/coman_walk OFF)
#optional_build(coman_test   examples/coman_test OFF)
#optional_build(coman_impedance   examples/coman_impedance OFF)
#optional_build(coman_torque examples/coman_torque OFF)
#optional_build(coman_V_loop examples/coman_V_loop OFF)
optional_build(ati_ft6      examples/ati_ft6_calib OFF)
optional_build(centAC_test  examples/centAC_test OFF)
optional_build(centAC_urdf  examples/centAC_urdf OFF)
optional_build(centAC_impedance  examples/centAC_impedance OFF)
optional_build(walkman_test examples/walkman_test OFF)
optional_build(hand_test    examples/hand_test OFF)
optional_build(impedance_trj_test examples/impedance_trj_test OFF)
optional_build(ecat_state   examples/ecat_states OFF)
optional_build(urdf_test    examples/urdf_test OFF)

optional_build(joy_forward      other_bin/joy_forward   OFF)
optional_build(spnav_forward    other_bin/spnav_forward OFF)
optional_build(zmq_pub          other_bin/zmq_pub   OFF)
optional_build(spline_test      other_bin/spline_test   OFF)
optional_build(lpms_forward     other_bin/lpms_forward  OFF)
optional_build(keyboard_control other_bin/keyboard_control OFF)
optional_build(rt_spline_test   other_bin/rt_spline_test OFF)
optional_build(xddp_proto       other_bin/xddp_proto OFF)
optional_build(xeno-3_test      other_bin/xeno-3_test OFF)
optional_build(cgos_gpio        other_bin/cgos_gpio OFF)
optional_build(motor_calibration other_bin/motor_calibration OFF)

