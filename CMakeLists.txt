cmake_minimum_required(VERSION 2.8.3)
project(walkman-ecat)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
set(ENABLE_XENO  ON)

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
endif()

set(HEADER_INSTALL_ROOT $ENV{WALKMAN_ETHERCAT_MASTER_ROOT}/include/ CACHE PATH "Where to install the headers")
set(LIB_INSTALL_ROOT    $ENV{WALKMAN_ETHERCAT_MASTER_ROOT}/lib/     CACHE PATH "Where to install the libraries")
set(SOEM_ROOT            $ENV{SOEM_ROOT}     CACHE PATH "Base path of the SOEM headers")

add_definitions(-DKEEP_POWER_ON)

###########
## Build ##
###########

include_directories(include)

include_directories(${SOEM_ROOT}/include)
link_directories(${SOEM_ROOT}/lib)

add_library(walkman_master SHARED
    src/ec_boards_iface.cpp
    src/mc_hipwr_esc.cpp
    src/mc_lowpwr_esc.cpp
    src/power_board.cpp
    src/ft6_esc.cpp
    src/test_esc.cpp
    ${RT_NRT_SRCS}
)

target_link_libraries(walkman_master iitecat_iface soem pthread yaml-cpp)


add_executable(
     ft_ati_test
     src/ati_iface.cpp
     test/ft_ati_test.cpp)

target_link_libraries(ft_ati_test walkman_master pthread yaml-cpp)

#add_executable(
#     walkman_test
#     test/walkman_test.cpp)

#target_link_libraries(walkman_test walkman_master iitecat_iface soem pthread yaml-cpp)

#add_executable(
#     walkman_test_2
#     test/walkman_test_2.cpp)

#target_link_libraries(walkman_test_2 walkman_master iitecat_iface soem pthread yaml-cpp)

#add_executable(
#     walkman_test_3
#     test/test_common
#     test/walkman_test_3.cpp)

#target_link_libraries(walkman_test_3 walkman_master iitecat_iface soem pthread yaml-cpp)

add_executable(
     sine_joint
     test/test_common.cpp
     test/sine_joint.cpp)

target_link_libraries(sine_joint walkman_master pthread yaml-cpp)

add_executable(
     coman_sine_joint
     test/test_common.cpp
     test/coman_sine_joint.cpp)

target_link_libraries(coman_sine_joint walkman_master pthread yaml-cpp)

#add_executable(
#     zero
#     test/test_common.cpp
#     test/zero.cpp)

#target_link_libraries(zero walkman_master iitecat_iface soem pthread yaml-cpp)

add_executable(
     simple_test
     test/test_common.cpp
     test/simple_test.cpp)

target_link_libraries(simple_test walkman_master pthread yaml-cpp)


add_executable(
     firm_update
     firm_update.cpp)

target_link_libraries(firm_update walkman_master pthread yaml-cpp)

add_executable(
     zpub
     test/test_common.cpp
     test/zpub.cpp)

target_link_libraries(zpub zmq jsoncpp pthread)

if( NOT (ENABLE_XENO AND RTNET_INCLUDE_DIR) )
    add_custom_command(TARGET ft_ati_test POST_BUILD COMMAND set_cap.sh ft_ati_test COMMENT set_cap)
    add_custom_command(TARGET simple_test POST_BUILD COMMAND set_cap.sh simple_test COMMENT set_cap)
    add_custom_command(TARGET sine_joint POST_BUILD COMMAND set_cap.sh sine_joint COMMENT set_cap)
    add_custom_command(TARGET coman_sine_joint POST_BUILD COMMAND set_cap.sh coman_sine_joint COMMENT set_cap)
    add_custom_command(TARGET zero POST_BUILD COMMAND set_cap.sh zero COMMENT set_cap)
    add_custom_command(TARGET firm_update POST_BUILD COMMAND set_cap.sh firm_update COMMENT set_cap)
endif()


# Install
install(DIRECTORY include/  DESTINATION ${HEADER_INSTALL_ROOT})
install(TARGETS walkman_master  DESTINATION ${LIB_INSTALL_ROOT})

