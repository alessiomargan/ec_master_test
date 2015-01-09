cmake_minimum_required(VERSION 2.8.3)
project(walkman-ecat)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
set(ENABLE_RTNET  ON)

if(ENABLE_RTNET AND RTNET_INCLUDE_DIR)
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -L../../soem_1-3-0/build")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
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
)

# add_executable(
#     walkman_test
#     walkman_test.cpp)

target_link_libraries(walkman_master iitecat_iface soem pthread )

# target_link_libraries(
#     walkman_test walkman_master
#     iitecat_iface soem pthread)


# Install
install(DIRECTORY include/  DESTINATION ${HEADER_INSTALL_ROOT})
install(TARGETS walkman_master  DESTINATION ${LIB_INSTALL_ROOT})
