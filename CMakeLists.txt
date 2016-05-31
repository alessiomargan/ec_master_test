cmake_minimum_required(VERSION 2.8.3)
project(walkman-ecat)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(ENABLE_XENO         CACHE BOOL "OFF")
set(SOEM_ROOT           $ENV{SOEM_ROOT} CACHE PATH "Base path of the SOEM headers")

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

add_subdirectory(protobuf)

#######################
## libwalkman_master
#######################

set(LIB_NAME walkman_master${LIB_TYPE})
add_definitions(-DKEEP_POWER_ON)

include_directories(include)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${SOEM_ROOT}/include)
link_directories(${SOEM_ROOT}/lib)

add_library(walkman_master${LIB_TYPE} SHARED
    src/ec_boards_iface.cpp
    src/mc_hipwr_esc.cpp
    src/mc_lowpwr_esc.cpp
    src/mc_lowpwr_sph_esc.cpp
    src/mc_lowpwr_sphbrk_esc.cpp
    src/power_board.cpp
    src/power_coman_board.cpp
    src/ft6_esc.cpp
    src/test_esc.cpp
)

add_dependencies(walkman_master${LIB_TYPE} pdo_pb)

if ( Xenomai_FOUND )
    set_xeno_flags(${LIB_NAME})
endif()

target_link_libraries(${LIB_NAME} iitecat_iface${LIB_TYPE} soem${LIB_TYPE} pthread yaml-cpp)

# Install
install(DIRECTORY include/  DESTINATION include)
install(TARGETS ${LIB_NAME}  DESTINATION lib)

