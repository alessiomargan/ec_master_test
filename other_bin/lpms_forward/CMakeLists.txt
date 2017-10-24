cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

if ( Xenomai_FOUND )
    add_definitions("-D__XENO_PIPE__")
    set(LIB_ECAT_MASTER ecat_master_rt)
else()
    set(LIB_ECAT_MASTER ecat_master_nrt)
endif()


add_definitions("-std=gnu++11")

add_executable(${PROJECT_NAME}
    ${SHARED_SRC}/test_common.cpp
    ${SHARED_SRC}/zmq_publisher.cpp    
    lpms_forward.cpp
)

target_link_libraries(${PROJECT_NAME} pthread zmq jsoncpp LpSensor pdo_pb ec_master_pb protobuf ${LIB_ECAT_MASTER})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

