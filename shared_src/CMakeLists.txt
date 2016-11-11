cmake_minimum_required(VERSION 2.8.3)
project(ecat_test_utils)

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
    set(RT_NRT_SRC
    ${SHARED_SRC}/rt_thread.cpp
    ${SHARED_SRC}/nrt_thread.cpp)
else()
    set(LIB_TYPE _nrt)
    set(RT_NRT_SRC
    ${SHARED_SRC}/nrt_thread.cpp)
endif()

set(LIB_NAME ${PROJECT_NAME}${LIB_TYPE})


add_library(${LIB_NAME} SHARED
    ${SHARED_SRC}/ec_boards_base.cpp
    ${SHARED_SRC}/test_common.cpp
    ${SHARED_SRC}/spline.cpp
    ${SHARED_SRC}/zmq_publisher.cpp    
    ${RT_NRT_SRC}
)

if ( Xenomai_FOUND )
    set_xeno_flags(${LIB_NAME})
endif()

target_link_libraries(${LIB_NAME} pdo_pb protobuf zmq jsoncpp yaml-cpp)

install(DIRECTORY ${SHARED_SRC}/include/  DESTINATION include)
install(TARGETS ${LIB_NAME} DESTINATION lib)

