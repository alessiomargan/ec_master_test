cmake_minimum_required(VERSION 2.8.3)
project(shared_utils)

	
if ( Xenomai_FOUND )
    set(RT_NRT_SRC
	${SHARED_SRC}/rt_ipc.cpp
	${SHARED_SRC}/rt_thread.cpp
	${SHARED_SRC}/nrt_thread.cpp)
else()
    set(RT_NRT_SRC
	${SHARED_SRC}/nrt_thread.cpp)
endif()

add_library(shared_utils SHARED
    ${SHARED_SRC}/ec_boards_base.cpp
    ${SHARED_SRC}/test_common.cpp
    ${SHARED_SRC}/spline.cpp
    ${RT_NRT_SRC}
)

if ( Xenomai_FOUND )
    set_xeno_flags(shared_utils)
endif()

install(DIRECTORY ${SHARED_SRC}/include/  DESTINATION include)
install(TARGETS shared_utils DESTINATION lib)

