cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

if ( Xenomai_FOUND )
    add_definitions("-D__XENO_PIPE__")
endif()

add_definitions("-DDEBUG")

add_executable(${PROJECT_NAME}
    ${SHARED_SRC}/test_common.cpp
    spnav_thread.cpp
)

target_link_libraries(${PROJECT_NAME} pthread spnav)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
