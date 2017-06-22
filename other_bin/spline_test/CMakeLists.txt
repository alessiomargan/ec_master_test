cmake_minimum_required(VERSION 3.5.1)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

find_package (Eigen3 NO_MODULE)

include_directories(include ${EIGEN3_INCLUDE_DIR} )

add_executable(${PROJECT_NAME}
    spline_test.cpp
    ${SHARED_SRC}/spline.cpp
)

target_link_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
