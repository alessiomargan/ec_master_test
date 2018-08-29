cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)
include_directories("/usr/local/src/LpSensor-1.3.4-Linux-x86-64/include")
link_directories("/usr/local/src/libftd2xx-x86_64-1.3.6/build")
link_directories("/usr/local/src/LpSensor-1.3.4-Linux-x86-64/lib")

add_executable(${PROJECT_NAME}
    lpms_forward.cpp
)

target_link_libraries(${PROJECT_NAME} LpSensor ${COMMON_LIBS})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

