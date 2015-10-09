cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

add_executable(${PROJECT_NAME}
spnav_thread.cpp
)

target_link_libraries(${PROJECT_NAME} pthread spnav)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
