cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

add_executable(${PROJECT_NAME}
ec_boards_sine.cpp
${SHARED_SRC}/test_common.cpp
sine_test.cpp
)

if(ENABLE_XENO)
     target_link_libraries(${PROJECT_NAME} walkman_master iitecat_iface soem coman_os_rt pthread)
else()
    target_link_libraries(${PROJECT_NAME} walkman_master iitecat_iface soem coman_os pthread)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND set_cap.sh ${PROJECT_NAME} COMMENT set_cap)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
