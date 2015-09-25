cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})


include_directories(include)

add_executable(joint_joy
ec_boards_joint_joy.cpp
${SHARED_SRC}/test_common.cpp
joint_joy_test.cpp
)

if(ENABLE_XENO)
     target_link_libraries(joint_joy walkman_master iitecat_iface soem coman_os_rt pthread)
else()
    target_link_libraries(joint_joy walkman_master iitecat_iface soem coman_os pthread)
    add_custom_command(TARGET joint_joy POST_BUILD COMMAND set_cap.sh joint_joy COMMENT set_cap)
endif()

install(TARGETS joint_joy RUNTIME DESTINATION bin)
