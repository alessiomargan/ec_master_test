cmake_minimum_required(VERSION 2.8.3)
project(basic)


include_directories(include)

add_executable(basic
ec_boards_basic.cpp
${SHARED_SRC}/test_common.cpp
basic_test.cpp
)

if(ENABLE_XENO)
     target_link_libraries(basic walkman_master iitecat_iface soem coman_os_rt pthread)
else()
    target_link_libraries(basic walkman_master iitecat_iface soem coman_os pthread)
    add_custom_command(TARGET basic POST_BUILD COMMAND set_cap.sh basic COMMENT set_cap)
endif()

install(TARGETS basic RUNTIME DESTINATION bin)
