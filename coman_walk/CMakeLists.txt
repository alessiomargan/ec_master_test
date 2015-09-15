cmake_minimum_required(VERSION 2.8.3)
project(coman_walk)

include_directories(include)

add_definitions(-DEPFL -DDEMO_TYPE=2)

add_executable(coman_walk
GlobalExt.cpp
Matrix.cpp
RTControl.cpp
Stabilizer.cpp
TestJoint.cpp
ec_boards_walk.cpp
${SHARED_SRC}/test_common.cpp
main_walk.cpp
)

if(ENABLE_XENO)
     target_link_libraries(coman_walk walkman_master iitecat_iface soem coman_os_rt pthread)
else()
    target_link_libraries(coman_walk walkman_master iitecat_iface soem coman_os pthread)
    add_custom_command(TARGET coman_walk POST_BUILD COMMAND set_cap.sh coman_walk COMMENT set_cap)
endif()

install(TARGETS coman_walk RUNTIME DESTINATION bin)
