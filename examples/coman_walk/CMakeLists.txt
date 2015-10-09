cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

add_definitions(-DEPFL -DDEMO_TYPE=2)

add_executable(coman_walk
    GlobalExt.cpp
    Matrix.cpp
    RTControl.cpp
    Stabilizer.cpp
    TestJoint.cpp
    ec_boards_walk.cpp
    main_walk.cpp
)

if(ENABLE_XENO)
     target_link_libraries(${PROJECT_NAME} coman_os_rt ${COMMON_LIBS})
else()
    target_link_libraries(${PROJECT_NAME} coman_os ${COMMON_LIBS})
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND set_cap.sh ${PROJECT_NAME} COMMENT set_cap)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

