cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

# DEMO_TYPE=1 joint limit
# DEMO_TYPE=2 walking
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

if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS})

if(NOT Xenomai_FOUND)
    set_cap_build(${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
if(NOT Xenomai_FOUND)
    set_cap_install(${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME})
endif()

