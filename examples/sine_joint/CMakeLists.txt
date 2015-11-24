cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

add_executable(${PROJECT_NAME}
    ec_boards_sine.cpp
    sine_test.cpp
)

if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

target_link_libraries(${PROJECT_NAME} boost_system ${COMMON_LIBS})

if( NOT Xenomai_FOUND)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND set_cap.sh ${PROJECT_NAME} COMMENT set_cap)
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

