cmake_minimum_required(VERSION 2.8.12)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

include_directories(include)

add_executable(${PROJECT_NAME}
    src/mainCalib.cpp
    src/ec_board_calib.cpp
    src/calibrationcontroller.cpp
    src/ati_iface.cpp
)

add_executable(testAti
    src/testati.cpp
    src/ati_iface.cpp
)


if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS})
target_link_libraries(testAti ${COMMON_LIBS})

if(NOT Xenomai_FOUND)
    set_cap_build(${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
if(NOT Xenomai_FOUND)
    set_cap_install(${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME})
endif()
