cmake_minimum_required(VERSION 2.8.3)
get_filename_component(PRJ_DIR ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PRJ_DIR})

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} )

include_directories(include)


add_executable(${PROJECT_NAME}
    ec_boards_${PRJ_DIR}.cpp
    ${PRJ_DIR}.cpp
)

if ( Xenomai_FOUND )
    set_xeno_flags(${PROJECT_NAME})
endif()

target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

if(NOT Xenomai_FOUND)
    set_cap_build(${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
if(NOT Xenomai_FOUND)
    set_cap_install(${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME})
endif()

