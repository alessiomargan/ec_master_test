cmake_minimum_required(VERSION 2.8.3)
project(walkman-ecat)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroOptionalBuild)
include(MacroSetcap)

set(CMAKE_CXX_FLAGS     "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(ENABLE_XENO         CACHE BOOL "OFF")

if ( ${ENABLE_XENO} )
    find_package(Xenomai)
    find_package(RTnet)
endif()

if ( Xenomai_FOUND )
    set(LIB_TYPE _rt)
else ()
    set(LIB_TYPE _nrt)
endif()

add_subdirectory(protobuf)

#######################
## libwalkman_master
#######################

find_package(soem${LIB_TYPE} REQUIRED)
find_package(iitecat_iface${LIB_TYPE} REQUIRED)

set(LIB_NAME walkman_master${LIB_TYPE})
add_definitions(-DKEEP_POWER_ON)

include_directories(include)
include_directories(${soem${LIB_TYPE}_INCLUDE_DIRS}
                    ${iitecat_iface${LIB_TYPE}_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})                    

set(LIB_SOURCES
    src/ec_boards_iface.cpp
    src/mc_centAC_esc.cpp
    src/mc_hipwr_esc.cpp
    src/mc_lowpwr_esc.cpp
    src/power_board.cpp
    src/power_coman_board.cpp
    src/ft6_esc.cpp
    src/foot_sensor_esc.cpp
    src/test_esc.cpp
)


if ( Xenomai_FOUND )
    set_xeno_flags(${LIB_NAME})
    set(RT_LIB_SOURCES
        src/rt_ipc.cpp
        )
endif()

add_library(${LIB_NAME} SHARED
            ${LIB_SOURCES} ${RT_LIB_SOURCES}
            )

add_dependencies(${LIB_NAME} protobuf)

target_link_libraries(${LIB_NAME} ${soem${LIB_TYPE}_LIBRARIES} 
                                  ${iitecat_iface${LIB_TYPE}_LIBRARIES}
                                  pthread 
                                  yaml-cpp)

########################################################################

set(VARS_PREFIX ${LIB_NAME})

set(${LIB_NAME}_MAJOR_VERSION 0)
set(${LIB_NAME}_MINOR_VERSION 0)
set(${LIB_NAME}_PATCH_VERSION 1)
set(${LIB_NAME}_VERSION ${${LIB_NAME}_MAJOR_VERSION}.${${LIB_NAME}_MINOR_VERSION}.${${LIB_NAME}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(${LIB_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(${LIB_NAME} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS ${LIB_NAME}
        EXPORT ${LIB_NAME}
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${LIB_NAME})


include(InstallBasicPackageFiles)
install_basic_package_files(${LIB_NAME} VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

include(AddUninstallTarget)
