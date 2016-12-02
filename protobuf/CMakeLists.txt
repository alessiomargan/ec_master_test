cmake_minimum_required(VERSION 2.8.3)
#cmake_minimum_required(VERSION 3.2)
project(protos)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include(FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
set(PROTOBUF_IMPORT_DIRS "/home/embedded/gaz7_proto")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs
  dummy.proto
  motor_pdo.proto
  ft6_pdo.proto
  power_pack_pdo.proto
  foot_sens_pdo.proto
  ecat_pdo.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)

add_definitions(-fPIC)
protobuf_generate_cpp(PROTO_SRC PROTO_HEADER ${msgs})
protobuf_generate_python(PROTO_PY ${msgs})
add_library (pdo_pb ${PROTO_HEADER} ${PROTO_SRC} ${PROTO_PY})

########################################################################

set(VARS_PREFIX pdo_pb)

set(pdo_pb_MAJOR_VERSION 0)
set(pdo_pb_MINOR_VERSION 0)
set(pdo_pb_PATCH_VERSION 1)
set(pdo_pb_VERSION ${pdo_pb_MAJOR_VERSION}.${pdo_pb_MINOR_VERSION}.${pdo_pb_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(pdo_pb PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                                "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(pdo_pb PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                               SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS pdo_pb
        EXPORT pdo_pb
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotInterface dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS pdo_pb)


include(InstallBasicPackageFiles)
install_basic_package_files(pdo_pb VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)                                    

# include(AddUninstallTarget)

install(FILES ${PROTO_HEADER}  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/protobuf)


