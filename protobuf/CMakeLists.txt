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

install(TARGETS pdo_pb DESTINATION lib)
install(FILES ${PROTO_HEADER}  DESTINATION include/protobuf)


