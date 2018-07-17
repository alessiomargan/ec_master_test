cmake_minimum_required(VERSION 3.1.0)
project(motorCalibGui)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

include_directories(include)

####################################
## QT STUFF
set (CMAKE_PREFIX_PATH $ENV{QT_CUSTOM_INSTALL_MODULE_PATH})
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Quick   CONFIG REQUIRED)
find_package(Qt5Qml     CONFIG REQUIRED)
find_package(Qt5Charts  CONFIG REQUIRED)
set(QRC_FILES qml/qml.qrc)
qt5_add_resources(RCC_FILES  ${QRC_FILES} )

message ( "-- QT Version " ${Qt5Core_VERSION_STRING} )
message ( "-- Env : " ${CMAKE_PREFIX_PATH} )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

include_directories( ${Qt5Widgets_INCLUDE_DIRS} )
include_directories( ${Qt5Qml_INCLUDE_DIRS} )
include_directories( ${Qt5Charts_INCLUDE_DIRS} )

link_directories( ${Qt5Widgets_LIBRARIES} )
link_directories( ${Qt5Qml_LIBRARIES} )
link_directories( ${Qt5Charts_LIBRARIES} )


add_definitions( ${Qt5Widgets_DEFINITIONS} )
add_definitions( ${Qt5Qml_DEFINITIONS} )
add_definitions( ${Qt5Quick_DEFINITIONS} )
add_definitions( ${Qt5Charts_DEFINITIONS} )

set(MOC_FILES
    include/calibgui.h
    )

add_executable(${PROJECT_NAME}
    src/mainCalib_gui.cpp
    src/calibgui.cpp
    ${MOC_FILES}
    ${RCC_FILES}
)


target_link_libraries(${PROJECT_NAME} ${Qt5Widgets_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Qt5Qml_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Qt5Quick_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Qt5Charts_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBS})

#####


