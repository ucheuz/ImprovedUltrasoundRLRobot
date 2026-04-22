cmake_minimum_required(VERSION 3.12...4.0)

project(USRobotControl)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Lib)

cmake_policy(SET CMP0053 NEW)

if(APPLE)
    set(GUI_EXECUTABLE MACOSX_BUNDLE)
endif(APPLE)

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE -MP)
endif(MSVC)

set(CMAKE_CXX_STANDARD 20)

set(VTK_VERSION 9.4)
set(QT_VERSION 6.9)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(USE_INPUT_IGTL "Use OpenIGTLink as an external input option" ON)
option(USE_INPUT_SPACEMOUSE "Use SpaceMouse 6DOF joystick as an external input option" ON)

#------------
### Libraries

add_subdirectory(Components)

#--------------
### Executables

add_subdirectory(Application)

#---------------
### Installation

set(ConfigPackageLocation lib/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${ConfigPackageLocation}
)

install(FILES CMake/${PROJECT_NAME}Config.cmake
  CONFIGURATIONS Release
  DESTINATION ${ConfigPackageLocation}
  COMPONENT development
)
#install(FILES robotInfo.xml
#  CONFIGURATION Release
#  DESTINATION bin
#)

#--------------------
### Build tree export

export(EXPORT ${PROJECT_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
)
configure_file(CMake/${PROJECT_NAME}Config.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  COPYONLY
)
if (CMAKE_CONFIGURATION_TYPES)
  configure_file(robotInfo.xml
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../../Resources/robotInfo.xml"
    )
else()
  configure_file(robotInfo.xml
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../../../Resources/robotInfo.xml"
  )
endif()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.h.in"
  "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}Config.h"
)
