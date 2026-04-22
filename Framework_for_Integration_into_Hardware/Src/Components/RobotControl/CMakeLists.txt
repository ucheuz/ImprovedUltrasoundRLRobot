#-----------------------
### RobotControl library

set(SOURCE RobotControl.cpp)
set(PUBLIC_HEADERS RobotControl.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(XmlUtils CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core REQUIRED)

add_library(RobotControl ${SOURCE} ${HEADERS})

target_include_directories(RobotControl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(RobotControl PUBLIC ExternalInputs MotorControl RobotPoseModel ScanSpace SensorControl)
target_link_libraries(RobotControl PUBLIC XmlUtils::XmlUtils)
target_link_libraries(RobotControl PUBLIC Qt6::Core)

#---------------
### Installation

install(TARGETS RobotControl
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/RobotControl
)
