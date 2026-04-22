#-----------------------
### MotorControl library

set(SOURCE AccelStepperMotors.cpp IMotorControl.cpp MotorControl_accelStepper.cpp MotorControl_simulator.cpp SimulatedMotors.cpp)
set(PUBLIC_HEADERS AccelStepperMotors.h IMotorControl.h MotorControl_accelStepper.h MotorControl_simulator.h SimulatedMotors.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(XmlUtils CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core SerialPort REQUIRED)

add_library(MotorControl ${SOURCE} ${HEADERS})

target_include_directories(MotorControl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(MotorControl PUBLIC XmlUtils::XmlUtils)
target_link_libraries(MotorControl PUBLIC Qt6::Core Qt6::SerialPort)

#---------------
### Installation

install(TARGETS MotorControl
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/MotorControl
)
