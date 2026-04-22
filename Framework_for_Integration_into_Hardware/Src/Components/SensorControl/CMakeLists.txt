#------------------------
### SensorControl library

set(SOURCE ArduinoSensors.cpp CalibratedSensor.cpp ISensorControl.cpp SensorControl_viaArduino.cpp)
set(PUBLIC_HEADERS ArduinoSensors.h CalibratedSensor.h ISensorControl.h SensorControl_viaArduino.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(XmlUtils CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core SerialPort REQUIRED)

add_library(SensorControl ${SOURCE} ${HEADERS})

target_include_directories(SensorControl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(SensorControl PUBLIC XmlUtils::XmlUtils)
target_link_libraries(SensorControl PUBLIC Qt6::Core Qt6::SerialPort)

target_compile_options(SensorControl PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra -pedantic-errors>
  $<$<CXX_COMPILER_ID:MSVC>: /W4>
)

#---------------
### Installation

install(TARGETS SensorControl
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/SensorControl
)
