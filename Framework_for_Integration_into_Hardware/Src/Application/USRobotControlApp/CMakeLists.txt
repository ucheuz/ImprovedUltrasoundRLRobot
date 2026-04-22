#-------------------------------
### USRobotControlApp executable

set(SOURCE App.cpp InputsWindow.cpp MainWindow.cpp MotorsWindow.cpp SensorsWindow.cpp StartupDialog.cpp)
set(HEADERS InputsWindow.h MainWindow.h MotorsWindow.h SensorsWindow.h StartupDialog.h)
set(UI_FILES InputsWindow.ui MainWindow.ui MotorsWindow.ui SensorsWindow.ui StartupDialog.ui)

find_package(VTK ${VTK_VERSION} COMPONENTS
  CommonCore CommonDataModel
  FiltersSources
  GUISupportQt
  IOImage IOXML
  RenderingCore RenderingOpenGL2
  CONFIG REQUIRED
)

find_package(Qt6 ${QT_VERSION} COMPONENTS Core Gui Widgets REQUIRED)

add_executable(USRobotControlApp ${GUI_EXECUTABLE} ${SOURCE} ${HEADERS} ${UI_FILES})

vtk_module_autoinit(TARGETS USRobotControlApp MODULES
  VTK::CommonCore VTK::CommonDataModel
  VTK::FiltersSources
  VTK::GUISupportQt
  VTK::IOImage VTK::IOXML
  VTK::RenderingCore VTK::RenderingOpenGL2
)

target_include_directories(USRobotControlApp PRIVATE
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(USRobotControlApp PRIVATE MotorControl RobotControl SensorControl)
target_link_libraries(USRobotControlApp PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets)
target_link_libraries(USRobotControlApp PRIVATE
  VTK::CommonCore VTK::CommonDataModel
  VTK::FiltersSources
  VTK::GUISupportQt
  VTK::IOImage VTK::IOXML
  VTK::RenderingCore VTK::RenderingOpenGL2
)

#---------------
### Installation

install(TARGETS USRobotControlApp
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  BUNDLE DESTINATION bin
  RUNTIME DESTINATION bin
)
