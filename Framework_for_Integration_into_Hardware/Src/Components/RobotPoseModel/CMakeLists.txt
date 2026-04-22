#-------------------------
### RobotPoseModel library

set(SOURCE GJKCollisionDetection.cpp LinkData.cpp RobotPoseModel.cpp)
set(PUBLIC_HEADERS GJKCollisionDetection.h LinkData.h RobotPoseModel.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(TransMatrix CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core REQUIRED)
find_package(VTK ${VTK_VERSION} COMPONENTS
  CommonCore CommonDataModel CommonTransforms
  FiltersCore FiltersGeneral FiltersSources
  IOXML
  CONFIG REQUIRED
)

add_library(RobotPoseModel ${SOURCE} ${HEADERS})

vtk_module_autoinit(TARGETS RobotPoseModel MODULES
  VTK::CommonCore VTK::CommonDataModel VTK::CommonTransforms
  VTK::FiltersCore VTK::FiltersGeneral VTK::FiltersSources
  VTK::IOXML
)

target_include_directories(RobotPoseModel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(RobotPoseModel PUBLIC Kinematics)
target_link_libraries(RobotPoseModel PUBLIC TransMatrix::TransMatrix)
target_link_libraries(RobotPoseModel PUBLIC Qt6::Core)
target_link_libraries(RobotPoseModel PUBLIC
  VTK::CommonCore VTK::CommonDataModel VTK::CommonTransforms
  VTK::FiltersCore VTK::FiltersGeneral VTK::FiltersSources
  VTK::IOXML
)

target_compile_options(RobotPoseModel PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra -pedantic-errors>
  $<$<CXX_COMPILER_ID:MSVC>: /W4>
)

#---------------
### Installation

install(TARGETS RobotPoseModel
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/RobotPoseModel
)
