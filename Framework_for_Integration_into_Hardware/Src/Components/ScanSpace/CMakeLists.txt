#--------------------
### ScanSpace library

set(SOURCE ScanSpace.cpp)
set(PUBLIC_HEADERS ScanSpace.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(TransMatrix CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core REQUIRED)
find_package(VTK ${VTK_VERSION} COMPONENTS
  CommonCore CommonDataModel CommonMath CommonTransforms
  FiltersCore FiltersGeneral
  IOXML
  CONFIG REQUIRED
)

add_library(ScanSpace ${SOURCE} ${HEADERS})

vtk_module_autoinit(TARGETS ScanSpace MODULES
  VTK::CommonCore VTK::CommonDataModel VTK::CommonMath VTK::CommonTransforms
  VTK::FiltersCore VTK::FiltersGeneral
  VTK::IOXML
)

target_include_directories(ScanSpace PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(ScanSpace PUBLIC TransMatrix::TransMatrix)
target_link_libraries(ScanSpace PUBLIC Qt6::Core)
target_link_libraries(ScanSpace PUBLIC
  VTK::CommonCore VTK::CommonDataModel VTK::CommonMath VTK::CommonTransforms
  VTK::FiltersCore VTK::FiltersGeneral
  VTK::IOXML
)

target_compile_options(ScanSpace PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra -pedantic-errors>
  $<$<CXX_COMPILER_ID:MSVC>: /W4>
)

#---------------
### Installation

install(TARGETS ScanSpace
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/ScanSpace
)

