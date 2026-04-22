#------------------------
### hidapi library import

if(${USE_INPUT_SPACEMOUSE})

  # Location set by user
  set(HIDAPI_LIB "" CACHE FILEPATH "Location of hidapi library")
  set(HIDAPI_INCLUDE "" CACHE PATH "Location of hidapi include files")

  if(APPLE)
    find_library(CORE_FOUNDATION CoreFoundation)
    if (NOT CORE_FOUNDATION)
      message(FATAL_ERROR "CoreFoundation framework not found")
    endif()

    find_library(IO_KIT IOKit)
    if (NOT IO_KIT)
      message(FATAL_ERROR "IOKit framework not found")
    endif()
  endif(APPLE)

  add_library(hidapi STATIC IMPORTED)
  set_property(TARGET hidapi PROPERTY IMPORTED_LOCATION ${HIDAPI_LIB})
  set_property(TARGET hidapi PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${HIDAPI_INCLUDE})
  set_property(TARGET hidapi PROPERTY INTERFACE_LINK_LIBRARIES ${CORE_FOUNDATION} ${IO_KIT})

  set(SPACEMOUSE_SOURCE SpaceMouse.cpp SpaceMouseInput.cpp)
  set(SPACEMOUSE_HEADERS SpaceMouse.h SpaceMouseInput.h)

else()

  set(SPACEMOUSE_SOURCE )
  set(SPACEMOUSE_HEADERS )

endif()

#-------------------------
### ExternalInputs library

if(${USE_INPUT_IGTL})
  set(IGTL_SOURCE IGTLInput.cpp)
  set(IGTL_HEADERS IGTLInput.h)
else()
  set(IGTL_SOURCE )
  set(IGTL_HEADERS )
endif()

set(SOURCE ExternalInputs.cpp ${IGTL_SOURCE} Joystick.cpp JoystickInput.cpp ${SPACEMOUSE_SOURCE} SurfacePointsData.cpp SurfacePointsInput.cpp ControlPanel.cpp ControlPanelInput.cpp RLInput.cpp)
set(PUBLIC_HEADERS ExternalInputs.h ${IGTL_HEADERS} Joystick.h JoystickInput.h ${SPACEMOUSE_HEADERS} SurfacePointsData.h SurfacePointsInput.h ControlPanel.h ControlPanelInput.h RLInput.h)
set(HEADERS ${PUBLIC_HEADERS})

if(${USE_INPUT_IGTL})
  find_package(IGTLControllerSimple REQUIRED)
endif()

find_package(TransMatrix CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core Network SerialPort Widgets REQUIRED)

add_library(ExternalInputs ${SOURCE} ${HEADERS})

target_include_directories(ExternalInputs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(ExternalInputs PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(ExternalInputs PUBLIC TransMatrix::TransMatrix)
target_link_libraries(ExternalInputs PUBLIC Qt6::Core Qt6::Network Qt6::SerialPort Qt6::Widgets)

if(${USE_INPUT_SPACEMOUSE})
target_link_libraries(ExternalInputs PUBLIC hidapi)
endif()

if(${USE_INPUT_IGTL})
  target_link_libraries(ExternalInputs PUBLIC IGTLControllerSimple::IGTLControllerSimple)
endif()

target_compile_options(ExternalInputs PRIVATE
  $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>: -Wall -Wextra -pedantic-errors>
  $<$<CXX_COMPILER_ID:MSVC>: /W4>
)

#---------------
### Installation

install(TARGETS ExternalInputs
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/ExternalInputs
)
