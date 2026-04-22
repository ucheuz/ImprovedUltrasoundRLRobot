#---------------------
### Kinematics library

set(SOURCE IKinematics.cpp Kinematics_1p0.cpp Kinematics_2p2.cpp Kinematics_3p3.cpp Kinematics_3p4.cpp)
set(PUBLIC_HEADERS IKinematics.h Kinematics_1p0.h Kinematics_2p2.h Kinematics_3p3.h Kinematics_3p4.h)
set(HEADERS ${PUBLIC_HEADERS})

find_package(TransMatrix CONFIG REQUIRED)
find_package(DHKinematics CONFIG REQUIRED)
find_package(XmlUtils CONFIG REQUIRED)
find_package(Qt6 ${QT_VERSION} COMPONENTS Core REQUIRED)

add_library(Kinematics ${SOURCE} ${HEADERS})

target_include_directories(Kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Components>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(Kinematics PUBLIC TransMatrix::TransMatrix)
target_link_libraries(Kinematics PUBLIC DHKinematics::DHKinematics)
target_link_libraries(Kinematics PUBLIC XmlUtils::XmlUtils)
target_link_libraries(Kinematics PUBLIC Qt6::Core)

#---------------
### Installation

install(TARGETS Kinematics
  EXPORT ${PROJECT_NAME}Targets
  CONFIGURATIONS Release
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PUBLIC_HEADERS}
  CONFIGURATIONS Release
  DESTINATION include/${PROJECT_NAME}/Kinematics
)
