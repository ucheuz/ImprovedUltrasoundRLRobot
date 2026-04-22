include(CMakeFindDependencyMacro)
find_dependency(Qt6 6.9)
find_dependency(VTK 9.4)
find_dependency(IGTLControllerSimple)
find_dependency(TransMatrix)
find_dependency(DHKinematics)
find_dependency(XmlUtils)

include("${CMAKE_CURRENT_LIST_DIR}/USRobotControlTargets.cmake")
