#include <iostream>
#ifdef WIN32
  #include "windows.h"  // For performance counter, because chrono high-def clock is not precise in Windows
#else
  #include <chrono>
#endif

#include "RobotControl.h"

#include "MotorControl/MotorControl_accelStepper.h"
#include "MotorControl/MotorControl_simulator.h"
#include "SensorControl/SensorControl_viaArduino.h"

#include <QDomDocument>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
RobotControl::RobotControl(QObject *parent, RobotVer_t robotVersion, bool simulationMode) : QObject(parent)
{
    _currentPoseModel = nullptr;
    _targetPoseModel = nullptr;
    _motorControl = nullptr;
    _sensorControl = nullptr;
    _scanSpace = nullptr;
    _externalInputs = nullptr;

    _currentPositionUpdateTimer = nullptr;
    _targetTrackingTimer = nullptr;

    _controlMode = FREE_CONTROL;
    _targetTracking = false;

    _currentPoseModel = new RobotPoseModel(robotVersion);
    _targetPoseModel = new RobotPoseModel(robotVersion);
    _linkData = new LinkData();

    _requestedPose.resize(_currentPoseModel->getNProbes());
    _activeProbes.resize(4, false);
    for (int i = 0; i < _currentPoseModel->getNProbes(); i++)
        _activeProbes[i] = true;
    _nActiveProbes = _currentPoseModel->getNProbes();

    _movementPending = false;

    switch (robotVersion) {
    case V1_ROBOT:
        _robotVersion = V1_ROBOT;
        _robotId = "1.0";
        std::cerr << "Using robot V1.0" << std::endl;
        break;

    case V2_ROBOT:
        _robotVersion = V2_ROBOT;
        _robotId = "2.2";
        std::cerr << "Using robot V2.2" << std::endl;
        break;

    case V3P3_ROBOT:
        _robotVersion = V3P3_ROBOT;
        _robotId = "3.3";
        std::cerr << "Using robot V3.3" << std::endl;
        break;

    case V3P4_ROBOT:
        _robotVersion = V3P4_ROBOT;
        _robotId = "3.4";
        std::cerr << "Using robot V3.4" << std::endl;
        break;

    default:
        _robotVersion = UNKNOWN_ROBOT;
        std::cerr << "Unknown robot version" << std::endl;
        break;
    }

    if (simulationMode) {
        _motorControl = new MotorControl_simulator(_robotId);
    }
    else {
        _motorControl = new MotorControl_accelStepper(_robotId);
    }
    _sensorControl = new SensorControl_viaArduino(_robotId);

    _linkData->initialise(_currentPoseModel->getNLinks(), _robotId.toStdString());

    // Forward signals for updated connection status
    connect(_motorControl, &IMotorControl::connectionStatus, this, &RobotControl::motorConnectionStatus);
    connect(dynamic_cast<SensorControl_viaArduino*>(_sensorControl), &SensorControl_viaArduino::connectionStatus, this, &RobotControl::sensorConnectionStatus);
    connect(_motorControl, &IMotorControl::cancelSequence, this, &RobotControl::setCancelSequence);

    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", _robotId);

    // Conversion between joint positions in motor control order (used by MotorsWindow)
    // and joint positions in robot order (used by Kinematics)
    xmlParseSuccess &= XmlUtils::readValFromElement(robotEl, "jointToPort", _jointToPort);
    xmlParseSuccess &= _jointToPort.size() == _currentPoseModel->getNJoints();
    xmlParseSuccess &= XmlUtils::readValFromElement(robotEl, "portToJoint", _portToJoint);
    xmlParseSuccess &= _portToJoint.size() == _motorControl->getNMotors();

    if (!xmlParseSuccess) {
        _robotVersion = UNKNOWN_ROBOT;
        std::cerr << "Robot info file missing." << std::endl;
    }

    if (_robotVersion == UNKNOWN_ROBOT) {
        std::cerr << "    Error intialising position control." << std::endl;
        std::cerr << "    Unknown robot configuration." << std::endl;
        _validRobotConfig = false;
        return;
    }
    std::cerr << std::endl;

    _currentPoseModel->setLinkData(_linkData);
    _targetPoseModel->setLinkData(_linkData);

    std::vector<double> neutralJointPositions_kinematicsOrder, neutralJointPositions_portOrder;
    _currentPoseModel->getJointPositions(neutralJointPositions_kinematicsOrder);
    neutralJointPositions_portOrder.resize(_motorControl->getNMotors(), 0.0);
    for (int i = 0; i < _motorControl->getNMotors(); i++) {
        if (_portToJoint[i] >= 0) {
            int idx = _portToJoint[i];
            neutralJointPositions_portOrder[i] = neutralJointPositions_kinematicsOrder[idx];
        }
        else
            neutralJointPositions_portOrder[i] = 0.0;
    }
    _motorControl->setNeutralPositions(neutralJointPositions_portOrder);

    // Target tracking thread
    /*_targetTrackingThread = new QThread();
    _targetTracking = new TargetTracking(_motorControl, _sensorControl, _scanSpace, _currentPoseModel, _targetPoseModel);

    _targetTracking->moveToThread(_targetTrackingThread);

    // Thread signals
    connect(_targetTrackingThread, &QThread::started, _targetTracking, &TargetTracking::run);
    connect(this, &RobotControl::targetTrackingStopProcess, _targetTracking, &TargetTracking::stop);
    connect(_targetTracking, &TargetTracking::finished, _targetTrackingThread, &QThread::quit);
    connect(_targetTracking, &TargetTracking::finished, _targetTracking, &TargetTracking::deleteLater);
    connect(_targetTrackingThread, &QThread::finished, _targetTrackingThread, &QThread::deleteLater);
    _targetTrackingThread->start();*/

    // Centre pose
    std::vector<TRTrans3D> neutralPose;
    _currentPoseModel->getProbeNeutralPoses(neutralPose);
    TRTrans3D centrePose;
    centrePose.setBasisVectors(Point3D(1,0,0), Point3D(0,0,-1));
    Point3D neutralPosition, averagePosition;
    averagePosition.setZero();
    for (unsigned int i = 0; i < neutralPose.size(); i++) {
        neutralPose[i].getTranslation(neutralPosition);
        averagePosition += neutralPosition;
    }

    averagePosition /= double(neutralPose.size());
    centrePose.setTranslation(averagePosition);
    centrePose.printMatrix();

    // External inputs
    _externalInputs = new ExternalInputs();
    _externalInputs->selectInput(ExternalInputs::NONE);
    _externalInputs->setNeutralPoses(neutralPose);
    _externalInputs->setCentrePose(centrePose);

    connect(_externalInputs, &ExternalInputs::connectionStatus, this, &RobotControl::externalInputsConnectionStatus);
    connect(_externalInputs, &ExternalInputs::singleProbePoseRequest, this, &RobotControl::setSingleProbeTarget);
    connect(_externalInputs, &ExternalInputs::singleProbeRelativePoseRequest, this, &RobotControl::setSingleProbeRelativeTarget);
    connect(_externalInputs, &ExternalInputs::multiProbePoseRequest, this, &RobotControl::setMultiProbeTarget);
    connect(_externalInputs, &ExternalInputs::multiProbeRelativePoseRequest, this, &RobotControl::setMultiProbeRelativeTarget);
    connect(_externalInputs, &ExternalInputs::poseAdjustRequest, this, &RobotControl::setPoseOffsetTarget);
    connect(_externalInputs, &ExternalInputs::setActiveProbe, this, &RobotControl::setActiveProbe);
    connect(_externalInputs, &ExternalInputs::cancel, this, &RobotControl::cancelMove);
    connect(_externalInputs, &ExternalInputs::jointAnglesFromRL, this, &RobotControl::setJointAnglesFromRL);

    connect(_currentPoseModel, &RobotPoseModel::robotPoseUpdated, this, &RobotControl::robotPoseUpdated);  // Just forward the signal

    _requestedPose = neutralPose;

    // Abdomen surface model
    _scanSpace = new ScanSpace();
    _scanSpace->setSurfacePosition(averagePosition);

    connect(_scanSpace, &ScanSpace::surfaceMeshChanged, this, &RobotControl::surfaceMeshChanged);
    connect(_scanSpace, &ScanSpace::surfaceMeshStatus, this, &RobotControl::surfaceMeshStatus);

    _useJointLimits = true;
    _useCollisionDetection = false;

    _validRobotConfig = true;

    _cancelSequence = false;

    // Timer event causes _currentPosition to update probe pose
    _currentPositionUpdateTimer = new QTimer(this);
    connect(_currentPositionUpdateTimer, &QTimer::timeout, this, &RobotControl::updateCurrentPosition);
    _currentPositionUpdateTimer->start(50);

    // Timer event causes _motors to track the target position
    _targetTrackingTimer = new QTimer(this);
    //connect(_targetTrackingTimer, &QTimer::timeout, this, &RobotControl::trackTarget);
    _targetTrackingTimer->start(100);  // Faster rates (e.g. 50) seem to prevent the Teensy from replying to status requests after a few seconds
}

RobotControl::~RobotControl()
{
    // Signal target tracking to stop
    /*emit targetTrackingStopProcess();
    while (_targetTracking != nullptr)
        qApp->processEvents();*/

    disconnect(_motorControl, nullptr, this, nullptr);
    disconnect(_sensorControl, nullptr, this, nullptr);

    disconnect(_currentPoseModel, nullptr, this, nullptr);
    if (_externalInputs != nullptr)
        disconnect(_externalInputs, nullptr, this, nullptr);
    if (_scanSpace != nullptr)
        disconnect(_scanSpace, nullptr, this, nullptr);
    if (_currentPositionUpdateTimer != nullptr)
        disconnect(_currentPositionUpdateTimer, nullptr, this, nullptr);
    if (_targetTrackingTimer != nullptr)
        disconnect(_targetTrackingTimer, nullptr, this, nullptr);

    delete _currentPoseModel;
    delete _targetPoseModel;
    delete _motorControl;
    delete _sensorControl;
    delete _scanSpace;
    delete _externalInputs;

    delete _currentPositionUpdateTimer;
    delete _targetTrackingTimer;
}

//----------------------------------------------------------------------
// Check that RobotControl was setup with a valid robot configuration
//----------------------------------------------------------------------
bool RobotControl::isValidRobotConfig() const
{
    return _validRobotConfig;
}

//----------------------------------------------------------------------
// Connect components
//----------------------------------------------------------------------
void RobotControl::connectMotors()
{
    _motorControl->connectMotors();
}

void RobotControl::connectSensors()
{
    _sensorControl->connectSensors();
}

void RobotControl::connectExternalInputs()
{
    _externalInputs->connectInputs();
}

void RobotControl::loadSurfaceMap(const QString& filename)
{
    //_targetPoseModel->removeCollisionCheck(_scanSpace->getSurfacePort());
    _scanSpace->loadSurfaceMesh(filename);
    //_targetPoseModel->addCollisionCheck(_scanSpace->getSurfacePort());
}

//----------------------------------------------------------------------
// Control access
//----------------------------------------------------------------------
IMotorControl* RobotControl::getMotorControlPtr(RobotVer_t robotVersion)
{
    IMotorControl* pMotorControl = (robotVersion == _robotVersion) ? _motorControl : nullptr;
    return pMotorControl;
}

ISensorControl* RobotControl::getSensorControlPtr(RobotVer_t robotVersion)
{
    ISensorControl* pSensorControl = (robotVersion == _robotVersion) ? _sensorControl : nullptr;
    return pSensorControl;
}

ControlPanelInput* RobotControl::getControlPanelInputPtr() const
{
    return _externalInputs->getControlPanelInputPtr();
}

JoystickInput* RobotControl::getJoystickInputPtr() const
{
    return _externalInputs->getJoystickInputPtr();
}

#ifdef USE_INPUT_IGTL
IGTLInput* RobotControl::getIGTLInputPtr() const
{
    return _externalInputs->getIGTLInputPtr();
}
#endif  // USE_INPUT_IGTL

#ifdef USE_INPUT_SPACEMOUSE
SpaceMouseInput* RobotControl::getSpaceMouseInputPtr() const
{
    return _externalInputs->getSpaceMouseInputPtr();
}
#endif  // USE_INPUT_SPACEMOUSE

SurfacePointsInput* RobotControl::getSurfPointsInputPtr() const
{
    return _externalInputs->getSurfPointsInputPtr();
}

RLInput* RobotControl::getRLInputPtr() const
{
    return _externalInputs->getRLInputPtr();
}

//----------------------------------------------------------------------
// Access functions needed for display
//----------------------------------------------------------------------
int RobotControl::getNLinks() const
{
    return _currentPoseModel->getNLinks();
}

int RobotControl::getNProbes() const
{
    return _currentPoseModel->getNProbes();
}

vtkAlgorithmOutput* RobotControl::getProbeCurrentPort(int idx)
{
    return _currentPoseModel->getProbePort(idx);
}

vtkAlgorithmOutput* RobotControl::getProbeTargetPort(int idx)
{
    return _targetPoseModel->getProbePort(idx);
}

vtkAlgorithmOutput* RobotControl::getRobotLinkCurrentPort(int idx)
{
    return _currentPoseModel->getRobotLinkPort(idx);
}

vtkAlgorithmOutput* RobotControl::getSurfacePort()
{
    return _scanSpace->getSurfacePort();
}

//----------------------------------------------------------------------
// Apply settings from GUI
//----------------------------------------------------------------------
void RobotControl::setUseJointLimits(bool useJointLimits)
{
    _useJointLimits = useJointLimits;
}

void RobotControl::setUseCollisionDetection(bool useCollisionDetection)
{
    _useCollisionDetection = useCollisionDetection;
    _targetPoseModel->setCollisionDetection(_useCollisionDetection);
}

void RobotControl::setControlMode(ControlMode_t controlMode)
{
    _controlMode = controlMode;
}

void RobotControl::setTargetTracking(bool targetTracking)
{
    _targetTracking = targetTracking;
    if (_targetTracking)
        connect(_targetTrackingTimer, &QTimer::timeout, this, &RobotControl::trackTarget);
    else
        disconnect(_targetTrackingTimer, &QTimer::timeout, this, &RobotControl::trackTarget);
}

//----------------------------------------------------------------------
// Check whether the target robot pose is in range and not colliding
// with any defined objects
//----------------------------------------------------------------------
bool RobotControl::isTargetPoseInRange(std::vector<bool>& jointsOk) const
{
   return _targetPoseModel->isPoseInRange(jointsOk);
}

bool RobotControl::isTargetPoseColliding(std::vector<bool>& linksOk) const
{
    return _targetPoseModel->isCollision(linksOk);
}

//----------------------------------------------------------------------
// Check whether motors are currently moving
//----------------------------------------------------------------------
bool RobotControl::isIdle() const
{
    return _motorControl->isIdle() && !_movementPending;
}

//----------------------------------------------------------------------
// Select which external input source to listen to
//----------------------------------------------------------------------
void RobotControl::selectInput(ExternalInputs::Input_t input)
{
    _externalInputs->selectInput(input);
}

//----------------------------------------------------------------------
// Set which probes to move using external input source
//----------------------------------------------------------------------
void RobotControl::clearActiveProbes()
{
    for (unsigned int i = 0; i < 4; i++)
        _activeProbes[i] = false;
    _nActiveProbes = 0;
}

void RobotControl::setActiveProbe(int idx, bool isActive)
{
    if (idx < 0 || idx >= 4) {
        std::cerr << "Error in RobotControl::setActiveProbe: index is out of range." << std::endl;
        return;
    }

    std::cerr << "Probe button " << idx << " " << isActive << std::endl;

    _activeProbes[idx] = isActive;

    if (isActive)
        _nActiveProbes++;
    else
        _nActiveProbes--;
    std::cerr << "Active probes: " << _nActiveProbes << " " << _activeProbes[0] << " " << _activeProbes[1] << " " << _activeProbes[2] << " " << _activeProbes[3] << std::endl;
}

//----------------------------------------------------------------------
// Direct movement functions - mainly for kinematics test. Plain version
// of each function will calculate from the current pose and run the
// movement. Planning version will calculate from a given pose and not
// run.
//----------------------------------------------------------------------
void RobotControl::goToNeutral()
{
    if (_currentPoseModel == nullptr)
        return;

    std::vector<TRTrans3D> neutralPoses;
    _currentPoseModel->getProbeNeutralPoses(neutralPoses);
    setMultiProbeTarget(neutralPoses);
}

void RobotControl::alignToSurface()
{
    if (_currentPoseModel == nullptr)
        return;

    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planAlignToSurface(currentProbePoses, targetProbePoses);

    setMultiProbeTarget(targetProbePoses);
}

void RobotControl::planAlignToSurface(const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    for (unsigned int p = 0; p < getNProbes(); p++) {
        if (_activeProbes[p])
            _scanSpace->getNearestSurfaceAlignedPose(startingProbePoses[p], newProbePoses[p]);
    }
}

void RobotControl::rotateX(double angle)  // Elevational tilt
{
    if (_currentPoseModel == nullptr)
        return;

    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(angle, 0.0, 0.0);

    tilt(rotation);
}

void RobotControl::planRotateX(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(angle, 0.0, 0.0);

    planTilt(rotation, startingProbePoses, newProbePoses);
}


void RobotControl::rotateY(double angle)  // Axial rotation
{
    if (_currentPoseModel == nullptr)
        return;

    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(0.0, angle, 0.0);

    rotate(rotation);
}

void RobotControl::planRotateY(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(0.0, angle, 0.0);

    planRotate(rotation, startingProbePoses, newProbePoses);
}

void RobotControl::rotateZ(double angle)  // Lateral tilt
{
    if (_currentPoseModel == nullptr)
        return;

    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(0.0, 0.0, angle);

    tilt(rotation);
}

void RobotControl::planRotateZ(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D rotation;
    rotation.setIdentity();
    rotation.setRotation(0.0, 0.0, angle);

    planTilt(rotation, startingProbePoses, newProbePoses);
}

void RobotControl::surfaceFollowX(double distance)  // Lateral movement along surface
{
    if (_currentPoseModel == nullptr || _nActiveProbes < 1)
        return;

    Point3D direction(1.0, 0.0, 0.0);  // X direction

    surfaceFollow(direction, distance);
}

void RobotControl::planSurfaceFollowX(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    Point3D direction(1.0, 0.0, 0.0);  // X direction

    planSurfaceFollow(direction, distance, startingProbePoses, newProbePoses);
}

void RobotControl::surfaceFollowZ(double distance)  // Elevational movement along surface
{
    if (_currentPoseModel == nullptr || _nActiveProbes < 1)
        return;

    Point3D direction(0.0, 0.0, 1.0);  // Z direction

    surfaceFollow(direction, distance);
}

void RobotControl::planSurfaceFollowZ(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    Point3D direction(0.0, 0.0, 1.0);  // Z direction

    planSurfaceFollow(direction, distance, startingProbePoses, newProbePoses);
}

void RobotControl::moveProbeY(double distance)  // Axial movement (advance and retract)
{
    if (_currentPoseModel == nullptr)
        return;

    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planMoveProbeY(distance, currentProbePoses, targetProbePoses);

    setMultiProbeTarget(targetProbePoses);
}

void RobotControl::planMoveProbeY(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(0.0, distance, 0.0);

    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    for (unsigned int p = 0; p < getNProbes(); p++) {
        if (_activeProbes[p])
            newProbePoses[p] = startingProbePoses[p] * move;
    }
}

void RobotControl::moveX(double distance)  // X movement in base coordinates
{
    if (_currentPoseModel == nullptr || _nActiveProbes < 1)
        return;

    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(distance, 0.0, 0.0);

    baseMove(move);
}

void RobotControl::planMoveX(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(distance, 0.0, 0.0);

    planBaseMove(move, startingProbePoses, newProbePoses);
}

void RobotControl::moveY(double distance)  // Y movement in base coordinates
{
    if (_currentPoseModel == nullptr || _nActiveProbes < 1)
        return;

    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(0.0, distance, 0.0);

    baseMove(move);
}

void RobotControl::planMoveY(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(0.0, distance, 0.0);

    planBaseMove(move, startingProbePoses, newProbePoses);
}

void RobotControl::moveZ(double distance)  // Z movement in base coordinates
{
    if (_currentPoseModel == nullptr || _nActiveProbes < 1)
        return;

    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(0.0, 0.0, distance);

    baseMove(move);
}

void RobotControl::planMoveZ(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    TRTrans3D move;
    move.setIdentity();
    move.setTranslation(0.0, 0.0, distance);

    planBaseMove(move, startingProbePoses, newProbePoses);
}

void RobotControl::cancelMove()
{
    _motorControl->cancelMove();
}

//----------------------------------------------------------------------
// Development function to programme and run through a sequence of
// target points.
//----------------------------------------------------------------------
void RobotControl::getCurrentProbePoses(std::vector<TRTrans3D>& currentProbePoses)
{
    if (_currentPoseModel == nullptr)
        return;

    _currentPoseModel->getProbePoses(currentProbePoses);
}

void RobotControl::runSequence(std::vector<std::vector<TRTrans3D> >& sequence)
{
    if (_cancelSequence)  // Clear any previous cancelations
        _cancelSequence = false;

    for (unsigned int i = 0; i < sequence.size(); i++) {
        setMultiProbeTarget(sequence[i]);

        // Wait for movement step to finish
        while (!isIdle())
            qApp->processEvents();
        qApp->processEvents();

        if (_cancelSequence) {
            _cancelSequence = false;
            break;
        }
    }
}

//----------------------------------------------------------------------
// Development function to run through the whole workspace in all
// degrees of freedom.
//----------------------------------------------------------------------
void RobotControl::runThroughWorkspace()
{
    _targetPoseModel->runThroughWorkspace();
}

//----------------------------------------------------------------------
// Receive raw joint angles from rl model and send directly to motors.
// This skipped inverse kinematics.
//
// Input: jointAngles in kinematics order (22 elements for V3.4)
//        Values are absolute joint positions in degrees (or mm for
//        linear joints like the gantry).
//----------------------------------------------------------------------
void RobotControl::setJointAnglesFromRL(const std::vector<double>& jointAngles)
{
    if (_motorControl == nullptr || _targetPoseModel == nullptr)
        return;

    if (!_motorControl->isConnected())
        return;

    // Validate array size matches the kinematic model
    int nJoints = _targetPoseModel->getNJoints();
    if (static_cast<int>(jointAngles.size()) != nJoints) {
        std::cerr << "Error in RobotControl::setJointAnglesFromRL: expected "
                  << nJoints << " joint angles, received "
                  << jointAngles.size() << std::endl;
        return;
    }

    // 1. Set joint angles on the target pose model.
    //    This runs forwaed kinematics to update the
    //    internal pose representation and VTK visualisation.
    _targetPoseModel->setJointPositions(jointAngles, true);

    // 2. Safety checks — range limits
    if (_useJointLimits) {
        std::vector<bool> jointsOk;
        if (!_targetPoseModel->isPoseInRange(jointsOk)) {
            std::cerr << "RobotControl::setJointAnglesFromRL: "
                      << "Target joint angles out of range. Joints: ";
            for (unsigned int i = 0; i < jointsOk.size(); i++) {
                if (!jointsOk[i])
                    std::cerr << i << " ";
            }
            std::cerr << std::endl;
            return;  // Reject the command
        }
    }

    // 3. Safety checks — collision detection
    if (_useCollisionDetection) {
        std::vector<bool> linksOk;
        if (_targetPoseModel->isCollision(linksOk)) {
            std::cerr << "RobotControl::setJointAnglesFromRL: "
                      << "Collision detected. Rejecting command." << std::endl;
            return;  // Reject the command
        }
    }

    // 4. Convert from kinematics joint order to motor port order
    std::vector<double> targetJointPos_portOrder(_motorControl->getNMotors(), 0.0);
    for (int i = 0; i < _motorControl->getNMotors(); i++) {
        if (_portToJoint[i] >= 0) {
            int jointIdx = _portToJoint[i];
            targetJointPos_portOrder[i] = jointAngles[jointIdx];
        }
        else {
            targetJointPos_portOrder[i] = 0.0;
        }
    }

    // 5. Send to motors
    _motorControl->moveAllAbsolute(targetJointPos_portOrder);
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Set indicator to discontinue sequence
//----------------------------------------------------------------------
void RobotControl::setCancelSequence()
{
    _cancelSequence = true;
}

//----------------------------------------------------------------------
// Set a target pose which will be moved to in the target tracking timer
// event
//----------------------------------------------------------------------
void RobotControl::setSingleProbeTarget(int idx, const TRTrans3D& pose)
{
    if (idx < 0 || idx >= _currentPoseModel->getNProbes()) {
        std::cerr << "Error in RobotControl::setSingleProbeTarget: index out of range." << std::endl;
        return;
    }

    _requestedPose[idx].set(pose);

    _movementPending = true;
}

void RobotControl::setSingleProbeRelativeTarget(int idx, const TRTrans3D& relativePose)
{
    if (idx < 0 || idx >= _currentPoseModel->getNProbes()) {
        std::cerr << "Error in RobotControl::setSingleProbeRelativeTarget: index out of range." << std::endl;
        return;
    }

    std::vector<TRTrans3D> currentProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    TRTrans3D targetPose = currentProbePoses[idx] * relativePose;
    _requestedPose[idx].set(targetPose);

    _movementPending = true;
}

void RobotControl::setMultiProbeTarget(const std::vector<TRTrans3D>& poses)
{
    //if (poses.size() != _currentPoseModel->getNProbes()) {
    if (poses.size() < _currentPoseModel->getNProbes()) {
        std::cerr << "Error in RobotControl::setMultiProbeTarget: wrong array size." << std::endl;
        return;
    }

    for (unsigned int p = 0; p < _currentPoseModel->getNProbes(); p++)
        _requestedPose[p].set(poses[p]);

    _movementPending = true;
}

void RobotControl::setMultiProbeRelativeTarget(const std::vector<TRTrans3D>& relativePoses)
{
    if (relativePoses.size() != _currentPoseModel->getNProbes()) {
        std::cerr << "Error in RobotControl::setMultiProbeRelativeTarget: wrong array size." << std::endl;
        return;
    }

    std::vector<TRTrans3D> currentProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    for (unsigned int p = 0; p < _currentPoseModel->getNProbes(); p++) {
        TRTrans3D targetPose = currentProbePoses[p] * relativePoses[p];
        _requestedPose[p].set(targetPose);
    }

    _movementPending = true;
}

void RobotControl::setPoseOffsetTarget(const TRTrans3D& poseAdjust, bool isLocal)
{
    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);
    targetProbePoses.resize(currentProbePoses.size());

    if (_nActiveProbes > 1) {
        _scanSpace->calcGroupPoseAdjustPoses(currentProbePoses, poseAdjust, isLocal, targetProbePoses);
        setMultiProbeTarget(targetProbePoses);
    }
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            int pIdx = firstProbe;
            if (isLocal) {
                targetProbePoses[pIdx].set(currentProbePoses[pIdx]);
                targetProbePoses[pIdx] *= poseAdjust;  // Apply adjustment in probe coordinate system
            }
            else {
                Point3D position, newPosition;
                currentProbePoses[pIdx].getTranslation(position);

                targetProbePoses[pIdx].set(currentProbePoses[pIdx]);
                targetProbePoses[pIdx].setTranslation(0.0, 0.0, 0.0);  // Translate probe to origin, so rotation will be applied about probe origin
                targetProbePoses[pIdx] = poseAdjust * targetProbePoses[pIdx];  // Apply adjustment in global coordinate system
                targetProbePoses[pIdx].getTranslation(newPosition);
                targetProbePoses[pIdx].setTranslation(newPosition + position);  // Translate back
            }

            poseAdjust.printParams();

            setSingleProbeTarget(pIdx, targetProbePoses[pIdx]);
        }
    }
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Return the index of the first active probe
//----------------------------------------------------------------------
int RobotControl::firstActiveProbe() const
{
    if (_nActiveProbes < 1)
        return -1;

    unsigned int probeIdx = 0;
    while (!_activeProbes[probeIdx])
        probeIdx++;

    return static_cast<int>(probeIdx);
}

//----------------------------------------------------------------------
// Basic kinematics control for testing.
// Move the probe or group of probes a given distance in the given
// direction over the surface.
//----------------------------------------------------------------------
void RobotControl::surfaceFollow(const Point3D& direction, double distance)
{
    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planSurfaceFollow(direction, distance, currentProbePoses, targetProbePoses);

    if (_nActiveProbes > 1)
        setMultiProbeTarget(targetProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            setSingleProbeTarget(pIdx, targetProbePoses[pIdx]);
        }
    }
}

void RobotControl::planSurfaceFollow(const Point3D& direction, double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    if (_nActiveProbes > 1)
        _scanSpace->calcGroupSurfaceStepPoses(startingProbePoses, direction, distance, newProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            _scanSpace->calcSurfaceStepPose(startingProbePoses[pIdx], direction, distance, newProbePoses[pIdx]);
        }
    }
}

//----------------------------------------------------------------------
// Basic kinematics control for testing.
// Move the probe or group of probes by the given movement in base
// coordinates.
//----------------------------------------------------------------------
void RobotControl::baseMove(const TRTrans3D& move)
{
    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planBaseMove(move, currentProbePoses, targetProbePoses);

    setMultiProbeTarget(targetProbePoses);
}

void RobotControl::planBaseMove(const TRTrans3D& move, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    for (unsigned int p = 0; p < getNProbes(); p++) {
        if (_activeProbes[p])
            newProbePoses[p] = move * startingProbePoses[p];
        //else
        //    newProbePoses[p] = startingProbePoses[p];
    }
}

//----------------------------------------------------------------------
// Basic kinematics control for testing.
// Move the probe or group of probes in a lateral or elevational
// rotation movement.
//----------------------------------------------------------------------
void RobotControl::tilt(const TRTrans3D& rotation)
{
    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planTilt(rotation, currentProbePoses, targetProbePoses);

    if (_nActiveProbes > 1)
        setMultiProbeTarget(targetProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            setSingleProbeTarget(pIdx, targetProbePoses[pIdx]);
        }
    }
}

void RobotControl::planTilt(const TRTrans3D& rotation, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    if (_nActiveProbes > 1)
        _scanSpace->calcGroupSurfaceTiltPoses(startingProbePoses, rotation, newProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            newProbePoses[pIdx] = startingProbePoses[pIdx] * rotation;
        }
    }
}

//----------------------------------------------------------------------
// Basic kinematics control for testing.
// Move the probe or group of probes in an axial rotation movement.
//----------------------------------------------------------------------
void RobotControl::rotate(const TRTrans3D& rotation)
{
    std::vector<TRTrans3D> currentProbePoses, targetProbePoses;
    _currentPoseModel->getProbePoses(currentProbePoses);

    planRotate(rotation, currentProbePoses, targetProbePoses);

    if (_nActiveProbes > 1)
        setMultiProbeTarget(targetProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            setSingleProbeTarget(pIdx, targetProbePoses[pIdx]);
        }
    }
}

void RobotControl::planRotate(const TRTrans3D& rotation, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses)
{
    newProbePoses.resize(getNProbes());
    for (unsigned int p = 0; p < getNProbes(); p++)
        newProbePoses[p] = startingProbePoses[p];

    if (_nActiveProbes > 1)
        _scanSpace->calcGroupSurfaceRotationPoses(startingProbePoses, rotation, newProbePoses);
    else {
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int pIdx = static_cast<unsigned int>(firstProbe);
            newProbePoses[pIdx] = startingProbePoses[pIdx] * rotation;
        }
    }
}

//----------------------------------------------------------------------
// Apply safety algorithms to a target pose to give a new target pose.
// The safety algorithm is determined by _controlMode.
//----------------------------------------------------------------------
void RobotControl::sensorAdjust(std::vector<TRTrans3D>& pose, const std::vector<Point3D>& targetSurfacePoints, const std::vector<Point3D>& targetSurfaceNormals)
{
    if (_controlMode == FREE_CONTROL || _sensorControl == nullptr)
        // No sensor-based safety algorithm. Leave the pose unchanged.
        return;

    std::vector<TRTrans3D> currentPose;
    _currentPoseModel->getProbePoses(currentPose);

    for (unsigned int p = 0; p < _currentPoseModel->getNProbes(); p++) {
        if (!_activeProbes[p]) {
            pose[p].set(currentPose[p]);
            continue;
        }

        // Get the current distance to the surface model
        Point3D currentSurfacePoint, currentSurfaceNormal;
        double currentSurfaceDist;
        currentPose[p].getTranslation(currentSurfacePoint);
        _scanSpace->getNearestPointOnSurface(currentSurfacePoint, currentSurfaceNormal, currentSurfaceDist);

        switch (_controlMode) {
        case SURFACE_FOLLOW_PROXIMITY_CONTROL:
        {
            // Keep the probe normal to the surface and a fixed distance away.
            // Currently only axial distance can be measured by the distance sensors.
            double thresholdDist = 50.0;  // In mm from probe face *** This could be set through the GUI
            double kp = 3;  // Proportional control parameter - not optimised

            // Distance error
            bool isInRange;
            double dist = _sensorControl->getCurrentDistanceComponent(p, 0, isInRange);
            //double dist = currentSurfaceDist;  // Debug test without sensors
            if (!isInRange) {
                // Stay in current position
                pose[p].set(currentPose[p]);
                break;
            }

            double distError = dist - thresholdDist;
            // Sensitivity limit
            //if (fabs(distError) < 2.0)
            //    distError = 0.0;

            // Normal direction control correction
            double adjustedDist = currentSurfaceDist - kp * distError;

            // Target orientation - adjust target to be normal to surface, while keeping y-axis rotation.
            Point3D xDir, yDir, zDir;
            pose[p].getBasisVectors(xDir, yDir, zDir);
            yDir.set(-targetSurfaceNormals[p]);
            yDir.normalise();
            xDir -= yDir*(xDir.dot(yDir));
            xDir.normalise();
            pose[p].setBasisVectors(xDir, yDir);

            // Target position
            Point3D targetPosition = targetSurfacePoints[p] + targetSurfaceNormals[p] * adjustedDist;
            pose[p].setTranslation(targetPosition);

            break;
        }

        case SURFACE_FOLLOW_FORCE_CONTROL:
        {
            // Keep the probe normal to the surface and in contact with a fixed force.
            // Currently only axial force can be measured.
            double thresholdForce = 4.0;  // In N *** This could be set through the GUI
            double kp = 5.0;  // Proportional control parameter - not optimised

            // Force error
            bool isInRange;
            double axialForce = _sensorControl->getCurrentForceComponent(p, 2, isInRange);  // Axial component of probe p *** currently x direction of sensor - not correct
            axialForce *= -1;  // *** Temporrary artifical calibration of Z to Axial
            std::cerr << "Axial force: " << axialForce << std::endl;
            //double axialForce = ??;  // Debug test without sensors
            if (!isInRange) {
                // Stay in current position
                pose[p].set(currentPose[p]);
                break;
            }

            double forceError = axialForce - thresholdForce;
            // Sensitivity limit
            //if (fabs(forceError) < 1.0)
            //    forceError = 0.0;

            // Normal direction control correction
            double adjustedDist = currentSurfaceDist + kp * forceError;

            // *** Alternative compliance control - not used
            // Needs prevAdjust to be a class variable and set to zero at the start - doesn't currently exist
            //double k = 0.2, c = 0.3;
            //double adjust = (forceError + c * prevAdjust) / (k + c);
            //double adjustedDist = currentSurfaceDist + adjust;
            //prevAdjust = adjust;

            // Target orientation - adjust target to be normal to surface, while keeping y-axis rotation.
            Point3D xDir, yDir, zDir;
            pose[p].getBasisVectors(xDir, yDir, zDir);
            yDir.set(-targetSurfaceNormals[p]);
            yDir.normalise();
            xDir -= yDir*(xDir.dot(yDir));
            xDir.normalise();
            pose[p].setBasisVectors(xDir, yDir);

            // Target position
            Point3D targetPosition = targetSurfacePoints[p] + targetSurfaceNormals[p] * adjustedDist;
            pose[p].setTranslation(targetPosition);

            break;
        }

        case FIXED_POINT_FORCE_CONTROL:
        {
            // Keep the probe at a fixed location and fixed force, allowing reorientation only.
            // Requires 3D force vector.
            // *** Currently does nothing.
            break;
        }

        case UNKNOWN_CONTROL:
        default:
            // Stay in current position
            pose[p].set(currentPose[p]);
            break;
        }
    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// QTimer idle slot for reading the current motor positions, updating
// _currentPosition and sending information to external inputs
//----------------------------------------------------------------------
void RobotControl::updateCurrentPosition()
{
    if (_motorControl == nullptr || _currentPoseModel == nullptr)
        return;

    //std::cerr << "Update" << std::endl;

    std::vector<double> jointPos_portOrder;
    _motorControl->getAllMotorPositions(jointPos_portOrder);
    if (jointPos_portOrder.size() == 0)  // Motors disconnected
        return;
    else if (jointPos_portOrder.size() != _motorControl->getNMotors()) {
        jointPos_portOrder.resize(_motorControl->getNMotors(), 0.0);
        std::cerr << "Warning in RobotControl::updateCurrentPosition: received motor positions vector is not the expected size (" <<
                     _motorControl->getNMotors() << ")." << std::endl;
    }

    std::vector<double> jointPos_kinematicsOrder(_currentPoseModel->getNJoints(), 0.0);
    _currentPoseModel->getJointNeutralPositions(jointPos_kinematicsOrder);
    for (unsigned int i = 0; i < _currentPoseModel->getNJoints(); i++) {
        if (_jointToPort[i] >= 0) {
            unsigned int idx = static_cast<unsigned int>(_jointToPort[i]);
            jointPos_kinematicsOrder[i] = jointPos_portOrder[idx];
        }
    }
    _currentPoseModel->setJointPositions(jointPos_kinematicsOrder);  // This update causes the new pose to be signalled from _currentPoseModel->_kinematics
    
    _externalInputs->setCurrentJointPositions(jointPos_kinematicsOrder);

    // Send current pose to external inputs (IGTL)
    std::vector<TRTrans3D> probePoses;
    _currentPoseModel->getProbePoses(probePoses);
    _externalInputs->setCurrentPoses(probePoses);

    // Send current forces to external inputs (Joystick, IGTL and ControlPanel)
    std::vector<double> forces;
    if (_sensorControl != nullptr && _sensorControl->isConnected()) {
        bool isInRange;
        int firstProbe = firstActiveProbe();
        if (firstProbe >= 0) {
            unsigned int probeIdx = static_cast<unsigned int>(firstProbe);
            _sensorControl->getCurrentForces(probeIdx, forces, isInRange);  // *** Currently from first active probe only
            if (isInRange)
                _externalInputs->setCurrentForce(forces);
        }
    }

    // Send simulated distance value to external inputs (*** not currently used by any external input)
    std::vector<double> distances(getNProbes(), 0.0);
    std::vector<TRTrans3D> poses;
    _currentPoseModel->getProbePoses(poses);
    Point3D probePosition, normal;
    poses[0].getTranslation(probePosition);
    _scanSpace->getNearestPointOnSurface(probePosition, normal, distances[0]);
    _externalInputs->setCurrentDistances(distances);
}

//----------------------------------------------------------------------
// QTimer slot for target tracking.
// Move to the target pose in _requestedPose.
// Potentially, this could involve various safety checks (in the
// kinematics) and control algorithms that modify the target before
// executing the move.
// *** Some of these still need implementing.
// *** There may be some inefficiency in how many times surface points
//     and normals are calculated.
//----------------------------------------------------------------------
void RobotControl::trackTarget()
{
    if (!(_motorControl->isConnected()) || _targetTracking == false)
        return;

#ifdef WIN32
    LARGE_INTEGER ticksPerSecond, startTime, endTime;  // For performance counter, because chrono high-def clock is not precise in Windows
    QueryPerformanceFrequency(&ticksPerSecond);
    QueryPerformanceCounter(&startTime);
#else
    std::chrono::high_resolution_clock::time_point startTime, endTime;
    startTime = std::chrono::high_resolution_clock::now();
#endif
    double timeDiff;

    unsigned int nProbes = _currentPoseModel->getNProbes();

    //std::cerr << "Moving to:" << std::endl;
    //_requestedPose[0].printMatrix();
    //_requestedPose[0].printParams();
    std::vector<double> targetJointOffsets_kinematicsOrder;
    std::vector<double> targetJointOffsets_portOrder(_motorControl->getNMotors(), 0.0);

    //std::vector<TRTrans3D> currentPose;
    //double x, y, z;
    //_currentPoseModel->getProbePoses(currentPose);
    //currentPose.getTranslation(x, y, z);

    std::vector<TRTrans3D> targetPose(_requestedPose);

    // This is needed just to get the surface normals at the target points
    Point3D targetPoint;
    std::vector<Point3D> targetSurfacePoints(nProbes), surfaceNormals(nProbes);
    double dist;
    for (unsigned int p = 0; p < nProbes; p++) {
        targetPose[p].getTranslation(targetPoint);
        targetSurfacePoints[p] = _scanSpace->getNearestPointOnSurface(targetPoint, surfaceNormals[p], dist);
    }

    // Possibly apply control algorithm adjustment to target pose
    sensorAdjust(targetPose, targetSurfacePoints, surfaceNormals);

    // Inverse kinematics calculation happens here
    std::vector<double> currentJointPositions;
    _currentPoseModel->getJointPositions(currentJointPositions);
    _targetPoseModel->setJointPositions(currentJointPositions, false);
    _targetPoseModel->setProbePoses(targetPose, surfaceNormals, false);

    // Extra checks on whether target pose is reachable
    if (!_targetPoseModel->isPoseInRange()) {
        std::cerr << "Target pose out of range" << std::endl;
        if (_useJointLimits)
            return;
    }

    if (_targetPoseModel->isCollision()) {
        std::cerr << "Target pose collision detected" << std::endl;
        //if (_useCollisionDetection)
        //    return;
    }

    _targetPoseModel->getJointPositions(targetJointOffsets_kinematicsOrder);
    // Reorder from kinematics to port order
    for (int i = 0; i < _motorControl->getNMotors(); i++) {
        if (_portToJoint[i] >= 0) {
            int idx = _portToJoint[i];
            targetJointOffsets_portOrder[i] = targetJointOffsets_kinematicsOrder[idx];
        }
        else
            targetJointOffsets_portOrder[i] = 0.0;
        //std::cerr << targetJointOffsets_portOrder[i] << ", ";
    }
    //std::cerr << std::endl;
    if (_motorControl != nullptr)
        _motorControl->moveAllAbsolute(targetJointOffsets_portOrder);
    _movementPending = false;

#ifdef WIN32
    QueryPerformanceCounter(&endTime);
    timeDiff = double(endTime.QuadPart - startTime.QuadPart) / double(ticksPerSecond.QuadPart);
#else
    endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeDuration = endTime - startTime;
    timeDiff = timeDuration.count();
#endif
    //std::cerr << "Target calculation took " << timeDiff << " seconds" << std::endl;
}
