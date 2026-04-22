#include <iostream>

#include "JoystickInput.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
JoystickInput::JoystickInput(QObject* parent) : QObject(parent)
{
    _joystickThread = nullptr;
    _joystick = nullptr;

    _controlledProbe = 0;  // *** Needs to be adjustable for full control

    _status = DISCONNECTED;

    // Timer event causes a new target position to be requested
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &JoystickInput::updateTargetPose);
    //_timer->start(200);
    _timer->start(0);

    qRegisterMetaType<std::vector<double> >("std::vector<double>");
    qRegisterMetaType<TRTrans3D>("TRTrans3D");
}

JoystickInput::~JoystickInput()
{
    disconnectJoystick();

    disconnect(_timer, nullptr, this, nullptr);
    delete _timer;
}

//----------------------------------------------------------------------
// Connect joystick
//----------------------------------------------------------------------
void JoystickInput::connectJoystick()
{
    disconnectJoystick();

    _joystickThread = new QThread();
    _joystick = new Joystick();

    bool success = _joystick->initialise(_joystickIpAddress, _receivePort);

    if (!success) {
        std::cerr << "Failed to create UDP socket for Joystick" << std::endl;
        delete _joystick;
        delete _joystickThread;
    }
    else {
        _joystick->moveToThread(_joystickThread);

        // Thread signals
        connect(_joystickThread, &QThread::started, _joystick, &Joystick::run);
        connect(this, &JoystickInput::stopJoystickProcess, _joystick, &Joystick::stop);
        connect(_joystick, &Joystick::finished, _joystickThread, &QThread::quit);
        connect(_joystick, &Joystick::finished, _joystick, &Joystick::deleteLater);
        connect(_joystickThread, &QThread::finished, _joystickThread, &QThread::deleteLater);

        _joystickThread->start();

        connect(this, &JoystickInput::forceData, _joystick, &Joystick::updateForce);
    }

    if (_joystick != nullptr) {
        _status = CONNECTED;
        emit connectionStatus("Port open");
    }
    else {
        _status = CONNECT_FAILED;
        emit connectionStatus("Connect failed");
    }

    //emit connectionChanged();
}

//----------------------------------------------------------------------
// Get joystick connection status
//----------------------------------------------------------------------
bool JoystickInput::isConnected() const
{
    return _joystick != nullptr;
}

//----------------------------------------------------------------------
// Set probe neutral poses as a reference for new pose calculations
//----------------------------------------------------------------------
void JoystickInput::setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses)
{
    _neutralPoses = neutralPoses;
}

void JoystickInput::setIpAddress(const QHostAddress& address)
{
    _joystickIpAddress = address;
    connectJoystick();
}

void JoystickInput::setPort(unsigned short port)
{
    _receivePort = port;
    connectJoystick();
}

//----------------------------------------------------------------------
// Access pose values
// Return value indicates whether the pose is valid
//----------------------------------------------------------------------
bool JoystickInput::getCurrentPoseData(std::vector<double>& poseData) const
{
    if (_joystick == nullptr)
        return false;
    else {
        bool valid = _joystick->getPoseData(poseData);
        return valid;
    }
}

//----------------------------------------------------------------------
// Send the current force vector to the Joystick for force feedback
//----------------------------------------------------------------------
void JoystickInput::setCurrentForce(const std::vector<double>& force)
{
    emit forceData(force);  // Send to Joystick class
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the joystick and clear variables ready to reconnect
//----------------------------------------------------------------------
void JoystickInput::disconnectJoystick()
{
    // Signal joystick to disconnect
    emit stopJoystickProcess();

    // Wait for joystick to disconnect fully so it's safe to reconnect if needed
    while (_joystick != nullptr)
        qApp->processEvents();

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    //emit connectionChanged();
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// QTimer slot to send a target pose request
//----------------------------------------------------------------------
void JoystickInput::updateTargetPose()
{
    if (_joystick == nullptr)
        return;

    TRTrans3D pose;

    std::vector<double> poseData;
    bool valid = getCurrentPoseData(poseData);
    if (!valid)
        return;

    pose.setRotation(poseData[3], poseData[4], poseData[5], poseData[6]);
    pose.setTranslation(poseData[0], poseData[1], poseData[2]);

    // T_probeToBase = T_P0ToBase * T_J0ToP0 * T_JcToJ0 * T_PcToJc
    // Convert from (X right, Y up, Z forward) to (X left, Y down, Z forward)
    TRTrans3D joystickCal;
    joystickCal.setIdentity();
    joystickCal.setRotation(-90.0, 0.0, 180.0);  // Probe pose to joystick pose
    TRTrans3D J0ToP0;
    J0ToP0.setIdentity();
    J0ToP0.setRotation(0.0, 0.0, 180.0);
    pose = _neutralPoses[_controlledProbe] * J0ToP0 * pose * joystickCal;

    emit singleProbePoseRequest(_controlledProbe, pose);
}
