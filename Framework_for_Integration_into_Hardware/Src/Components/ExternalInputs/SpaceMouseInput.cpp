#include <iostream>

#include "SpaceMouseInput.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SpaceMouseInput::SpaceMouseInput(QObject* parent) : QObject(parent)
{
    _spaceMouseThread = nullptr;
    _spaceMouse = nullptr;

    _status = DISCONNECTED;

    // Timer event causes a new target position to be requested
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &SpaceMouseInput::updateTargetPose);
    _timer->start(10);

    qRegisterMetaType<std::vector<double> >("std::vector<double>");
    qRegisterMetaType<TRTrans3D>("TRTrans3D");
}

SpaceMouseInput::~SpaceMouseInput()
{
    disconnectSpaceMouse();

    disconnect(_timer, nullptr, this, nullptr);
    delete _timer;
}

//----------------------------------------------------------------------
// Connect SpaceMouse
//----------------------------------------------------------------------
void SpaceMouseInput::connectSpaceMouse()
{
    disconnectSpaceMouse();

    _spaceMouseThread = new QThread();
    _spaceMouse = new SpaceMouse();

    bool success = _spaceMouse->initialise();

    if (!success) {
        std::cerr << "Failed to connect to SpaceMouse" << std::endl;
        delete _spaceMouse;
        delete _spaceMouseThread;
    }
    else {
        _spaceMouse->moveToThread(_spaceMouseThread);

        // Thread signals
        connect(_spaceMouseThread, &QThread::started, _spaceMouse, &SpaceMouse::run);
        connect(this, &SpaceMouseInput::stopSpaceMouseProcess, _spaceMouse, &SpaceMouse::stop);
        connect(_spaceMouse, &SpaceMouse::finished, _spaceMouseThread, &QThread::quit);
        connect(_spaceMouse, &SpaceMouse::finished, _spaceMouse, &SpaceMouse::deleteLater);
        connect(_spaceMouseThread, &QThread::finished, _spaceMouseThread, &QThread::deleteLater);

        _spaceMouseThread->start();
    }

    if (_spaceMouse != nullptr) {
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
// Get SpaceMouse connection status
//----------------------------------------------------------------------
bool SpaceMouseInput::isConnected() const
{
    return _spaceMouse != nullptr;
}

//----------------------------------------------------------------------
// Access pose values
// Return value indicates whether the pose is valid
//----------------------------------------------------------------------
bool SpaceMouseInput::getCurrentPoseData(std::vector<double>& poseData) const
{
    if (_spaceMouse == nullptr)
        return false;
    else {
        bool valid = _spaceMouse->getPoseData(poseData);

        return valid;
    }
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the SpaceMouse and clear variables ready to reconnect
//----------------------------------------------------------------------
void SpaceMouseInput::disconnectSpaceMouse()
{
    // Signal SpaceMouse to disconnect
    emit stopSpaceMouseProcess();

    // Wait for SpaceMouse to disconnect fully so it's safe to reconnect if needed
    while (_spaceMouse != nullptr)
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
void SpaceMouseInput::updateTargetPose()
{
    if (_spaceMouse == nullptr)
        return;

    TRTrans3D pose;

    std::vector<double> poseData;
    bool valid = getCurrentPoseData(poseData);
    if (!valid)
        return;

    TRTrans3D poseAdjust;
    double translateScale = 0.005;
    double rotateScale = 0.005;
    poseAdjust.setParams(poseData[0]*translateScale, poseData[1]*translateScale, poseData[2]*translateScale,
            poseData[3]*rotateScale, poseData[4]*rotateScale, poseData[5]*rotateScale);

    emit poseAdjustRequest(poseAdjust, false);  // Adjustment to be applied in global coordinates (with rotation about probe origin)
}
