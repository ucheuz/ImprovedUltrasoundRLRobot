#include <iostream>
#include "ControlPanelInput.h"

#include <QDebug>

static const double pi = std::atan2(1.0, 1.0) * 4.0;

ControlPanelInput::ControlPanelInput(QObject* parent) : QObject(parent)
{
    _controlPanelThread = nullptr;
    _controlPanel = nullptr;

    _status = DISCONNECTED;

    // Timer event causes a new target position to be requested
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &ControlPanelInput::updateTargetPose);
    _timer->start(200);
    //_timer->start(0);
}

ControlPanelInput::~ControlPanelInput()
{
    disconnectControlPanel();

    disconnect(_timer, nullptr, this, nullptr);
    delete _timer;
}

//----------------------------------------------------------------------
// Connect control panel
//----------------------------------------------------------------------
void ControlPanelInput::connectControlPanel()
{
    disconnectControlPanel();

    _controlPanelThread = new QThread();
    _controlPanel = new ControlPanel();

    bool success = _controlPanel->initialise();

    if (!success) {
        std::cerr << "Failed to connect to control panel" << std::endl;
        delete _controlPanel;
        delete _controlPanelThread;
    }
    else {
        _controlPanel->moveToThread(_controlPanelThread);

        // Thread signals
        connect(_controlPanelThread, &QThread::started, _controlPanel, &ControlPanel::run);
        connect(this, &ControlPanelInput::stopControlPanelProcess, _controlPanel, &ControlPanel::stop);
        connect(_controlPanel, &ControlPanel::finished, _controlPanelThread, &QThread::quit);
        connect(_controlPanel, &ControlPanel::finished, _controlPanel, &ControlPanel::deleteLater);
        connect(_controlPanelThread, &QThread::finished, _controlPanelThread, &QThread::deleteLater);

        _controlPanelThread->start();

        // Connect force data signal to control panel for sending data back to device
        connect(this, &ControlPanelInput::forceData, _controlPanel, &ControlPanel::updateForces);
    }

    if (_controlPanel != nullptr) {
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
// Get control panel connection status
//----------------------------------------------------------------------
bool ControlPanelInput::isConnected() const
{
    return _controlPanel != nullptr;
}

//----------------------------------------------------------------------
// Access pose values
// Return value indicates whether the pose is valid
//----------------------------------------------------------------------
/*bool ControlPanelInput::getCurrentPoseData(std::vector<double>& poseData) const
{
    if (_joystick == nullptr)
        return false;
    else {
        bool valid = _joystick->getPoseData(poseData);
        return valid;
    }
}*/

//----------------------------------------------------------------------
// Send the current forces to the control panel for LED indicators.
//----------------------------------------------------------------------
void ControlPanelInput::setCurrentForces(const std::vector<double>& forces)
{
    emit forceData(forces);  // Send to ControlPanel class
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the control panel and clear variables ready to reconnect
//----------------------------------------------------------------------
void ControlPanelInput::disconnectControlPanel()
{
    // Signal control panel to disconnect
    emit stopControlPanelProcess();

    // Wait for control panel to disconnect fully so it's safe to reconnect if needed
    while (_controlPanel != nullptr)
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
void ControlPanelInput::updateTargetPose()
{
    if (_controlPanel == nullptr)
        return;

    double buttonVal1, buttonVal2;
    ControlPanel::ButtonCode_t buttonCode = _controlPanel->getButtonCode(buttonVal1, buttonVal2);

    if (buttonCode != ControlPanel::BUTTON_NONE)
        qDebug() << "Button code = " << buttonCode << ", val1 = " << buttonVal1 << ", val2 = " << buttonVal2;

    TRTrans3D poseAdjust;

    switch (buttonCode) {
    // Probe selection buttons
    case ControlPanel::BUTTON_PROBE_RIGHT:
        emit setActiveProbe(1, buttonVal1 > 0.0);
        break;
    case ControlPanel::BUTTON_PROBE_LEFT:
        emit setActiveProbe(0, buttonVal1 > 0.0);
        break;
    // Basic momentary buttons for movement control
    case ControlPanel::BUTTON_X:
        poseAdjust.setParams(buttonVal1 * 10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        emit poseAdjustRequest(poseAdjust, false);  // X translation in global coordinates
        break;
    case ControlPanel::BUTTON_Y:
        poseAdjust.setParams(0.0, buttonVal1 * 10.0, 0.0, 0.0, 0.0, 0.0);
        emit poseAdjustRequest(poseAdjust, false);  // Y translation in global coordinates
        break;
    case ControlPanel::BUTTON_Z:
        poseAdjust.setParams(0.0, 0.0, buttonVal1 * 10.0, 0.0, 0.0, 0.0);
        emit poseAdjustRequest(poseAdjust, false);  // Z translation in global coordinates
        break;
    case ControlPanel::BUTTON_AX:
        poseAdjust.setParams(0.0, 0.0, 0.0, 0.0, buttonVal1 * 5.0, 0.0);
        emit poseAdjustRequest(poseAdjust, true);  // Axial rotation in local probe coordinates
        break;
    // Stop button
    case ControlPanel::BUTTON_STOP:
        emit cancel();
        break;
    // Joystick movements for tilting
    case ControlPanel::BUTTON_JOYSTICK:
    {
        poseAdjust.setIdentity();
        // Define rotation using a quaternion representing angle and axis
        double angle = buttonVal1 * 5.0;  // From magnitude
        Point3D axis(std::sin(buttonVal2), -std::cos(buttonVal2), 0.0);  // Rotation axis, from direction value. Zero direction is a -Y axis rotation, +90 deg is +X axis.
        axis.normalise();
        double cosHalfAngle = std::cos(angle * pi/180.0 * 0.5);
        double sinHalfAngle = std::sin(angle * pi/180.0 * 0.5);
        poseAdjust.setRotation(cosHalfAngle, axis * sinHalfAngle);
        emit poseAdjustRequest(poseAdjust, false);
        break;
    }
    case ControlPanel::BUTTON_NONE:
        break;
    }
}
