#include <iostream>

#include "MotorControl_accelStepper.h"

Q_DECLARE_METATYPE(std::vector<double>)

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
MotorControl_accelStepper::MotorControl_accelStepper(const QString& robotId)
{
    _accelStepperThread = nullptr;
    _accelStepper = nullptr;

    initialise(robotId);

    // Additional checks specific to this motor controller
    if (_arduinoIds.size() != 1) {
        _status = INVALID_SETUP;
        std::cerr << "Error in MotorControl_accelStepper: failed to read valid motor specification from xml file." << std::endl;
    }

    qRegisterMetaType<std::vector<double> >("std::vector<double>");
    qRegisterMetaType<std::vector<unsigned int> >("std::vector<unsigned int>");
}

MotorControl_accelStepper::~MotorControl_accelStepper()
{
    disconnectMotors();
}

//======================================================================
// Functions inplemented from IMotorControl
//======================================================================
//----------------------------------------------------------------------
// Connect motors
//----------------------------------------------------------------------
void MotorControl_accelStepper::connectMotors()
{
    if (_status == INVALID_SETUP)
        return;

    disconnectMotors();
    // *** Starts up paused because the above function sends a cancel command

    _accelStepperThread = new QThread();
    _accelStepper = new AccelStepperMotors(_arduinoIds[0], _nMotors);
    _accelStepper->setStepsPerDegree(_stepsPerDegree);
    _accelStepper->setHomePosition(_homePos);

    bool success = _accelStepper->initialise();

    if (!success) {
        std::cerr << "Error in MotorControl_accelStepper::connectMotors: failed to connect to AccelStepper Teensy (" << _arduinoIds[0].toStdString() << ")." << std::endl;
        delete _accelStepper;
        delete _accelStepperThread;
    }
    else {

        _accelStepper->moveToThread(_accelStepperThread);

        // Thread signals
        connect(_accelStepperThread, &QThread::started, _accelStepper, &AccelStepperMotors::run);
        connect(this, &MotorControl_accelStepper::accelStepperStopMotorProcess, _accelStepper, &AccelStepperMotors::stop);
        connect(_accelStepper, &AccelStepperMotors::finished, _accelStepperThread, &QThread::quit);
        connect(_accelStepper, &AccelStepperMotors::finished, _accelStepper, &AccelStepperMotors::deleteLater);
        connect(_accelStepperThread, &QThread::finished, _accelStepperThread, &QThread::deleteLater);

        // Motor control signals
        connect(this, &MotorControl_accelStepper::accelStepperTogglePause, _accelStepper, &AccelStepperMotors::togglePause);
        connect(this, &MotorControl_accelStepper::accelStepperCancelMove, _accelStepper, &AccelStepperMotors::cancelMove);
        connect(this, &MotorControl_accelStepper::accelStepperSend, _accelStepper, &AccelStepperMotors::send);

        connect(this, &MotorControl_accelStepper::accelStepperMoveAllAbsolute, _accelStepper, &AccelStepperMotors::moveAllAbsolute);
        connect(this, &MotorControl_accelStepper::accelStepperMoveAbsolute, _accelStepper, &AccelStepperMotors::moveAbsolute);

        connect(this, &MotorControl_accelStepper::accelStepperMoveSeveralHome, _accelStepper, &AccelStepperMotors::moveSeveralHome);
        connect(this, &MotorControl_accelStepper::accelStepperMoveHome, _accelStepper, &AccelStepperMotors::moveHome);

        connect(_accelStepper, &AccelStepperMotors::movementFinished, this, &MotorControl_accelStepper::goIdle);

        _accelStepperThread->start();
    }

    //_accelStepper->togglePause();  // Unpause

    // Notify status label in GUI
    if (_accelStepper != nullptr) {
        _status = CONNECTED;
        emit connectionStatus("OK");
    }
    else {
        _status = CONNECT_FAILED;
        emit connectionStatus("Connect failed");
    }

    emit connectionChanged();
}

//----------------------------------------------------------------------
// Direct motor control via AccelStepperMotors
//----------------------------------------------------------------------
void MotorControl_accelStepper::moveAllAbsolute(const std::vector<double>& motorPositions)
{
    if (!isConnected())
        return;

    if (motorPositions.size() < _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::moveAllAbsolute: wrong size of positions vector." << std::endl;
        return;
    }
    std::vector<double> motorPositionsResized(motorPositions);
    motorPositionsResized.resize(_accelStepper->getNMotors(), 0.0);

    _moving = true;
    emit accelStepperMoveAllAbsolute(_arduinoIds[0], motorPositionsResized, _speedScale);
}

void MotorControl_accelStepper::moveAbsolute(unsigned int idx, double motorPosition)
{
    if (!isConnected())
        return;

    if (idx >= _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::moveAbsolute: index is out of range." << std::endl;
        return;
    }

    _moving = true;
    emit accelStepperMoveAbsolute(_arduinoIds[0], idx, motorPosition, _speedScale);
}

void MotorControl_accelStepper::moveAllRelative(const std::vector<double> &motorDistances)
{
    if (!isConnected())
        return;

    if (motorDistances.size() != _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::moveAllRelative: wrong size of positions vector." << std::endl;
        return;
    }

    std::vector<double> motorPositions;
    _accelStepper->getAllPositions(motorPositions);
    for (unsigned int i = 0; i < _accelStepper->getNMotors(); i++)
        motorPositions[i] += motorDistances[i];
    _moving = true;
    emit accelStepperMoveAllAbsolute(_arduinoIds[0], motorPositions, _speedScale);
}

void MotorControl_accelStepper::moveRelative(unsigned int idx, double motorDistance)
{
    if (!isConnected())
        return;

    if (idx >= _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::moveRelative: index is out of range." << std::endl;
        return;
    }

    double motorPosition = _accelStepper->getPosition(idx);
    motorPosition += motorDistance;
    _moving = true;
    emit accelStepperMoveAbsolute(_arduinoIds[0], idx, motorPosition, _speedScale);
}

//----------------------------------------------------------------------
// Motor homing
//----------------------------------------------------------------------
void MotorControl_accelStepper::moveAllHome()
{
    if (!isConnected())
        return;

    // Number of sequence steps
    unsigned int nSequenceSteps = 0;
    for (unsigned int i = 0; i < _homingOrder.size(); i++) {
        if (_homingOrder[i] > nSequenceSteps)
            nSequenceSteps = _homingOrder[i];
    }

    std::vector<double> targetPositions;
    _accelStepper->getAllPositions(targetPositions);

    if (_cancelSequence)  // Clear any previous cancelations
        _cancelSequence = false;

    for (unsigned int s = 1; s <= nSequenceSteps; s++) {
        // List which motors to move
        std::vector<unsigned int> motorsToHome;
        for (unsigned int i = 0; i < _homingOrder.size(); i++) {
            if (_homingOrder[i] == s && isHomeable(i)) {
                motorsToHome.push_back(i);
                targetPositions[i] = _neutralPos[i];
            }
        }

        // Start homing movement
        _moving = true;
        emit accelStepperMoveSeveralHome(_arduinoIds[0], motorsToHome, _speedScale);
        //std::cerr << "Running home" << std::endl;

        // Wait for homing to finish
        while (_moving)
            qApp->processEvents();
        qApp->processEvents();
        if (_cancelSequence) {
            //std::cerr << "Sequence cancelled" << std::endl;
            _cancelSequence = false;
            return;
        }

        // Move these motors to neutral position
        _moving = true;
        emit accelStepperMoveAllAbsolute(_arduinoIds[0], targetPositions, _speedScale);
        //std::cerr << "Running neutral" << std::endl;

        // Wait for movement to finish
        while (_moving)
            qApp->processEvents();
        qApp->processEvents();
        if (_cancelSequence) {
            //std::cerr << "Sequence cancelled" << std::endl;
            _cancelSequence = false;
            return;
        }
    }
}

void MotorControl_accelStepper::moveHome(unsigned int idx)
{
    if (!isConnected())
        return;

    if (idx >= _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::moveHome: index is out of range." << std::endl;
        return;
    }

    if (!isHomeable(idx)) {
        std::cerr << "Error in MotorControl_accelStepper::moveHome: motor is not homeable." << std::endl;
        return;
    }

    if (_cancelSequence)  // Clear any previous cancelations
        _cancelSequence = false;

    // Start homing
    _moving = true;
    emit accelStepperMoveHome(_arduinoIds[0], idx, _speedScale);
    //std::cerr << "Running home" << std::endl;

    // Wait for homing to finish
    while (_moving)
        qApp->processEvents();
    qApp->processEvents();
    if (_cancelSequence) {
        //std::cerr << "Sequence cancelled" << std::endl;
        _cancelSequence = false;
        return;
    }

    // Move to neutral position
    _moving = true;
    emit accelStepperMoveAbsolute(_arduinoIds[0], idx, _neutralPos[idx], _speedScale);
    //std::cerr << "Running neutral" << std::endl;

    // Wait for movement to finish
    while (_moving)
        qApp->processEvents();
    qApp->processEvents();
    if (_cancelSequence) {
        //std::cerr << "Sequence cancelled" << std::endl;
        _cancelSequence = false;
        return;
    }
}

//----------------------------------------------------------------------
// Pause / stop motors
//----------------------------------------------------------------------
void MotorControl_accelStepper::togglePause()
{
    emit accelStepperTogglePause();
}

void MotorControl_accelStepper::cancelMove()
{
    _cancelSequence = true;
    emit cancelSequence();
    emit accelStepperCancelMove();
}

//----------------------------------------------------------------------
// Access motor positions
//----------------------------------------------------------------------
void MotorControl_accelStepper::getAllMotorPositions(std::vector<double>& currentPositions) const
{
    if (!isConnected())
        return;

    _accelStepper->getAllPositions(currentPositions);
}

double MotorControl_accelStepper::getMotorPosition(unsigned int idx) const
{
    if (!isConnected())
        return 0.0;

    if (idx >= _accelStepper->getNMotors()) {
        std::cerr << "Error in MotorControl_accelStepper::getMotorPosition: index is out of range." << std::endl;
        return 0.0;
    }

    double pos = _accelStepper->getPosition(idx);

    return pos;
}

//----------------------------------------------------------------------
// Access motor positions, speeds and status
//----------------------------------------------------------------------
void MotorControl_accelStepper::getAllMotorSpeeds(std::vector<double>& currentSpeed) const
{
    if (!isConnected())
        return;

    _accelStepper->getAllSpeeds(currentSpeed);
}

double MotorControl_accelStepper::getMotorSpeed(unsigned int idx) const
{
    if (!isConnected())
        return 0.0;
    else
        return _accelStepper->getSpeed(idx);
}

void MotorControl_accelStepper::getAllStatusStrings(std::vector<QString>& statusString) const
{
    if (!isConnected())
        return;

    std::vector<int> status;
    _accelStepper->getAllStatuses(status);
    statusString.resize(_accelStepper->getNMotors(), "");
    for (unsigned int i = 0; i < _accelStepper->getNMotors(); i++)
        statusString[i] = _accelStepper->statusToStatusString(status[i]);
}

QString MotorControl_accelStepper::getStatusString(unsigned int idx) const
{
    if (!isConnected())
        return QString();
    else
        return _accelStepper->getStatusString(idx);
}

void MotorControl_accelStepper::getAllMotorParams(std::vector<float>& maxSpeed, std::vector<float>& acceleration) const
{
    if (!isConnected())
        return;

    _accelStepper->getAllMotorParams(maxSpeed, acceleration);
}

//======================================================================
// Functions needed by StepperMotorsWindow GUI
//======================================================================
//----------------------------------------------------------------------
// Directly send a serial command from the GUI
//----------------------------------------------------------------------
void MotorControl_accelStepper::send(QString message)
{
    emit accelStepperSend(_arduinoIds[0], message);
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the motors and clear variables ready to reconnect
//----------------------------------------------------------------------
void MotorControl_accelStepper::disconnectMotors()
{
    // Signal Arduino to disconnect
    emit accelStepperStopMotorProcess();

    // Wait for motors to disconnect fully so it's safe to reconnect if needed
    while (_accelStepper != nullptr)
        qApp->processEvents();

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    emit connectionChanged();
}
