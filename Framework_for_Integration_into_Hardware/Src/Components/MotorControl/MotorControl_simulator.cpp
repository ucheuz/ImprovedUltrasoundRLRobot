#include <iostream>

#include "MotorControl_simulator.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
MotorControl_simulator::MotorControl_simulator(const QString& robotId)
{
    initialise(robotId);

    qRegisterMetaType<std::vector<double> >("std::vector<double>");
    qRegisterMetaType<std::vector<unsigned int> >("std::vector<unsigned int>");
}

MotorControl_simulator::~MotorControl_simulator()
{
    disconnectMotors();
}

//======================================================================
// Functions inplemented from IMotorControl
//======================================================================
//----------------------------------------------------------------------
// Connect motors
//----------------------------------------------------------------------
void MotorControl_simulator::connectMotors()
{
    if (_status == INVALID_SETUP)
        return;

    disconnectMotors();

    _motorSimThread = new QThread();
    _motorSim = new SimulatedMotors(_nMotors);
    _motorSim->setStepsPerDegree(_stepsPerDegree);
    _motorSim->setHomePosition(_homePos);

    _motorSim->moveToThread(_motorSimThread);

    // Thread signals
    connect(_motorSimThread, &QThread::started, _motorSim, &SimulatedMotors::run);
    connect(this, &MotorControl_simulator::motorSimStopMotorProcess, _motorSim, &SimulatedMotors::stop);
    connect(_motorSim, &SimulatedMotors::finished, _motorSimThread, &QThread::quit);
    connect(_motorSim, &SimulatedMotors::finished, _motorSim, &SimulatedMotors::deleteLater);
    connect(_motorSimThread, &QThread::finished, _motorSimThread, &QThread::deleteLater);

    // Motor control signals
    connect(this, &MotorControl_simulator::motorSimTogglePause, _motorSim, &SimulatedMotors::togglePause);
    connect(this, &MotorControl_simulator::motorSimCancelMove, _motorSim, &SimulatedMotors::cancelMove);

    connect(this, &MotorControl_simulator::motorSimMoveAllAbsolute, _motorSim, &SimulatedMotors::moveAllAbsolute);
    connect(this, &MotorControl_simulator::motorSimMoveAbsolute, _motorSim, &SimulatedMotors::moveAbsolute);

    connect(this, &MotorControl_simulator::motorSimMoveSeveralHome, _motorSim, &SimulatedMotors::moveSeveralHome);
    connect(this, &MotorControl_simulator::motorSimMoveHome, _motorSim, &SimulatedMotors::moveHome);

    connect(_motorSim, &SimulatedMotors::movementFinished, this, &MotorControl_simulator::goIdle);

    _motorSimThread->start();

    // Notify status label in GUI
    _status = CONNECTED;
    emit connectionStatus("OK");

    emit connectionChanged();
}

//----------------------------------------------------------------------
// Direct motor control via motor simulator
//----------------------------------------------------------------------
void MotorControl_simulator::moveAllAbsolute(const std::vector<double>& motorPositions)
{
    if (!isConnected())
        return;

    if (motorPositions.size() < _nMotors) {
        std::cerr << "Error in MotorControl_simulator::moveAllAbsolute: wrong size of positions vector." << std::endl;
        return;
    }    
    std::vector<double> motorPositionsResized(motorPositions);
    motorPositionsResized.resize(_motorSim->getNMotors(), 0.0);

    _moving = true;
    emit motorSimMoveAllAbsolute(motorPositionsResized, _speedScale);
}

void MotorControl_simulator::moveAbsolute(unsigned int idx, double motorPosition)
{
    if (!isConnected())
        return;

    if (idx >= _nMotors) {
        std::cerr << "Error in MotorControl_simulator::moveAbsolute: index is out of range." << std::endl;
        return;
    }

    _moving = true;
    emit motorSimMoveAbsolute(idx, motorPosition, _speedScale);
}

void MotorControl_simulator::moveAllRelative(const std::vector<double> &motorDistances)
{
    if (!isConnected())
        return;

    if (motorDistances.size() != _nMotors) {
        std::cerr << "Error in MotorControl_simulator::moveAllRelative: wrong size of positions vector." << std::endl;
        return;
    }

    std::vector<double> motorPositions;
    _motorSim->getAllPositions(motorPositions);
    for (unsigned int i = 0; i < _motorSim->getNMotors(); i++)
        motorPositions[i] += motorDistances[i];

    _moving = true;
    emit motorSimMoveAllAbsolute(motorPositions, _speedScale);
}

void MotorControl_simulator::moveRelative(unsigned int idx, double motorDistance)
{
    if (!isConnected())
        return;

    if (idx >= _nMotors) {
        std::cerr << "Error in MotorControl_accelStepper::moveRelative: index is out of range." << std::endl;
        return;
    }

    double motorPosition = _motorSim->getPosition(idx);
    motorPosition += motorDistance;

    _moving = true;
    emit motorSimMoveAbsolute(idx, motorPosition, _speedScale);
}

//----------------------------------------------------------------------
// Motor homing
//----------------------------------------------------------------------
void MotorControl_simulator::moveAllHome()
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
    _motorSim->getAllPositions(targetPositions);

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
        emit motorSimMoveSeveralHome(motorsToHome);

        // Wait for homing to finish
        while (_moving)
            qApp->processEvents();
        qApp->processEvents();
        if (_cancelSequence) {
            _cancelSequence = false;
            return;
        }

        // Move these motors to neutral position
        _moving = true;
        //emit motorSimMoveAllAbsolute(targetPositions, _speedScale);
        emit motorSimMoveAllAbsolute(targetPositions, -1.0);

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

void MotorControl_simulator::moveHome(unsigned int idx)
{
    if (!isConnected())
        return;

    if (idx >= _nMotors) {
        std::cerr << "Error in MotorControl_simulator::moveHome: index is out of range." << std::endl;
        return;
    }

    if (!isHomeable(idx)) {
        std::cerr << "Error in MotorControl_simulator::moveHome: motor is not homeable." << std::endl;
        return;
    }

    if (_cancelSequence)  // Clear any previous cancelations
        _cancelSequence = false;

    // Start homing
    _moving = true;
    emit motorSimMoveHome(idx);

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
    //emit motorSimMoveAbsolute(idx, _neutralPos[idx], _speedScale);
    emit motorSimMoveAbsolute(idx, _neutralPos[idx], -1.0);

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
void MotorControl_simulator::togglePause()
{
    emit motorSimTogglePause();
}

void MotorControl_simulator::cancelMove()
{
    _cancelSequence = true;
    emit cancelSequence();
    emit motorSimCancelMove();
}

//----------------------------------------------------------------------
// Access motor positions
//----------------------------------------------------------------------
void MotorControl_simulator::getAllMotorPositions(std::vector<double>& currentPositions) const
{
    if (!isConnected())
        return;

    _motorSim->getAllPositions(currentPositions);
}

double MotorControl_simulator::getMotorPosition(unsigned int idx) const
{
    if (!isConnected())
        return 0.0;

    if (idx >= _nMotors) {
        std::cerr << "Error in MotorControl_simulator::getMotorPosition: index is out of range." << std::endl;
        return 0.0;
    }

    double pos = _motorSim->getPosition(idx);

    return pos;
}

//----------------------------------------------------------------------
// Access motor positions, speeds and status
//----------------------------------------------------------------------
void MotorControl_simulator::getAllMotorSpeeds(std::vector<double>& currentSpeed) const
{
    if (!isConnected())
        return;

    _motorSim->getAllSpeeds(currentSpeed);
}

double MotorControl_simulator::getMotorSpeed(unsigned int idx) const
{
    if (!isConnected())
        return 0.0;
    else
        return _motorSim->getSpeed(idx);
}

void MotorControl_simulator::getAllStatusStrings(std::vector<QString>& statusString) const
{
    if (!isConnected())
        return;

    std::vector<int> status;
    _motorSim->getAllStatuses(status);
    statusString.resize(_motorSim->getNMotors(), "");
    for (unsigned int i = 0; i < _motorSim->getNMotors(); i++)
        statusString[i] = _motorSim->statusToStatusString(status[i]);
}

QString MotorControl_simulator::getStatusString(unsigned int idx) const
{
    if (!isConnected())
        return QString();
    else
        return _motorSim->getStatusString(idx);
}

void MotorControl_simulator::getAllMotorParams(std::vector<float>& maxSpeed, std::vector<float>& acceleration) const
{
    if (!isConnected())
        return;

    _motorSim->getAllMotorParams(maxSpeed, acceleration);
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Stops the motor simulator thread
//----------------------------------------------------------------------
void MotorControl_simulator::disconnectMotors()
{
    // Signal motor simulator thread to finish
    emit motorSimStopMotorProcess();

    // Wait for motors to disconnect fully so it's safe to reconnect if needed
    while (_motorSim != nullptr)
        qApp->processEvents();

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    emit connectionChanged();
}
