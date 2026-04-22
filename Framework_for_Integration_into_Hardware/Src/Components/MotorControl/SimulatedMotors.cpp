#include <iostream>
#include <sstream>
#include <cmath>

#include "SimulatedMotors.h"

#include <qdebug.h>
#include <QDateTime>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SimulatedMotors::SimulatedMotors(unsigned int nMotors)
{
    _nMotors = nMotors;

    _running = false;

    _paused = false;
    _homing = false;
    _allMotorsIdle = false;

    _motorStatus.resize(_nMotors, MOTOR_DISCONNECTED);
    _currentPos.resize(_nMotors, 0);
    _targetPos.resize(_nMotors, 0);
    _origTargetPos.resize(_nMotors, 0);
    _currentSpeed.resize(_nMotors, 0);
    _currentMaxSpeed.resize(_nMotors, 0);

    _posScale.resize(_nMotors, 1.0);
    _posOffset.resize(_nMotors, 0.0);

    // Typical values used in the robot control
    // These could be hardcoded to other values for more accurate simulation
    if (_nMotors == 17)  // Robot v3.3 or v3.4
        _maxSpeed = {8000, 8000, 8000, 8000, 8000, 8000, 3000, 3000, 3000, 8000, 8000, 8000, 8000, 8000, 3000, 3000, 3000};
    else
        _maxSpeed.resize(_nMotors, 5000);
    _acceleration.resize(_nMotors, 8000);

    _stepInterval.resize(_nMotors, 0);
    _lastStepTime.resize(_nMotors, 0);
    _direction.resize(_nMotors, 0);
    _n.resize(_nMotors, 0);
    _cn.resize(_nMotors, 0.0);
    _c0.resize(_nMotors, 0.0);
    _cmin.resize(_nMotors, 1.0);

    // Starting step interval
    for (unsigned int m = 0; m < _nMotors; m++) {
        // New c0 per Equation 7, with correction per Equation 15
        _c0[m] = 0.676 * sqrt(2.0 / _acceleration[m]) * 1000000.0; // Equation 15
    }

#ifdef WIN32
    QueryPerformanceFrequency(&_ticksPerSecond);
    QueryPerformanceCounter(&_startTime);
#else
    _startTime = std::chrono::high_resolution_clock::now();
#endif

#ifdef WRITE_TO_FILE
        QString outFilename("./Motors.txt");
        _outFile.setFileName(outFilename);
        bool success = _outFile.open(QFile::WriteOnly | QFile::Text);
        if (!success) {
            std::cerr << "Error in SimulatedMotors: unable to open file " << outFilename.toStdString() << "." << std::endl;
            return;
        }
        _fout.setDevice(&_outFile);

        _loggingTimer = new QTimer(this);
        connect(_loggingTimer, &QTimer::timeout, this, &SimulatedMotors::logStatus);
        _loggingTimer->start(10);
#endif  // WRITE_TO_FILE
}

SimulatedMotors::~SimulatedMotors()
{
#ifdef WRITE_TO_FILE
    if (_outFile.isOpen())
        _outFile.close();
#endif  // WRITE_TO_FILE
}

//----------------------------------------------------------------------
// Set movement scale
//----------------------------------------------------------------------
void SimulatedMotors::setStepsPerDegree(std::vector<double> stepsPerDegree)
{
    if (stepsPerDegree.size() != _nMotors) {
        std::cerr << "Error in SimulatedMotors::setStepsPerDegree: wrong input array size." << std::endl;
        return;
    }

    for (unsigned int i = 0; i < _nMotors; i++)
        _posScale[i] = stepsPerDegree[i];
}

void SimulatedMotors::setHomePosition(std::vector<double> homePos)
{
    if (homePos.size() != _nMotors) {
        std::cerr << "Error in SimulatedMotors::setHomePosition: wrong input array size." << std::endl;
        return;
    }

    for (unsigned int i = 0; i < _nMotors; i++)
        _posOffset[i] = homePos[i];
}

//----------------------------------------------------------------------
// Access functions
//----------------------------------------------------------------------
void SimulatedMotors::getAllPositions(std::vector<double>& pos) const
{
    pos.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        pos[i] = double(_currentPos[i]) / _posScale[i] + _posOffset[i];
}

double SimulatedMotors::getPosition(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return double(_currentPos[idx]) / _posScale[idx] + _posOffset[idx];
    }
    else {
        std::cerr << "Error in SimulatedMotors::getPosition: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return 0.0;
    }
}

void SimulatedMotors::getAllSpeeds(std::vector<double>& speed) const
{
    speed.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        speed[i] = double(_currentSpeed[i]) / _posScale[i];
}

double SimulatedMotors::getSpeed(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return double(_currentSpeed[idx]) / _posScale[idx];
    }
    else {
        std::cerr << "Error in SimulatedMotors::getSpeed: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return 0.0;
    }
}

void SimulatedMotors::getAllStatuses(std::vector<int>& status) const
{
    status.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        status[i] = _motorStatus[i];
}

QString SimulatedMotors::getStatusString(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return statusToStatusString(_motorStatus[idx]);
    }
    else {
        std::cerr << "Error in SimulatedMotors::getStatusString: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return "";
    }
}

void SimulatedMotors::getAllMotorParams(std::vector<float> &maxSpeed, std::vector<float> &acceleration) const
{
    maxSpeed.resize(_nMotors);
    acceleration.resize(_nMotors);

    for (unsigned int i = 0; i < _nMotors; i++) {
        maxSpeed[i] = float(_maxSpeed[i]);
        acceleration[i] = float(_acceleration[i]);
    }
}

QString SimulatedMotors::statusToStatusString(int status)
{
    QString statusString;

    switch(status) {
    case MOTOR_DISCONNECTED:
        statusString.append("Disconn.");
        break;
    case MOTOR_IDLE:
        statusString.append("Idle");
        break;
    case MOTOR_RUNNING:
        statusString.append("Run");
        break;
    case MOTOR_HOMING:
        statusString.append("Homing");
        break;
    case MOTOR_PAUSED:
        statusString.append("Paused");
        break;
    case MOTOR_UNKNOWN:
    default:
        statusString.append("Unknown");
        break;
    }

    return statusString;
}

unsigned int SimulatedMotors::getNMotors() const
{
    return _nMotors;
}

bool SimulatedMotors::isConnected() const
{
    bool connected = true;
    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        if (_motorStatus[i] == MOTOR_DISCONNECTED || _motorStatus[i] == MOTOR_UNKNOWN) {
            connected = false;
            break;
        }

    return connected;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for motor simulation
//----------------------------------------------------------------------
void SimulatedMotors::run()
{
    _running = true;

    while (_running) {
        updateStatus();
        qApp->processEvents();
    }

    emit finished();
}

//----------------------------------------------------------------------
// Thread process stop function
//----------------------------------------------------------------------
void SimulatedMotors::stop()
{
    cancelMove();
    _running = false;
}

//----------------------------------------------------------------------
// General motor control
//----------------------------------------------------------------------
void SimulatedMotors::togglePause()
{
    QMutexLocker locker(&_mutex);

    _paused = !_paused;
    if (_paused) {
        for (unsigned int i = 0; i < _nMotors; i++) {
            _origTargetPos[i] = _targetPos[i];
            _targetPos[i] = _currentPos[i];
        }
    }
    else {
        for (unsigned int i = 0; i < _nMotors; i++) {
            _targetPos[i] = _origTargetPos[i];
        }
    }
}

void SimulatedMotors::cancelMove()
{
    for (unsigned int i = 0; i < _nMotors; i++)
        _targetPos[i] = _currentPos[i];
}

//----------------------------------------------------------------------
// Slot for motor movement - all motors
//----------------------------------------------------------------------
void SimulatedMotors::moveAllAbsolute(std::vector<double> pos, double speedScale)
{
    QMutexLocker locker(&_mutex);

    if (speedScale < 0) {
        // Jump directly to target position
        for (unsigned int m = 0; m < _nMotors; m++) {
            _targetPos[m] = int((pos[m] - _posOffset[m]) * _posScale[m] + 0.5);
            _currentPos[m] = _targetPos[m];
            _currentSpeed[m] = 0.0;
            _stepInterval[m] = 0;
            _n[m] = 0;
            _motorStatus[m] = MOTOR_IDLE;
        }
    }

    else {
        for (unsigned int m = 0; m < _nMotors; m++) {
            _targetPos[m] = int((pos[m] - _posOffset[m]) * _posScale[m] + 0.5);
        }

        updateCurrentMaxSpeeds(speedScale);
    }

    _allMotorsIdle = false;  // Assume move has started
}

//----------------------------------------------------------------------
// Slot for motor movement - single motor
//----------------------------------------------------------------------
void SimulatedMotors::moveAbsolute(unsigned int idx, double pos, double speedScale)
{
    if (idx >= _nMotors) {
        std::cerr << "Error in SimulatedMotors::moveAbsolute: idx is out of range (0-" << _nMotors-1 << ")." << std::endl;
        return;
    }

    QMutexLocker locker(&_mutex);

    if (speedScale < 0) {
        // Jump directly to target position
        _targetPos[idx] = int((pos - _posOffset[idx]) * _posScale[idx] + 0.5);
        _currentPos[idx] = _targetPos[idx];
        _currentSpeed[idx] = 0.0;
        _stepInterval[idx] = 0;
        _n[idx] = 0;
        _motorStatus[idx] = MOTOR_IDLE;
    }

    else {
        _targetPos[idx] = int((pos - _posOffset[idx]) * _posScale[idx] + 0.5);

        updateCurrentMaxSpeeds(speedScale);
    }

    _allMotorsIdle = false;  // Assume move has started
}

//----------------------------------------------------------------------
// Slot for motor homing - several motors
//----------------------------------------------------------------------
void SimulatedMotors::moveSeveralHome(std::vector<unsigned int> motorsToMove)
{
    for (unsigned int i = 0; i < motorsToMove.size(); i++) {
        if (motorsToMove[i] >= _nMotors) {
            std::cerr << "Error in AccelStepperMotors::moveSeveralHome: index is out of range." << std::endl;
            return;
        }
    }

    QMutexLocker locker(&_mutex);

    for (unsigned int m = 0; m < motorsToMove.size(); m++) {
        unsigned int idx = motorsToMove[m];
        _currentPos[idx] = int(_posOffset[idx]);
        _targetPos[idx] = _currentPos[idx];
        _currentSpeed[idx] = 0.0;
        _stepInterval[idx] = 0;
        _n[idx] = 0;
        _motorStatus[idx] = MOTOR_IDLE;
    }

    _allMotorsIdle = false;  // Assume move has started
}

//----------------------------------------------------------------------
// Slot for motor homing - single motor
//----------------------------------------------------------------------
void SimulatedMotors::moveHome(unsigned int idx)
{
    if (idx >= _nMotors) {
        std::cerr << "Error in SimulatedMotors::moveHome: idx is out of range (0-" << _nMotors-1 << ")." << std::endl;
        return;
    }

    QMutexLocker locker(&_mutex);

    _currentPos[idx] = int(_posOffset[idx]);
    _targetPos[idx] = _currentPos[idx];
    _currentSpeed[idx] = 0.0;
    _stepInterval[idx] = 0;
    _n[idx] = 0;
    _motorStatus[idx] = MOTOR_IDLE;

    _allMotorsIdle = false;  // Assume move has started
}

//======================================================================
// Private functions
//======================================================================
#ifdef WRITE_TO_FILE
//----------------------------------------------------------------------
// Log current status to file.
//----------------------------------------------------------------------
void SimulatedMotors::logStatus()
{
    if (_outFile.isOpen()) {
        QMutexLocker locker(&_mutex);

        for (unsigned int m = 0; m < _nMotors; m++) {
            //std::cerr << _currentPos[m] << " " << _currentSpeed[m] << " ";
            _fout << _currentPos[m] << " " << _currentSpeed[m] << " ";
        }
        //std::cerr << std::endl;
        _fout << endl;
    }
}
#endif  // WRITE_TO_FILE

//----------------------------------------------------------------------
// Calculate new max speeds for each motor given current position and
// targets so that all motors arrive at their targets simultaneously.
//----------------------------------------------------------------------
void SimulatedMotors::updateCurrentMaxSpeeds(double speedScale)
{
    std::vector<double> scaledMaxSpeed(_nMotors, 0.0);
    for (unsigned int m = 0; m < _nMotors; m++)
        scaledMaxSpeed[m] = _maxSpeed[m] * speedScale;

    // Calculate minimum time for each motor to reach target
    std::vector<int> stepsToTarget(_nMotors, 0);
    std::vector<double> minTimes(_nMotors, 0.0);
    std::vector<int8_t> moveType(_nMotors, 0);

    //QMutexLocker locker(&_mutex);  // Not needed, as mutex is already locked when this function is called.

    for (unsigned int m = 0; m < _nMotors; m++) {
        stepsToTarget[m] = _targetPos[m] - _currentPos[m];
        int stepsToStop = int(_currentSpeed[m] * _currentSpeed[m] / (2.0 * _acceleration[m]));

        if (stepsToStop <= stepsToTarget[m]) {  // Case 1
            moveType[m] = 1;
            int stepsToAccel = int((scaledMaxSpeed[m] * scaledMaxSpeed[m] - _currentSpeed[m] * _currentSpeed[m]) / (2.0 * _acceleration[m]));
            int stepsToDecel = int(scaledMaxSpeed[m] * scaledMaxSpeed[m] / (2.0 * _acceleration[m]));

            if (stepsToTarget[m] >= stepsToAccel + stepsToDecel) {  // Case 1a
                minTimes[m] = stepsToTarget[m] / scaledMaxSpeed[m] + (scaledMaxSpeed[m] - _currentSpeed[m]) / _acceleration[m] + _currentSpeed[m] * _currentSpeed[m] / (2.0 * _acceleration[m] * scaledMaxSpeed[m]);
            }
            else {  // Case 1b
                minTimes[m] = (2.0 / _acceleration[m]) * (sqrt(_acceleration[m] * stepsToTarget[m] + _currentSpeed[m] * _currentSpeed[m] / 2.0) - _currentSpeed[m] / 2.0);
            }
        }
        else {  // Case 2
            moveType[m] = 2;
            int stepsToAccel = int((_currentSpeed[m] * _currentSpeed[m] - scaledMaxSpeed[m] * scaledMaxSpeed[m]) / (2.0 * _acceleration[m]));
            int stepsToDecel = int(-scaledMaxSpeed[m] * scaledMaxSpeed[m] / (2.0 * _acceleration[m]));

            if (stepsToTarget[m] <= stepsToAccel + stepsToDecel) {  // Case 2a
                minTimes[m] = -stepsToTarget[m] / scaledMaxSpeed[m] + (scaledMaxSpeed[m] + _currentSpeed[m]) / _acceleration[m] + _currentSpeed[m] * _currentSpeed[m] / (2.0 * _acceleration[m] * scaledMaxSpeed[m]);
            }
            else {  // Case 2b
                minTimes[m] = (2.0 / _acceleration[m]) * (sqrt(-_acceleration[m] * stepsToTarget[m] + _currentSpeed[m] * _currentSpeed[m] / 2.0) + _currentSpeed[m] / 2.0);
            }
        }
    }

    // Find which motor needs the most time
    unsigned int slowestMotorIdx = 0;
    double maxAllTimes = -1.0;
    for (unsigned int m = 0; m < _nMotors; m++) {
        if (m == 0 || minTimes[m] > maxAllTimes) {
            maxAllTimes = minTimes[m];
            slowestMotorIdx = m;
        }
    }

    // Calculate new speeds to slow each motor to take maxAllTimes
    double newMaxSpeed;
    for (unsigned int m = 0; m < _nMotors; m++) {
        if (moveType[m] == 1) {  // Case 1
            double c = 0.5 * (_acceleration[m] * maxAllTimes + _currentSpeed[m]);
            double A = c*c - (_currentSpeed[m] * _currentSpeed[m] / 2.0 + stepsToTarget[m] * _acceleration[m]);
            newMaxSpeed = c - std::sqrt(std::max(A, 0.0));
        }
        else {  // Case 2
            double c = 0.5 * (_acceleration[m] * maxAllTimes - _currentSpeed[m]);
            double A = c*c - (_currentSpeed[m] * _currentSpeed[m] / 2.0 - stepsToTarget[m] * _acceleration[m]);
            newMaxSpeed = c - std::sqrt(std::max(A, 0.0));
        }
        if (newMaxSpeed > _maxSpeed[m]) {
            //std::cerr << "Error in calculating new speeds: newMaxSpeed " << newMaxSpeed << " is above motor maximum " << _maxSpeed[m] << "." << std::endl;
            _currentMaxSpeed[m] = _maxSpeed[m];
        }
        else
            _currentMaxSpeed[m] = newMaxSpeed;

        // Update other parameters for motor control
        _cmin[m] = 1000000.0 / _currentMaxSpeed[m];
        if (_n[m] > 0) {
            _n[m] = int((_currentSpeed[m] * _currentSpeed[m]) / (2.0 * _acceleration[m]));
        }
        computeNewRunSpeed(m);
    }
}

//----------------------------------------------------------------------
// Called repeatedly by run function. Calculates the current position
// and speed of the motors, given a recent position, speed and target.
// This is the same calculation done by the Arduino AccelStepper
// library.
//----------------------------------------------------------------------
void SimulatedMotors::updateStatus()
{
    QMutexLocker locker(&_mutex);

    bool prevAllMotorsIdle = _allMotorsIdle;
    _allMotorsIdle = true;

    for (unsigned int m = 0; m < _nMotors; m++) {
        if (_stepInterval[m] == 0) {
            _motorStatus[m] = MOTOR_IDLE;
            continue;
        }

        computeNewRunSpeed(m);

        if (_paused)
            _motorStatus[m] = MOTOR_PAUSED;
        else if (_homing)
            _motorStatus[m] = MOTOR_HOMING;
        else
            _motorStatus[m] = MOTOR_RUNNING;

        bool motorIdle = _currentSpeed[m] == 0.0 && _currentPos[m] == _targetPos[m];
        _allMotorsIdle &= motorIdle;
    }

    if (!prevAllMotorsIdle && _allMotorsIdle) {
        //std::cerr << "Movement finished" << std::endl;
        emit movementFinished();
    }
}

//----------------------------------------------------------------------
// Calculate the next step interval. This is the same calculation done
// by the Arduino AccelStepper library.
//----------------------------------------------------------------------
void SimulatedMotors::computeNewRunSpeed(unsigned int motorIdx)
{
    int64_t time = getMicroSecondTimeSinceStart();
    if (time - _lastStepTime[motorIdx] >= _stepInterval[motorIdx]) {

        // Move the motor
        if (_direction[motorIdx] > 0)
            _currentPos[motorIdx]++;
        else if (_direction[motorIdx] < 0)
            _currentPos[motorIdx]--;
        _lastStepTime[motorIdx] = time;

        // Recalculate step interval
        int distanceToGo = _targetPos[motorIdx] - _currentPos[motorIdx];
        int stepsToStop = static_cast<int>((_currentSpeed[motorIdx] * _currentSpeed[motorIdx]) / (2.0 * _acceleration[motorIdx]));

        if (distanceToGo == 0 && stepsToStop <= 1) {
            // Already at the target
            _stepInterval[motorIdx] = 0;
            _currentSpeed[motorIdx] = 0;
            _n[motorIdx] = 0;
            _motorStatus[motorIdx] = MOTOR_IDLE;
            return;
        }

        if (distanceToGo > 0) {
            // Target is in positive direction from current position
            if (_n[motorIdx] > 0) {
                // Currently accelerating. Motor might need to decelerate now, or may be going the wrong way.
                if (stepsToStop >= distanceToGo || _direction[motorIdx] < 0)
                    _n[motorIdx] = -stepsToStop; // Start deceleration
            }
            else if (_n[motorIdx] < 0) {
                // Currently decelerating. Motor might need to accelerate now.
                if (stepsToStop < distanceToGo && _direction[motorIdx] > 0)
                    _n[motorIdx] = -_n[motorIdx]; // Start accceleration
            }
        }
        else if (distanceToGo < 0) {
            // Target is in negative direction from current position
            if (_n[motorIdx] > 0) {
                // Currently accelerating. Motor might need to decelerate now, or may be going the wrong way.
                if (stepsToStop >= -distanceToGo || _direction[motorIdx] > 0)
                    _n[motorIdx] = -stepsToStop; // Start deceleration
            }
            else if (_n[motorIdx] < 0) {
                // Currently decelerating. Motor might need to accelerate now.
                if (stepsToStop < -distanceToGo && _direction[motorIdx] < 0)
                    _n[motorIdx] = -_n[motorIdx]; // Start accceleration
            }
        }

        // Apply acceleration and deceleration
        if (_n[motorIdx] == 0) {
            // First step from stopped
            _cn[motorIdx] = _c0[motorIdx];
            _direction[motorIdx] = (distanceToGo > 0) ? 1 : -1;
        }
        else {
            // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            _cn[motorIdx] = _cn[motorIdx] - ((2.0 * _cn[motorIdx]) / ((4.0 * _n[motorIdx]) + 1)); // Equation 13
            _cn[motorIdx] = std::max(_cn[motorIdx], _cmin[motorIdx]);
        }
        _n[motorIdx]++;
        _stepInterval[motorIdx] = static_cast<int64_t>(_cn[motorIdx] + 0.5);  // Rounded
        _currentSpeed[motorIdx] = 1000000.0 / _cn[motorIdx];
        if (_direction[motorIdx] < 0)
        _currentSpeed[motorIdx] = -_currentSpeed[motorIdx];
    }
}

//----------------------------------------------------------------------
// System-independent method to get a high-resolution system time
//----------------------------------------------------------------------
int64_t SimulatedMotors::getMicroSecondTimeSinceStart()
{
#ifdef WIN32
    LARGE_INTEGER currentCounts;  // For performance counter, because chrono high-def clock is not precise in Windows
    QueryPerformanceCounter(&currentCounts);
    int64_t timeDifference = (currentCounts.QuadPart - _startTime.QuadPart) * 1000000 / _ticksPerSecond.QuadPart;  // Microsecond time, rounded down
    return timeDifference;
#else
    std::chrono::high_resolution_clock::time_point currentTime;
    currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds timeDifference = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - _startTime);
    //std::chrono::duration<double> timeDifference = currentTime - _startTime;
    return timeDifference.count();
#endif
}
