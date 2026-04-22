#include <iostream>
#include <sstream>

#include "AccelStepperMotors.h"

#include <qdebug.h>
#include <QDateTime>

//#define NO_MOTOR_TEST  // Obsolete motor simulator mode; replaced by MotorControl_simulator class
//#define PRINT_SERIAL

//#include <chrono>
//extern std::chrono::high_resolution_clock::time_point startTime, endTime;

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
AccelStepperMotors::AccelStepperMotors(QString arduinoId, unsigned int nMotors)
{
    _arduinoId = arduinoId;
    _nMotors = nMotors;

    _running = false;
    _allMotorsIdle = false;

    _motorStatus.resize(_nMotors, MOTOR_DISCONNECTED);
    _mPos.resize(_nMotors, 0.0);
    _intTargetPos.resize(_nMotors, 0);
    _speed.resize(_nMotors, 0.0);
    _posScale.resize(_nMotors, 1.0);
    _posOffset.resize(_nMotors, 0.0);

    _maxSpeed.resize(_nMotors, 0);
    _acceleration.resize(_nMotors, 0);

    // Timer event causes request of current motor status
    // This prevents too many requests being sent to the Teensy
    _requestStatusTimer = new QTimer(this);
    //_requestStatusTimer->start(200);
    _requestStatusTimer->start(0);
    _requestId = 0;
    _latestMovementRequestId = 0;
    _latestStatusRequestId = 0;
    _latestStatusReceivedId = 0;
    _latestStatusRequestTime = 0;

    _serialPort.setParent(this);
    connect(&_serialPort, &QSerialPort::readyRead, this, &AccelStepperMotors::serialPort_dataReceived);
}

AccelStepperMotors::~AccelStepperMotors()
{
    if (_serialPort.isOpen())
        _serialPort.close();
}

//----------------------------------------------------------------------
// Locate arduino by ID and connect to port
//----------------------------------------------------------------------
bool AccelStepperMotors::initialise()
{
    if (_serialPort.isOpen())
        _serialPort.close();

    bool portFound = false;
    QSerialPortInfo portInfo;

    QList<QSerialPortInfo> portList = QSerialPortInfo::availablePorts();
    for (int i = 0; i < portList.length(); i++) {
        //std::cerr << portList[i].description().toStdString() << " | " << portList[i].manufacturer().toStdString() << " | " << portList[i].serialNumber().toStdString() << std::endl;
        if (QString::compare(portList[i].serialNumber(), _arduinoId) == 0) {
            portFound = true;
            portInfo = portList[i];
            break;
        }
    }

    if (portFound)
        connectToPort(portInfo);

#ifdef NO_MOTOR_TEST
    return true;
#else
    return _serialPort.isOpen();
#endif
}

//----------------------------------------------------------------------
// Set movement scale
//----------------------------------------------------------------------
void AccelStepperMotors::setStepsPerDegree(std::vector<double> stepsPerDegree)
{
    if (stepsPerDegree.size() != _nMotors) {
        std::cerr << "Error in AccelStepperMotors::setStepsPerDegree: wrong input array size." << std::endl;
        return;
    }

    for (unsigned int i = 0; i < _nMotors; i++)
        _posScale[i] = stepsPerDegree[i];
}

void AccelStepperMotors::setHomePosition(std::vector<double> homePos)
{
    if (homePos.size() != _nMotors) {
        std::cerr << "Error in AccelStepperMotors::setHomePosition: wrong input array size." << std::endl;
        return;
    }

    for (unsigned int i = 0; i < _nMotors; i++)
        _posOffset[i] = homePos[i];
}

//----------------------------------------------------------------------
// Access functions
//----------------------------------------------------------------------
void AccelStepperMotors::getAllPositions(std::vector<double>& pos) const
{
    pos.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        pos[i] = _mPos[i];
}

double AccelStepperMotors::getPosition(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return _mPos[idx];
    }
    else {
        std::cerr << "Error in AccelStepperMotors::getPosition: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return 0.0;
    }
}

void AccelStepperMotors::getAllSpeeds(std::vector<double>& speed) const
{
    speed.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        speed[i] = _speed[i];
}

double AccelStepperMotors::getSpeed(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return _speed[idx];
    }
    else {
        std::cerr << "Error in AccelStepperMotors::getSpeed: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return 0.0;
    }
}

void AccelStepperMotors::getAllStatuses(std::vector<int>& status) const
{
    status.resize(_nMotors);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nMotors; i++)
        status[i] = _motorStatus[i];
}

QString AccelStepperMotors::getStatusString(unsigned int idx) const
{
    if (idx < _nMotors) {
        QMutexLocker locker(&_mutex);
        return statusToStatusString(_motorStatus[idx]);
    }
    else {
        std::cerr << "Error in AccelStepperMotors::getStatusString: idx out of range (0-" << _nMotors-1 << ")." << std::endl;
        return "";
    }
}

void AccelStepperMotors::getAllMotorParams(std::vector<float> &maxSpeed, std::vector<float> &acceleration) const
{
    maxSpeed.resize(_nMotors);
    acceleration.resize(_nMotors);

    for (unsigned int i = 0; i < _nMotors; i++) {
        maxSpeed[i] = _maxSpeed[i];
        acceleration[i] = _acceleration[i];
    }
}

QString AccelStepperMotors::statusToStatusString(int status)
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

unsigned int AccelStepperMotors::getNMotors() const
{
    return _nMotors;
}

bool AccelStepperMotors::isConnected() const
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
// Thread process function for motor monitoring
//----------------------------------------------------------------------
void AccelStepperMotors::run()
{
    _running = true;

    connect(_requestStatusTimer, &QTimer::timeout, this, &AccelStepperMotors::requestStatus);

    // Initial long pause to give Teensy time to wake up
    requestStatus();
    QThread::msleep(1000);
    qApp->processEvents();

    while (_running) {
        qApp->processEvents();
    }

    disconnect(_requestStatusTimer, nullptr, this, nullptr);

    if (_serialPort.isOpen())
        _serialPort.close();

    emit finished();
}

//----------------------------------------------------------------------
// Thread process stop function
//----------------------------------------------------------------------
void AccelStepperMotors::stop()
{
    cancelMove();
    _running = false;
}

//----------------------------------------------------------------------
// General motor control
//----------------------------------------------------------------------
void AccelStepperMotors::togglePause()
{
    if (_serialPort.isOpen())
        _serialPort.write("!\n");
}

void AccelStepperMotors::cancelMove()
{
    if (_serialPort.isOpen())
        _serialPort.write("&\n");
}

void AccelStepperMotors::send(QString arduinoId, QString message)
{
    if (_arduinoId.compare(arduinoId) != 0)
        return;

    if (_serialPort.isOpen()) {
        message.append('\n');
        std::cerr << message.toStdString();
        _serialPort.write(message.toStdString().c_str());
    }
}

//----------------------------------------------------------------------
// Slot for motor movement - all motors
// Expected format:
// t1:<speedScale>;{<intPos_1>,<intPos_2>,...,<intPos_nMotors>}
//----------------------------------------------------------------------
void AccelStepperMotors::moveAllAbsolute(QString arduinoId, std::vector<double> pos, double speedScale)
{
    if (_arduinoId.compare(arduinoId) != 0)
        return;

#ifdef NO_MOTOR_TEST
    {
        QMutexLocker locker(&_mutex);
        for (unsigned int i = 0; i < _nMotors; i++)
            _intTargetPos[i] = int((pos[i] - _posOffset[i]) * _posScale[i] + 0.5);
        _allMotorsIdle = false;  // Assume move has started

        //endTime = std::chrono::high_resolution_clock::now();
        //std::chrono::duration<float> timeDuration = endTime - startTime;
        //double timeDiff = timeDuration.count();
        //std::cerr << "Movement happened after " << timeDiff << " seconds." << std::endl;
    }
#else
    if (_serialPort.isOpen()) {
        QMutexLocker locker(&_mutex);
        std::ostringstream oss;
        oss << "t1:" << speedScale << ";";
        for (unsigned int i = 0; i < _nMotors; i++) {
            int intPos = int((pos[i] - _posOffset[i]) * _posScale[i] + 0.5);
            if (i < _nMotors - 1)
                oss << intPos << ",";
            else
                oss << intPos << "\n";
            _intTargetPos[i] = intPos;
        }
#ifdef PRINT_SERIAL
        std::cerr << oss.str();
#endif  // PRINT_SERIAL
        if (oss.str().length() > 0) {
            _requestId++;
            _latestMovementRequestId = _requestId;
            _serialPort.write(oss.str().c_str());
            _serialPort.write("g\n");
            _allMotorsIdle = false;  // Assume move has started
        }
    }
    else
        std::cerr << "AccelStepperMotors::moveAllAbsolute: serial port is not open" << std::endl;
#endif
}

//----------------------------------------------------------------------
// Slot for motor movement - single motor
// Expected format:
// t2:<speedScale>;<motorIdx>,<intPos>
//----------------------------------------------------------------------
void AccelStepperMotors::moveAbsolute(QString arduinoId, unsigned int idx, double pos, double speedScale)
{
    if (_arduinoId.compare(arduinoId) != 0)
        return;

    if (idx >= _nMotors) {
        std::cerr << "Error in AccelStepperMotors::moveAbsolute: idx is out of range (0-" << _nMotors-1 << ")." << std::endl;
        return;
    }

#ifdef NO_MOTOR_TEST
    {
        QMutexLocker locker(&_mutex);
        _intTargetPos[idx] = int((pos - _posOffset[idx]) * _posScale[idx] + 0.5);
        _allMotorsIdle = false;  // Assume move has started
    }
#else
    if (_serialPort.isOpen()) {
        QMutexLocker locker(&_mutex);
        int intPos = int((pos - _posOffset[idx]) * _posScale[idx] + 0.5);
        _intTargetPos[idx] = intPos;
        std::ostringstream oss;
        oss << "t2:" << speedScale << ";" << idx << "," << intPos << "\n";
#ifdef PRINT_SERIAL
        std::cerr << oss.str();
#endif  // PRINT_SERIAL
        if (oss.str().length() > 0) {
            _requestId++;
            _latestMovementRequestId = _requestId;
            _serialPort.write(oss.str().c_str());
            _serialPort.write("g\n");
            _allMotorsIdle = false;  // Assume move has started
        }
    }
    else
        std::cerr << "AccelStepperMotors::moveAbsolute: serial port is not open" << std::endl;
#endif
}

//----------------------------------------------------------------------
// Slot for motor homing - several motors
//----------------------------------------------------------------------
void AccelStepperMotors::moveSeveralHome(QString arduinoId, std::vector<unsigned int> motorsToMove, double speedScale)
{
    if (_arduinoId.compare(arduinoId) != 0)
        return;

    for (unsigned int i = 0; i < motorsToMove.size(); i++) {
        if (motorsToMove[i] >= _nMotors) {
            std::cerr << "Error in AccelStepperMotors::moveSeveralHome: index is out of range." << std::endl;
            return;
        }
    }

#ifdef NO_MOTOR_TEST
    {
        QMutexLocker locker(&_mutex);
        for (unsigned int i = 0; i < motorsToMove.size(); i++)
            _intTargetPos[motorsToMove[i]] = 0;
        _allMotorsIdle = false;  // Assume move has started
    }
#else
    if (_serialPort.isOpen()) {
        QMutexLocker locker(&_mutex);
        std::ostringstream oss;
        for (unsigned int i = 0; i < motorsToMove.size(); i++) {
            oss << "h:" << speedScale << ";" << motorsToMove[i] << "\n";
            _intTargetPos[i] = 0;
        }
#ifdef PRINT_SERIAL
        std::cerr << oss.str();
#endif  // PRINT_SERIAL
        if (oss.str().length() > 0) {
            _requestId++;
            _latestMovementRequestId = _requestId;
            _serialPort.write(oss.str().c_str());
            _serialPort.write("g\n");
            _allMotorsIdle = false;  // Assume move has started
            //qDebug() << "Homing request";
        }
    }
    else
        std::cerr << "AccelStepperMotors::moveSeveralHome: serial port is not open" << std::endl;
#endif
}

//----------------------------------------------------------------------
// Slot for motor homing - single motor
// Expected format:
// h:<speedScale>;<motorIdx>
//----------------------------------------------------------------------
void AccelStepperMotors::moveHome(QString arduinoId, unsigned int idx, double speedScale)
{
    if (_arduinoId.compare(arduinoId) != 0)
        return;

    if (idx >= _nMotors) {
        std::cerr << "Error in AccelStepperMotors::moveHome: idx is out of range (0-" << _nMotors-1 << ")." << std::endl;
        return;
    }

#ifdef NO_MOTOR_TEST
    {
        QMutexLocker locker(&_mutex);
        _intTargetPos[idx] = 0;
        _allMotorsIdle = false;  // Assume move has started
    }
#else
    if (_serialPort.isOpen()) {
        QMutexLocker locker(&_mutex);
        _intTargetPos[idx] = 0;
        std::ostringstream oss;
        oss << "h:" << speedScale << ";" << idx << "\n";
#ifdef PRINT_SERIAL
        std::cerr << oss.str();
#endif  // PRINT_SERIAL
        if (oss.str().length() > 0) {
            _requestId++;
            _latestMovementRequestId = _requestId;
            _serialPort.write(oss.str().c_str());
            _serialPort.write("g\n");
            _allMotorsIdle = false;  // Assume move has started
        }
    }
    else
        std::cerr << "AccelStepperMotors::moveHome: serial port is not open" << std::endl;
#endif
}


//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Port connection
//----------------------------------------------------------------------
void AccelStepperMotors::connectToPort(QSerialPortInfo portInfo)
{
    _serialPort.setPortName(portInfo.portName());
    std::cerr << "Connecting to port " << _serialPort.portName().toStdString() << std::endl;

    if (!_serialPort.open(QIODevice::ReadWrite)) {
        std::cerr << "Failed to open port " << _serialPort.portName().toStdString() << ". " << _serialPort.errorString().toStdString() << std::endl;
        return;
    }

    if (_serialPort.isOpen()) {
        // Default settings for Arduino
        //_serialPort.setBaudRate(QSerialPort::Baud115200);
        _serialPort.setBaudRate(250000);
        _serialPort.setParity(QSerialPort::NoParity);
        _serialPort.setDataBits(QSerialPort::Data8);
        _serialPort.setStopBits(QSerialPort::OneStop);
    }

    // Reset
    cancelMove();

    // Update local parameter information
    //if (_serialPort.isOpen())
    //    _serialPort.write("s\n");
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Called repeatedly by QTimer. Requests current status from the
// Arduino. The response is handled by serialPort_dataReceived().
//----------------------------------------------------------------------
void AccelStepperMotors::requestStatus()
{
    QMutexLocker locker(&_mutex);  // *** Not sure if this is needed. It depends whether timer events can interrupt other functions in this thread, so that _requestId and _latestStatusRequestId need to be protected.

    // Don't send another status request if we're still waiting for old status requests,
    // unless more than a threshold time has passed because then the status info is probably lost
    int64_t time = QDateTime::currentMSecsSinceEpoch();
    if (_latestStatusReceivedId < _latestStatusRequestId && time - _latestStatusRequestTime < 200)
        return;

    //qDebug() << time - _latestStatusRequestTime << _latestStatusReceivedId << _latestStatusRequestId;

    _requestId++;
    _latestStatusRequestId = _requestId;
    _latestStatusRequestTime = time;
#ifdef NO_MOTOR_TEST
    bool previouslyIdle = _allMotorsIdle;
    for (unsigned int i = 0; i < _nMotors; i++) {
        // Assume any requested move has finished
        _motorStatus[i] = MOTOR_IDLE;
        _mPos[i] = _intTargetPos[i] / _posScale[i] + _posOffset[i];
        _speed[i] = 0.0;
        _maxSpeed[i] = 15000;
        _acceleration[i] = 30000;
    }
    _allMotorsIdle = true;

    if (!previouslyIdle && _allMotorsIdle) {
        //QThread::msleep(250);  // To simulate time for motors to move
        emit movementFinished(_arduinoId);
    }
#else
    if (_serialPort.isOpen()) {
        // Query Arduino and read current values in serialPort_dataReceived()
        std::ostringstream oss;
        oss << "?" << _requestId << "\n";
        _serialPort.write(oss.str().c_str());
    }
    //qDebug() << "Status request " << _requestId;
#endif
}

//----------------------------------------------------------------------
// Serial port receive slot.
// expected format:
// <status_0,pos_0,speed_0,maxSpeed_0,acceleration_0;status_1,...,acceleration_{nMotors-1}>
//----------------------------------------------------------------------
void AccelStepperMotors::serialPort_dataReceived()
{
    while (_serialPort.canReadLine()) {

        QByteArray data = _serialPort.readLine();
        QString dataStr(data);

        //int64_t time = QDateTime::currentMSecsSinceEpoch();
        //static int64_t prevTime;
        //qDebug() << "Time: " << time - prevTime;
        //prevTime = time;

        if (dataStr.startsWith("<")) {
            QList<QString> words = dataStr.split(QRegularExpression("[<:;,>]"));

            bool ok;

            if (words.length() == 5*_nMotors + 4) {

                QMutexLocker locker(&_mutex);

                uint32_t requestId = words[1].toUInt(&ok);
                _latestStatusReceivedId = requestId;
                if (!ok || requestId <= _latestMovementRequestId || requestId < _latestStatusRequestId)
                    continue;

                //if (words.length() != 5*_nMotors + 4)
                //    continue;

                //qDebug() << dataStr;

                bool previouslyIdle = _allMotorsIdle;

                int iTmp;
                float fTmp;
                _allMotorsIdle = true;
                for (unsigned int i = 0; i < _nMotors; i++) {
                    // Motor status
                    iTmp = words[5*i+2].toInt(&ok);
                    if (ok) {
                        if (iTmp == 0)
                            _motorStatus[i] = MOTOR_IDLE;
                        else if (iTmp == 1)
                            _motorStatus[i] = MOTOR_RUNNING;
                        else if (iTmp == 2)
                            _motorStatus[i] = MOTOR_HOMING;
                        else if (iTmp == 3)
                            _motorStatus[i] = MOTOR_PAUSED;
                        else
                            _motorStatus[i] = MOTOR_UNKNOWN;
                    }
                    else
                        _motorStatus[i] = MOTOR_UNKNOWN;
                    // Motor position
                    iTmp = words[5*i+3].toInt(&ok);
                    if (ok)
                        _mPos[i] = double(iTmp / _posScale[i] + _posOffset[i]);
                    else
                        _mPos[i] = 0.0;
                    // Motor speed
                    fTmp = words[5*i+4].toFloat(&ok);
                    if (ok)
                        _speed[i] = double(fTmp) / _posScale[i];
                    else
                        _speed[i] = 0.0;
                    // Motor max speed
                    fTmp = words[5*i+5].toFloat(&ok);
                    if (ok)
                        _maxSpeed[i] = static_cast<int>(fTmp);
                    else
                        _maxSpeed[i] = 0;
                    // Motor acceleration
                    fTmp = words[5*i+6].toFloat(&ok);
                    if (ok)
                        _acceleration[i] = static_cast<int>(fTmp);
                    else
                        _acceleration[i] = 0;

                    if (_motorStatus[i] != MOTOR_IDLE)
                        _allMotorsIdle = false;
                }

                //qDebug() << "Status update " << requestId;

                if (!previouslyIdle && _allMotorsIdle) {
                    emit movementFinished(_arduinoId);
                    //qDebug() << "  Movement finished";
                }
            }
        }
    }
}
