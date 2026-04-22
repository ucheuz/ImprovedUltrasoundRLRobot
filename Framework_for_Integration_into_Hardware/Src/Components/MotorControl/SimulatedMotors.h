#ifndef SIMULATEDMOTORS_H
#define SIMULATEDMOTORS_H

#include <vector>
#ifdef WIN32
  #include "Windows.h"  // For high-resolution counter, because chrono high-def clock is not precise in Windows
#else
  #include <chrono>
#endif

#include <QObject>
#include <QString>
#include <QtSerialPort/QtSerialPort>
#include <QMutex>

//#define WRITE_TO_FILE
#ifdef WRITE_TO_FILE
#include <QFile>
#include <QTextStream>
#include <QTimer>
#endif  // WRITE_TO_FILE

//======================================================================
// SimulatedMotors class simulates the AccelStepperMotors class without
// requiring conection to real motors. It reproduces some of the timing
// calculations that would usually happen on the motor control Teensy.
//======================================================================
class SimulatedMotors : public QObject
{
  Q_OBJECT
public:
    explicit SimulatedMotors(unsigned int nMotors);
    ~SimulatedMotors();

    // Possible stepper motor states
    enum MotorStatus_t {MOTOR_DISCONNECTED=0, MOTOR_IDLE, MOTOR_RUNNING, MOTOR_HOMING, MOTOR_PAUSED, MOTOR_UNKNOWN};

    // Set movement scaling (steps per degree or mm)
    void setStepsPerDegree(std::vector<double> stepsPerDegree);
    void setHomePosition(std::vector<double> homePos);

    // Access functions
    void getAllStatuses(std::vector<int>& status) const;
    QString getStatusString(unsigned int idx) const;
    void getAllPositions(std::vector<double>& pos) const;
    double getPosition(unsigned int idx) const;
    void getAllSpeeds(std::vector<double>& speed) const;
    double getSpeed(unsigned int idx) const;
    void getAllMotorParams(std::vector<float>& maxSpeed, std::vector<float>& acceleration) const;

    static QString statusToStatusString(int statusNum);

    unsigned int getNMotors() const;

    bool isConnected() const;

public slots:
    // Thread functions
    void run();
    void stop();

    // General motor control
    void togglePause();
    void cancelMove();

    void moveAllAbsolute(std::vector<double> pos, double speedScale);
    void moveAbsolute(unsigned int idx, double pos, double speedScale);

    void moveSeveralHome(std::vector<unsigned int> motorsToMove);
    void moveHome(unsigned int idx);

signals:
    void finished();
    void movementFinished();

private:
    QMutex mutable _mutex;

    bool _running;
    bool _paused;
    bool _homing;

    unsigned int _nMotors;
    std::vector<MotorStatus_t> _motorStatus;
    bool _allMotorsIdle;
    std::vector<int> _currentPos;  // Steps
    std::vector<int> _targetPos;
    std::vector<int> _origTargetPos;
    std::vector<double> _currentSpeed;  // Steps / s
    std::vector<double> _currentMaxSpeed;
    std::vector<double> _posScale;  // Steps per degree or mm
    std::vector<double> _posOffset;  // Home position offset in conversion from degrees or mm to steps

    // Fixed motor parameters
    std::vector<double> _maxSpeed;
    std::vector<double> _acceleration;

    // Parameters copied from Arudino AccelStepper library
    std::vector<int64_t> _stepInterval;
    std::vector<int64_t> _lastStepTime;
    std::vector<int8_t> _direction;
    std::vector<int> _n;        // Step counter
    std::vector<double> _cn;    // Step interval
    std::vector<double> _c0;    // Starting step interval
    std::vector<double> _cmin;  // Minimum step interval

#ifdef WIN32
    LARGE_INTEGER _ticksPerSecond;
    LARGE_INTEGER _startTime;
#else
    std::chrono::high_resolution_clock::time_point _startTime;
#endif

#ifdef WRITE_TO_FILE
    QFile _outFile;
    QTextStream _fout;
    QTimer* _loggingTimer;

    void logStatus();
#endif  // WRITE_TO_FILE

    void updateCurrentMaxSpeeds(double speedScale);
    void updateStatus();
    void computeNewRunSpeed(unsigned int motorIdx);
    int64_t getMicroSecondTimeSinceStart();
};

#endif // SIMULATEDMOTORS_H
