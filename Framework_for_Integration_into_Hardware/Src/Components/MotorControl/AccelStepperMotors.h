#ifndef ACCELSTEPPERMOTORS_H
#define ACCELSTEPPERMOTORS_H

#include <vector>

#include <QObject>
#include <QString>
#include <QtSerialPort/QtSerialPort>
#include <QMutex>
	
//======================================================================
// AccelStepperMotors class handles communication with the stepper motor
// master Teensy board via a QSerialPort. Functions control stepper
// motors via custom commands and continually request status updates
// from the Teensy.
//======================================================================
class AccelStepperMotors : public QObject
{
  Q_OBJECT
public:
    explicit AccelStepperMotors(QString arduinoId, unsigned int nMotors);
    ~AccelStepperMotors();

    // Possible stepper motor states
    enum MotorStatus_t {MOTOR_DISCONNECTED=0, MOTOR_IDLE, MOTOR_RUNNING, MOTOR_HOMING, MOTOR_PAUSED, MOTOR_UNKNOWN};

    bool initialise();

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
    void send(QString arduinoId, QString message);

    void moveAllAbsolute(QString arduinoId, std::vector<double> pos, double speedScale);
    void moveAbsolute(QString arduinoId, unsigned int idx, double pos, double speedScale);

    void moveSeveralHome(QString arduinoId, std::vector<unsigned int> motorsToMove, double speedScale);
    void moveHome(QString arduinoId, unsigned int idx, double speedScale);

signals:
    void finished();
    void movementFinished(QString arduinoId);

private:
    QSerialPort _serialPort;
    QString _arduinoId;

    QTimer* _requestStatusTimer;

    uint32_t _requestId;
    uint32_t _latestMovementRequestId;
    uint32_t _latestStatusRequestId;
    uint32_t _latestStatusReceivedId;
    int64_t _latestStatusRequestTime;

    QMutex mutable _mutex;

    bool _running;

    unsigned int _nMotors;
    std::vector<MotorStatus_t> _motorStatus;
    bool _allMotorsIdle;
    std::vector<double> _mPos;  // Degrees or mm
    std::vector<int> _intTargetPos;  // Steps
    std::vector<double> _speed;  // Degrees or mm per second
    std::vector<double> _posScale;  // Steps per degree or mm
    std::vector<double> _posOffset;  // Home position offset in conversion from degrees or mm to steps

    // Motor parameters
    std::vector<int> _maxSpeed;
    std::vector<int> _acceleration;

    void connectToPort(QSerialPortInfo portInfo);

private slots:
    void requestStatus();
    void serialPort_dataReceived();
};

#endif // ACCELSTEPPERMOTORS_H
