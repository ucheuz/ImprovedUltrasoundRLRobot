#ifndef IMOTORCONTROL_H
#define IMOTORCONTROL_H

#include <vector>

#include <QObject>
#include <QtPlugin>
#include <QString>

//======================================================================
// IMotorControl is a base class for various types of motor control. It
// provides functions for accessing and controlling motor positions and
// other motor control functions.
// Currently available motor controllers are:
//   MotorControl_accelStepper using AccelStepper motors library
//======================================================================
class IMotorControl : public QObject
{
    Q_OBJECT

public:
    IMotorControl();
    virtual ~IMotorControl();

    enum ConnectionStatus_t {INVALID_SETUP=0, DISCONNECTED, CONNECT_FAILED, CONNECTED_PARTIAL, CONNECTED};

    virtual void connectMotors() = 0;
    bool isConnected() const;

    int getNMotors() const;

    // Set positions for motor to return to after homing
    void setNeutralPositions(std::vector<double>& neutralPositions);
    // Set speed scale between 0.1 and 1.0 of maximum (default is 1.0)
    void setSpeedScale(double speedScale);

    // Move motors
    virtual void moveAllAbsolute(const std::vector<double>& motorPositions) = 0;
    virtual void moveAbsolute(unsigned int idx, double motorPosition) = 0;
    virtual void moveAllRelative(const std::vector<double>& motorDistances) = 0;
    virtual void moveRelative(unsigned int idx, double motorDistance) = 0;

    // Motor homing
    bool isHomeable(unsigned int idx) const;  // Check whether the motor has a homing sensor
    virtual void moveAllHome() = 0;
    virtual void moveHome(unsigned int idx) = 0;

    // Pause / stop motors
    virtual void togglePause() = 0;
    virtual void cancelMove() = 0;

    // Get positions
    bool isIdle() const;
    virtual void getAllMotorPositions(std::vector<double>& currentPositions) const = 0;
    virtual double getMotorPosition(unsigned int idx) const = 0;

    // Information to display in GUI
    virtual void getAllMotorSpeeds(std::vector<double>& currentSpeed) const = 0;
    virtual double getMotorSpeed(unsigned int idx) const = 0;
    virtual void getAllStatusStrings(std::vector<QString>& status) const = 0;
    virtual QString getStatusString(unsigned int idx) const = 0;
    virtual void getAllMotorParams(std::vector<float>& maxSpeed, std::vector<float>& acceleration) const = 0;

signals:
    void connectionChanged();
    void connectionStatus(QString status);

    void cancelSequence();

public slots:
    void goIdle();

protected:
    std::vector<QString> _arduinoIds;
    int _nMotors;

    std::vector<double> _stepsPerDegree;

    std::vector<double> _homePos;
    std::vector<bool> _homingSensor;
    std::vector<unsigned int> _homingOrder;

    std::vector<double> _neutralPos;

    ConnectionStatus_t _status;

    bool _moving;
    bool _cancelSequence;

    double _speedScale;

    void initialise(const QString& robotId);
};

Q_DECLARE_INTERFACE(IMotorControl, "Interface.IMotorControl")

#endif // IMOTORCONTROL_H
