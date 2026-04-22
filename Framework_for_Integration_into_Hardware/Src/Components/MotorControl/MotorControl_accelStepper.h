#ifndef MOTORCONTROL_ACCELSTEPPER_H
#define MOTORCONTROL_ACCELSTEPPER_H

#include <vector>

#include <QObject>
#include <QString>

#include "AccelStepperMotors.h"

#include "IMotorControl.h"

//======================================================================
// MotorControl_accelStepper class implements the functions of
// IMotorControl and those needed by MotorsWindow GUI using the
// AccelStepperMotors class.
//======================================================================
class MotorControl_accelStepper : public IMotorControl
{
    Q_OBJECT
    Q_INTERFACES(IMotorControl)

public:
    MotorControl_accelStepper(const QString& robotId);
    ~MotorControl_accelStepper();

    // --- Functions implemented from IMotorControl ---
    void connectMotors();

    void moveAllAbsolute(const std::vector<double>& motorPositions);
    void moveAbsolute(unsigned int idx, double motorPosition);
    void moveAllRelative(const std::vector<double>& motorDistances);
    void moveRelative(unsigned int idx, double motorDistance);

    void moveAllHome();
    void moveHome(unsigned int idx);

    void togglePause();
    void cancelMove();

    void getAllMotorPositions(std::vector<double>& currentPositions) const;
    double getMotorPosition(unsigned int idx) const;

    void getAllMotorSpeeds(std::vector<double>& currentSpeed) const;
    double getMotorSpeed(unsigned int idx) const;
    void getAllStatusStrings(std::vector<QString>& status) const;
    QString getStatusString(unsigned int idx) const;
    void getAllMotorParams(std::vector<float>& maxSpeed, std::vector<float>& acceleration) const;

    // --- Functions required by GUI ---
    void send(QString message);

signals:
    void accelStepperStopMotorProcess();
    void accelStepperTogglePause();
    void accelStepperCancelMove();
    void accelStepperSend(QString arduinoId, QString message);

    void accelStepperMoveAllAbsolute(QString arduinoId, std::vector<double> pos, double speedScale);
    void accelStepperMoveAbsolute(QString arduinoId, unsigned int idx, double pos, double speedScale);

    void accelStepperMoveSeveralHome(QString arduinoId, std::vector<unsigned int> motorsToMove, double speedScale);
    void accelStepperMoveHome(QString arduinoId, unsigned int idx, double speedScale);

private:
    QPointer<QThread> _accelStepperThread;
    QPointer<AccelStepperMotors> _accelStepper;

    void disconnectMotors();
};

#endif // MOTORCONTROL_ACCELSTEPPER_H
