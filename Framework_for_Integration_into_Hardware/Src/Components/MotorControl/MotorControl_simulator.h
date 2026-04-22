#ifndef MOTORCONTROL_SIMULATOR_H
#define MOTORCONTROL_SIMULATOR_H

#include <QObject>

#include "SimulatedMotors.h"

#include "IMotorControl.h"

//======================================================================
// MotorControl_simulator class implements the functions of
// IMotorControl for when there are no motors attached.
//======================================================================
class MotorControl_simulator : public IMotorControl
{
    Q_OBJECT
    Q_INTERFACES(IMotorControl)

public:
    MotorControl_simulator(const QString& robotId);
    ~MotorControl_simulator();

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

signals:
    void motorSimStopMotorProcess();
    void motorSimTogglePause();
    void motorSimCancelMove();

    void motorSimMoveAllAbsolute(std::vector<double> pos, double speedScale);
    void motorSimMoveAbsolute(unsigned int idx, double pos, double speedScale);

    void motorSimMoveSeveralHome(std::vector<unsigned int> motorsToMove);
    void motorSimMoveHome(unsigned int idx);

private:
    QPointer<QThread> _motorSimThread;
    QPointer<SimulatedMotors> _motorSim;

    void disconnectMotors();
};

#endif // MOTORCONTROL_SIMULATOR_H
