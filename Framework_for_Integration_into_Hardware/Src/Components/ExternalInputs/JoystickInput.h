#ifndef JOYSTICKINPUT_H
#define JOYSTICKINPUT_H

#include <vector>

#include <QObject>
#include <QString>
#include <QThread>
#include <QPointer>
#include <QTimer>

#include "Joystick.h"

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// JoystickInput class implements the functions of InputsWindow using
// the Joystick class.
//======================================================================
class JoystickInput : public QObject
{
    Q_OBJECT

public:
    explicit JoystickInput(QObject* parent = nullptr);
    ~JoystickInput();

    enum ConnectionStatus_t {DISCONNECTED=0, CONNECT_FAILED, CONNECTED};

    void connectJoystick();
    bool isConnected() const;

    void setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses);

    void setIpAddress(const QHostAddress& address);
    void setPort(unsigned short port);

    bool getCurrentPoseData(std::vector<double>& poseData) const;
    //bool getPoseMatrix(TRTrans3D& pose) const;

    void setCurrentForce(const std::vector<double>& forceData);

signals:
    void stopJoystickProcess();

    //void connectionChanged();
    void connectionStatus(QString status);

    void forceData(const std::vector<double>& force);

    void singleProbePoseRequest(int idx, const TRTrans3D& pose);
    void multiProbePoseRequest(const std::vector<TRTrans3D>& poses);  // *** Not currently used

private:
    QPointer<QThread> _joystickThread;
    QPointer<Joystick> _joystick;

    QTimer* _timer;

    ConnectionStatus_t _status;

    std::vector<TRTrans3D> _neutralPoses;

    QHostAddress _joystickIpAddress;
    unsigned short _receivePort;

    int _controlledProbe;

    void disconnectJoystick();

private slots:
    void updateTargetPose();
};

#endif // JOYSTICKINPUT_H
