#ifndef SPACEMOUSEINPUT_H
#define SPACEMOUSEINPUT_H

#include <vector>

#include <QObject>
#include <QString>
#include <QThread>
#include <QPointer>
#include <QTimer>

#include "SpaceMouse.h"

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// SpaceMouseInput class implements the functions of InputsWindow using
// the SpaceMouse class.
//======================================================================
class SpaceMouseInput : public QObject
{
    Q_OBJECT

public:
    explicit SpaceMouseInput(QObject* parent = nullptr);
    ~SpaceMouseInput();

    enum ConnectionStatus_t {DISCONNECTED=0, CONNECT_FAILED, CONNECTED};

    void connectSpaceMouse();
    bool isConnected() const;

    bool getCurrentPoseData(std::vector<double>& poseData) const;

signals:
    void stopSpaceMouseProcess();

    void connectionStatus(QString status);

    void poseAdjustRequest(const TRTrans3D& poseAdjust, bool isLocal);

private:
    QPointer<QThread> _spaceMouseThread;
    QPointer<SpaceMouse> _spaceMouse;

    QTimer* _timer;

    ConnectionStatus_t _status;

    void disconnectSpaceMouse();

private slots:
    void updateTargetPose();
};

#endif // SPACEMOUSEINPUT_H
