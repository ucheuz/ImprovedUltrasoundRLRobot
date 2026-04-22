#ifndef CONTROLPANELINPUT_H
#define CONTROLPANELINPUT_H
#include <vector>

#include <QObject>
#include <QString>
#include <QThread>
#include <QPointer>
#include <QTimer>

#include "ControlPanel.h"

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// ControlPanelInput class implements the functions of InputsWindow
// using the ControlPanel class.
//======================================================================
class ControlPanelInput: public QObject
{
    Q_OBJECT

public:
    explicit ControlPanelInput(QObject* parent = nullptr);
    ~ControlPanelInput();

    enum ConnectionStatus_t {DISCONNECTED=0, CONNECT_FAILED, CONNECTED};

    void connectControlPanel();
    bool isConnected() const;

    //bool getCurrentPoseData(std::vector<double>& poseData) const;

    void setCurrentForces(const std::vector<double>& forces);

signals:
    void stopControlPanelProcess();

    void connectionStatus(QString status);

    void poseAdjustRequest(const TRTrans3D& poseAdjust, bool isLocal);

    // *** Replace most of these with a simple pose request
    void setActiveProbe(int idx, bool isActive);
    void moveX(double distance);  // X movement in base coordinates
    void moveY(double distance);  // Y movement in base coordinates
    void moveZ(double distance);  // Z movement in base coordinates

    void rotateX(double angle);  // Elevational tilt
    void rotateY(double angle);  // Axial rotation
    void rotateZ(double angle);  // Lateral tilt

    void cancel();

    // Signal sent to ControlPanel to light up LEDs
    void forceData(const std::vector<double>& forces);

private:
    QPointer<QThread> _controlPanelThread;
    QPointer<ControlPanel> _controlPanel;

    QTimer* _timer;

    ConnectionStatus_t _status;

    void disconnectControlPanel();

private slots:
    void updateTargetPose();
    //void updateButtonState();
};

#endif // CONTROLPANELINPUT_H
