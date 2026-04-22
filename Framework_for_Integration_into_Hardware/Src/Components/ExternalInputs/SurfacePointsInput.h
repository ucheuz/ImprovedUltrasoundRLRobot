#ifndef SURFACEPOINTSINPUT_H
#define SURFACEPOINTSINPUT_H

#include <vector>

#include <QObject>
#include <QString>
#include <QThread>
#include <QPointer>
#include <QTimer>

#include "SurfacePointsData.h"

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// SurfacePointsInput class implements the functions of InputsWindow
// using the surface points data external interface class.
//======================================================================
class SurfacePointsInput : public QObject
{
    Q_OBJECT

public:
    explicit SurfacePointsInput(QObject* parent = nullptr);
    ~SurfacePointsInput();

    enum ConnectionStatus_t {DISCONNECTED=0, CONNECT_FAILED, CONNECTED};

    void connectSurfPointsData();
    bool isConnected() const;

    void setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses);

    void setIpAddress(const QHostAddress& address);
    void setPort(unsigned short port);

    bool getCurrentPoseData(std::vector<double>& poseData) const;
    //bool getPoseMatrix(TRTrans3D& pose) const;

    void setCurrentForce(std::vector<double>& forceData);

signals:
    void stopSurfPointsDataProcess();

    //void connectionChanged();
    void connectionStatus(QString status);

    void singleProbePoseRequest(int idx, const TRTrans3D& pose);
    void multiProbePoseRequest(const std::vector<TRTrans3D>& poses);  // *** Not currently used

private:
    QPointer<QThread> _surfPointsDataThread;
    QPointer<SurfacePointsData> _surfPointsData;

    ConnectionStatus_t _status;

    std::vector<TRTrans3D> _neutralPoses;

    QHostAddress _surfPointsDataIpAddress;
    unsigned short _receivePort;

    unsigned int _controlledProbe;

    void disconnectSurfPointsData();

private slots:
    void updateTargetPose();
};

#endif // SURFACEPOINTSINPUT_H
