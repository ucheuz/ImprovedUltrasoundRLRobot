#ifndef IGTLINPUT_H
#define IGTLINPUT_H

#include <thread>
#include <vector>

#include <QObject>

#ifndef WIN32
#include "igtliFindSessionManager.h"  // From IGTLController library
#endif  // !WIN32

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// IGTLInput class runs the IGTL communication thread and stores an up
// to date version of the requested position. There are also functions
// for sending force and current position data back to the source.
// The functionality is not available in Windows as the server
// implementation relies on pthread.
//======================================================================
class IGTLInput : public QObject
{
    Q_OBJECT
public:
    explicit IGTLInput(QObject *parent = nullptr);
    ~IGTLInput();

    enum ConnectionStatus_t {DISCONNECTED=0, CONNECT_FAILED, CONNECTED};

    void connectIGTL();
    bool isConnected() const;

    void setCentrePose(const TRTrans3D& centrePose);

    void setPort(unsigned int port);

    bool getCurrentPoseData(int idx, std::vector<double>& poseData) const;
    //bool getPoseMatrix(TRTrans3D& pose) const;

    void setCurrentPoses(const std::vector<TRTrans3D>& probePoses);
    //void setCurrentForce(const std::vector<double>& forceData);  // Not implemented in IGTLController

signals:
    //void connectionChanged();
    void connectionStatus(QString status);

    void singleProbePoseRequest(int idx, const TRTrans3D& pose);  // Not currently used
    void multiProbePoseRequest(const std::vector<TRTrans3D>& poses);

private:
#ifndef WIN32
    // IGTL server for sending tracking data
    igtl::iFindSessionManager::Pointer _trackingDataServer;
#endif  // !WIN32

    ConnectionStatus_t _status;

    TRTrans3D _centrePose;

    unsigned int _receivePort;

    static const int _nDofs = 7;
    static const int _maxProbes = 4;
    double _poseData[_maxProbes*_nDofs];

    void disconnectIGTL();

    std::thread* serverthread;
private slots:
#ifndef WIN32
    void positionRequestReceived(igtl::TrackingDataMessage::Pointer position);
#endif  // !WIN32
};

#endif // IGTLINPUT_H
