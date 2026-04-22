#ifndef SURFACEPOINTSDATA_H
#define SURFACEPOINTSDATA_H

#include <vector>

#include <QApplication>
#include <QObject>
#include <QString>
#include <QUdpSocket>
#include <QMutex>

//======================================================================
// SurfacePointsData class handles communication with the an exernal
// source sending target poses via a QUdpSocket as surface location and
// normal data. Functionality is to continually receive data and store
// the latest requested position.
//======================================================================
class SurfacePointsData : public QObject
{
    Q_OBJECT

public:
    explicit SurfacePointsData(QObject *parent = nullptr);
    ~SurfacePointsData();

    bool initialise(const QHostAddress& surfPointsIpAddress, unsigned short receivePort);

    // Access functions
    bool getPoseData(std::vector<double>& poseData) const;

    static const int _nDofs = 6;

public slots:
    // Thread functions
    void run();
    void stop();

signals:
    void finished();

    void poseRequestReceived();

private:
    QUdpSocket* _socket;
    QHostAddress _surfPointsAddress;
    unsigned short _receivePort;

    QMutex mutable _mutex;

    bool _running;

    double _poseData[_nDofs];

private slots:
    void socket_dataReceived();
};

#endif // SURFACEPOINTSDATA_H
