#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <vector>

#include <QApplication>
#include <QObject>
#include <QString>
#include <QUdpSocket>
#include <QMutex>

//======================================================================
// Joystick class handles communication with the Phantom Omni joystick
// via a QUdpSocket. Functionality is to continually receive data and
// store the latest requested position. Also to send back force data
// when requested.
//======================================================================
class Joystick : public QObject
{
    Q_OBJECT

public:
    explicit Joystick(QObject *parent = nullptr);
    ~Joystick();

    bool initialise(const QHostAddress& joystickIpAddress, unsigned short receivePort);

    // Access functions
    bool getPoseData(std::vector<double>& poseData) const;

    static const int _nDofs = 7;

public slots:
    // Thread functions
    void run();
    void stop();

    void updateForce(const std::vector<double>& force);

signals:
    void finished();

private:
    QUdpSocket* _socket;
    QHostAddress _joystickAddress;
    unsigned short _receivePort;

    QMutex mutable _mutex;

    bool _running;

    double _poseData[_nDofs];

private slots:
    void socket_dataReceived();
};

#endif // JOYSTICK_H
