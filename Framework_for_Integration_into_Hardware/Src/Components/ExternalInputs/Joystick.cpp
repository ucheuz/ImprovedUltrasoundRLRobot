#include <iostream>
#include <sstream>
#include <cmath>

#include "Joystick.h"

#include <QMutexLocker>
#include <QThread>
#include <QNetworkDatagram>
#include <QDebug>

#define EPS_D std::numeric_limits<double>::epsilon()

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
Joystick::Joystick(QObject *parent) : QObject(parent)
{
    _socket = nullptr;

    _joystickAddress = QHostAddress(QHostAddress::Null);
    _receivePort = 0;

    for (int i = 0; i < _nDofs; i++)
        _poseData[i] = 0.0;
}

Joystick::~Joystick()
{
    if (_socket != nullptr) {
        disconnect(_socket, nullptr, this, nullptr);
        delete _socket;
    }
}

//----------------------------------------------------------------------
// Connect to UDP socket
//----------------------------------------------------------------------
bool Joystick::initialise(const QHostAddress& joystickIpAddress, unsigned short receivePort)
{
    if (_socket != nullptr) {
        disconnect(_socket, nullptr, this, nullptr);
        delete _socket;
    }

    _socket = new QUdpSocket(this);
    bool success = _socket->bind(receivePort);
    _receivePort = receivePort;

    _joystickAddress = joystickIpAddress;

    if (!success) {
        delete _socket;
        _socket = nullptr;
    }
    else
        connect(_socket, &QUdpSocket::readyRead, this, &Joystick::socket_dataReceived);

    return success;
}

//----------------------------------------------------------------------
// Access functions
// Return value indicates whether pose is valid
//----------------------------------------------------------------------
bool Joystick::getPoseData(std::vector<double>& poseData) const
{
    poseData.resize(_nDofs, 0.0);

    QMutexLocker locker(&_mutex);
    for (int i = 0; i < _nDofs; i++)
        poseData[i] = _poseData[i];

    // Invalid pose if quaternion is zero
    return !(std::fabs(_poseData[3]) < EPS_D && std::fabs(_poseData[4]) < EPS_D &&
             std::fabs(_poseData[5]) < EPS_D && std::fabs(_poseData[6]) < EPS_D);

}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for receiving joystick data
//----------------------------------------------------------------------
void Joystick::run()
{
    _running = true;

    while (_running) {
        qApp->processEvents();
        //QThread::msleep(200);
    }

    emit finished();
}

//----------------------------------------------------------------------
// Thread process stop function
//----------------------------------------------------------------------
void Joystick::stop()
{
    _running = false;
}

//----------------------------------------------------------------------
// Send force vector to joystick via UDP
//----------------------------------------------------------------------
void Joystick::updateForce(const std::vector<double>& force)
{
    if (force.size() == 0)
        return;

    if (_socket != nullptr) {
        std::ostringstream message;
        for (int i = 0; std::cmp_less(i, force.size() - 1); i++)
            message << force[i] << ",";
        message << force.back() << "\n";
        QByteArray baMessage(message.str().c_str());

        _socket->writeDatagram(baMessage, _joystickAddress, 15000);
    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// UDP socket receive slot.
//----------------------------------------------------------------------
void Joystick::socket_dataReceived()
{
    while (_socket->hasPendingDatagrams()) {

        // Get the data
        QNetworkDatagram datagram = _socket->receiveDatagram(_socket->pendingDatagramSize());
        //std::cerr << datagram.senderAddress().toString().toStdString() << " " << datagram.senderPort() << std::endl;
        if (!datagram.senderAddress().isEqual(_joystickAddress))
            continue;

        QByteArray data = datagram.data();
        QString dataStr(data);
        QList<QString> words = dataStr.split(",");

        if (words.length() < _nDofs)
            continue;

        QMutexLocker locker(&_mutex);

        bool ok;
        double dTmp;
        for (int i = 0; i < _nDofs; i++) {
            dTmp = words[i].toDouble(&ok);
            if (ok)
                _poseData[i] = dTmp;
        }
    }
}
