#include <iostream>
#include <sstream>
#include <cmath>

#include "SurfacePointsData.h"

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
SurfacePointsData::SurfacePointsData(QObject *parent) : QObject(parent)
{
    _socket = nullptr;

    _surfPointsAddress = QHostAddress(QHostAddress::Null);
    _receivePort = 0;

    for (int i = 0; i < _nDofs; i++)
        _poseData[i] = 0.0;
}

SurfacePointsData::~SurfacePointsData()
{
    if (_socket != nullptr) {
        disconnect(_socket, nullptr, this, nullptr);
        delete _socket;
    }
}

//----------------------------------------------------------------------
// Connect to UDP socket
//----------------------------------------------------------------------
bool SurfacePointsData::initialise(const QHostAddress& surfPointsIpAddress, unsigned short receivePort)
{
    if (_socket != nullptr) {
        disconnect(_socket, nullptr, this, nullptr);
        delete _socket;
    }

    _socket = new QUdpSocket(this);
    bool success = _socket->bind(receivePort);
    _receivePort = receivePort;

    _surfPointsAddress = surfPointsIpAddress;
    std::cerr << _surfPointsAddress.toString().toStdString() << " " << _receivePort << std::endl;

    if (!success) {
        delete _socket;
        _socket = nullptr;
    }
    else
        connect(_socket, &QUdpSocket::readyRead, this, &SurfacePointsData::socket_dataReceived);

    return success;
}

//----------------------------------------------------------------------
// Access functions
// Return value indicates whether pose is valid
//----------------------------------------------------------------------
bool SurfacePointsData::getPoseData(std::vector<double>& poseData) const
{
    poseData.resize(_nDofs, 0.0);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nDofs; i++)
        poseData[i] = _poseData[i];

    // Invalid pose if normal is zero
    return !(std::fabs(_poseData[3]) < EPS_D && std::fabs(_poseData[4]) < EPS_D &&
             std::fabs(_poseData[5]) < EPS_D );
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for receiving joystick data
//----------------------------------------------------------------------
void SurfacePointsData::run()
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
void SurfacePointsData::stop()
{
    _running = false;
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// UDP socket receive slot.
//----------------------------------------------------------------------
void SurfacePointsData::socket_dataReceived()
{
    while (_socket->hasPendingDatagrams()) {

        // Get the data
        QNetworkDatagram datagram = _socket->receiveDatagram(_socket->pendingDatagramSize());
        std::cerr << datagram.senderAddress().toString().toStdString() << " " << datagram.senderPort() << std::endl;
        if (!datagram.senderAddress().isEqual(_surfPointsAddress))
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

    emit poseRequestReceived();
}
