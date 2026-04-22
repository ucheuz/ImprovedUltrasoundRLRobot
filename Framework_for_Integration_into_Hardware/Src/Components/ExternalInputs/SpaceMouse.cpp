#include <iostream>
#include <sstream>
#include <cmath>

#include "SpaceMouse.h"

#include <QMutexLocker>
#include <QThread>

#define BUFSIZE 128

union u_i16Val {
    unsigned char uc[2];
    int16_t i16;
};
int16_t bytesToInt16(const unsigned char* bytes)
{
    u_i16Val u;
    u.uc[0] = bytes[0];
    u.uc[1] = bytes[1];
    return u.i16;
}

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SpaceMouse::SpaceMouse(QObject *parent) : QObject(parent)
{
    _device = nullptr;

    hid_init();

    for (int i = 0; i < _nDofs; i++)
        _poseData[i] = 0.0;
}

SpaceMouse::~SpaceMouse()
{
    // Close connection if open
    if (_device != nullptr)
        hid_close(_device);

    hid_exit();
}

//----------------------------------------------------------------------
// Connect to 3DConnexion SpaceMouse device
//----------------------------------------------------------------------
bool SpaceMouse::initialise()
{
    if (_device != nullptr) {
        hid_close(_device);
    }

    // Vendor ID for SpaceMouse Compact: 9583
    // Product ID for SpaceMouse Compact: 50741
    _device = hid_open(9583, 50741, nullptr);
    if (_device == nullptr) {
        std::cerr << "Unable to open SpaceMouse device" << std::endl;
        return false;
    }

    hid_set_nonblocking(_device, 1);

    return true;
}

//----------------------------------------------------------------------
// Access functions
// Return value indicates whether pose is valid
//----------------------------------------------------------------------
bool SpaceMouse::getPoseData(std::vector<double>& poseData) const
{
    poseData.resize(_nDofs, 0.0);

    QMutexLocker locker(&_mutex);
    for (unsigned int i = 0; i < _nDofs; i++)
        poseData[i] = _poseData[i];

    // Calibration to axis directions
    poseData[0] *= -1.0;
    poseData[2] *= -1.0;
    poseData[3] *= -1.0;
    poseData[5] *= -1.0;

    return true;  // Always valid
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for receiving SpaceMouse data
//----------------------------------------------------------------------
void SpaceMouse::run()
{
    _running = true;

    while (_running) {
        checkForNewData();
        qApp->processEvents();
        //QThread::msleep(200);
    }

    emit finished();
}

//----------------------------------------------------------------------
// Thread process stop function
//----------------------------------------------------------------------
void SpaceMouse::stop()
{
    _running = false;
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Read from space mouse
//----------------------------------------------------------------------
bool SpaceMouse::checkForNewData()
{
    if (_device == nullptr)
        return false;

    int res;
    unsigned char buf[BUFSIZE];
    memset(buf, 0, sizeof(buf));

    res = hid_read(_device, buf, BUFSIZE);

    if (res < 1)
        return false;

    else {
        uint8_t channel = buf[0];

        if (channel == 3 && res == 3) {
            // Button pressed

            uint8_t buttonId = buf[1];
            //uint8_t bit = buf[2];  // Doesn't seem to do anything
            if (buttonId == 1)
                emit button1Pressed();
            else if (buttonId == 2)
                emit button2Pressed();
            else
                return false;
        }

        else if (channel == 1 && res == 7) {
            // SpaceMouse position data

            int16_t x, y, z;
            x = bytesToInt16(buf+1);
            y = bytesToInt16(buf+3);
            z = bytesToInt16(buf+5);

            QMutexLocker locker(&_mutex);
            _poseData[0] = x;
            _poseData[1] = y;
            _poseData[2] = z;
        }

        else if (channel == 2 && res == 7) {
            // SpaceMouse orientation data

            int16_t pitch, roll, yaw;
            pitch = bytesToInt16(buf+1);
            roll = bytesToInt16(buf+3);
            yaw = bytesToInt16(buf+5);

            QMutexLocker locker(&_mutex);
            _poseData[3] = pitch;
            _poseData[4] = roll;
            _poseData[5] = yaw;
        }

        else
            return false;

        return true;
    }
}
