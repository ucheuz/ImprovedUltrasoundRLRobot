#include "ControlPanel.h"

#include <iostream>
#include <sstream>

#include <QMutexLocker>
#include <QThread>
#include <QDebug>

#define EPS_D std::numeric_limits<double>::epsilon()

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
ControlPanel::ControlPanel()
{
    //_arduinoId = "553303435343512130F1";
    //_arduinoId = "cu.wchusbserial1430";
    //_arduinoId = "cu.usbserial-1420";
    _arduinoId = "cu.usbserial-1410";

    _running = false;

    _prevProbeButtons[0] = false;
    _prevProbeButtons[1] = false;

    _delimiters.setPattern("[<,;>]");  // Set QRegularExpression once here so it doesn't have to be recreated every time new data is received.

    _serialPort.setParent(this);
    connect(&_serialPort, &QSerialPort::readyRead, this, &ControlPanel::serialPort_dataReceived);
}

ControlPanel::~ControlPanel()
{
    if (_serialPort.isOpen())
        _serialPort.close();


}

//----------------------------------------------------------------------
// Locate arduino by ID and connect to port
//----------------------------------------------------------------------
bool ControlPanel::initialise()
{
    if (_serialPort.isOpen())
        _serialPort.close();

    bool portFound = false;
    QSerialPortInfo portInfo;

    QList<QSerialPortInfo> portList = QSerialPortInfo::availablePorts();
    for (int i = 0; i < portList.length(); i++) {
        std::cerr << portList[i].portName().toStdString() << " | " << portList[i].description().toStdString() << " | " << portList[i].manufacturer().toStdString() << " | " << portList[i].serialNumber().toStdString() << std::endl;
        //if (QString::compare(portList[i].serialNumber(), _arduinoId) == 0) {
        if (QString::compare(portList[i].portName(), _arduinoId) == 0) {  // Arduino used doesn't have a serial number
            portFound = true;
            portInfo = portList[i];
            break;
        }
    }

    if (portFound)
        connectToPort(portInfo);
    return _serialPort.isOpen();

}

//----------------------------------------------------------------------
// Access function
// Decodes the list of button states and returns a value indicating
// which button was pressed. The 'value1' and 'value2' variables contain
// further information, such as direction or intensity
//----------------------------------------------------------------------
ControlPanel::ButtonCode_t ControlPanel::getButtonCode(double& value1, double& value2)
{
    ButtonCode_t button = BUTTON_NONE;
    value1 = 0.0;
    value2 = 0.0;

    // Check probe selection buttons first
    // These are maintained buttons, so these have priority over other buttons
    if (_probeButtons[0] && !_prevProbeButtons[0]) {
        button = BUTTON_PROBE_RIGHT;
        value1 = 1.0;
    }
    else if (!_probeButtons[0] && _prevProbeButtons[0]) {
        button = BUTTON_PROBE_RIGHT;
        value1 = -1.0;
    }

    // Left probe selection button
    if (_probeButtons[1] && !_prevProbeButtons[1]) {
        button = BUTTON_PROBE_LEFT;
        value1 = 1.0;
    }
    else if (!_probeButtons[1] && _prevProbeButtons[1]) {
        button = BUTTON_PROBE_LEFT;
        value1 = -1.0;
    }

    // Track the previous probe button states
    for (int i = 0; i < _nProbeButtons; i++)
        _prevProbeButtons[i] = _probeButtons[i];

    if (button == BUTTON_PROBE_RIGHT || button == BUTTON_PROBE_LEFT)
        return button;

    // All basic momentary buttons
    if (_buttons[0] || _buttons[1]) {
        value1 = _buttons[0] ? 1.0 : -1.0;  // Direction indicated by sign of value
        return BUTTON_X;
    }
    else if (_buttons[2] || _buttons[3]) {
        value1 = _buttons[2] ? 1.0 : -1.0;
        return BUTTON_Y;
    }
    else if (_buttons[4] || _buttons[5]) {
        value1 = _buttons[4] ? 1.0 : -1.0;
        return BUTTON_Z;
    }
    else if (_buttons[6] || _buttons[7]) {
        value1 = _buttons[6] ? 1.0 : -1.0;
        return BUTTON_AX;
    }
    else if (_buttons[8])
        return BUTTON_STOP;

    // Joystick - will only affect the output if no other buttons are pressed
    // Allow a 15% deadzone in the centre
    if (std::fabs(_joystick[0]) > 0.15f || std::fabs(_joystick[1]) > 0.15f) {
        value1 = static_cast<double>(std::sqrt(_joystick[0] * _joystick[0] + _joystick[1] * _joystick[1]));  // Magnitude
        value2 = static_cast<double>(atan2(-_joystick[1], _joystick[0]));  // Direction in radians
        // _joystick[1] is minus above so that upward movements produce a positive angle
        return BUTTON_JOYSTICK;
    }

    return BUTTON_NONE;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for receiving ControlPanel data
//----------------------------------------------------------------------
void ControlPanel::run()
{
    _running = true;

    while (_running) {
        qApp->processEvents();
        //QThread::msleep(200);
    }

    if (_serialPort.isOpen())
        _serialPort.close();

    emit finished();
}

//----------------------------------------------------------------------
// Thread process stop function
//----------------------------------------------------------------------
void ControlPanel::stop()
{
    _running = false;
}

//----------------------------------------------------------------------
// Send force vector to control panel via Serial
// *** Only uses first force value so far
//----------------------------------------------------------------------
void ControlPanel::updateForces(const std::vector<double>& forces)
{
    if (forces.empty())
        return;

    int ledCode = 0;
    if (forces[0] <= 4)
        ledCode = 1;
    else if (forces[0] < 6)
        ledCode = 2;
    else
        ledCode = 3;

    if (_serialPort.isOpen()) {
        std::ostringstream message;
        message << ledCode << "\n";
        _serialPort.write(message.str().c_str());
    }
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Port connection
//----------------------------------------------------------------------
void ControlPanel::connectToPort(QSerialPortInfo portInfo)
{
    _serialPort.setPortName(portInfo.portName());
    std::cerr << "Connecting to port " << _serialPort.portName().toStdString() << std::endl;

    if (!_serialPort.open(QIODevice::ReadWrite)) {
        std::cerr << "Failed to open port " << _serialPort.portName().toStdString() << ". " << _serialPort.errorString().toStdString() << std::endl;
        return;
    }

    if (_serialPort.isOpen()) {
        _serialPort.setBaudRate(QSerialPort::Baud115200);
        _serialPort.setParity(QSerialPort::NoParity);
        _serialPort.setDataBits(QSerialPort::Data8);
        _serialPort.setStopBits(QSerialPort::OneStop);

    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Serial port receive slot.
// Expected format of data received is a text string listing the states
// of all buttons:
// <RProbe,LProbe,X1,X2,Y1,Y2,Z1,Z2,Ax1,Ax2,Stop,JoystickX,JoystickY>
// <     0,     1, 2, 3, 4, 5, 6, 7,  8,  9,  10,       11,       12>

// All are integers representing booleans (0/1), except for the
// joystick values, which are intergers representing a range (0-4095)
//----------------------------------------------------------------------
void ControlPanel::serialPort_dataReceived()
{
    while (_serialPort.canReadLine()) {

        QByteArray data = _serialPort.readLine();
        QString dataStr(data);
        if (dataStr.startsWith("<")) {
            QList<QString> words = dataStr.split(_delimiters);

            if (words.length() < _nProbeButtons + _nButtons + _nJoystick + 2)
                continue;

            //qDebug() << dataStr;

            QMutexLocker locker(&_mutex);

            bool ok;
            int iTmp;
            for (int i = 0; i < _nProbeButtons; i++) {
                iTmp = words[i+1].toInt(&ok);
                if (ok) {
                    _probeButtons[i] = iTmp == 0;  // 0 means button is down, 1 is up
                }
            }
            for (int i = 0; i < _nButtons; i++) {
                iTmp = words[i+1+_nProbeButtons].toInt(&ok);
                if (ok) {
                    _buttons[i] = iTmp != 0;
                }
            }
            for (int i =  0; i < _nJoystick; i++) {
                iTmp = words[i+1+_nProbeButtons+_nButtons].toInt(&ok);
                if (ok) {
                    // Map the range 0 - 4095 to -1 - +1
                    _joystick[i] = 2.0f * float(iTmp) / 4095.0f - 1.0f;
                }
            }
        }
    }
}
