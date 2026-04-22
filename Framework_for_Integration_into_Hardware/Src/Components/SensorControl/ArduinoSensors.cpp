#include <iostream>

#include "ArduinoSensors.h"

#include <QMutexLocker>
#include <QThread>

#include <QDebug>

#ifdef WRITE_TO_FILE
#include <QDateTime>
#endif  // WRITE_TO_FILE

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
ArduinoSensors::ArduinoSensors(QString arduinoId, int nSensorValues)
{
    _arduinoId = arduinoId;
    _running = false;
    _nSensorValues = nSensorValues;

    _sensorValues.resize(_nSensorValues, 0);

    _delimiters.setPattern("[<,;>]");  // Set QRegularExpression once here so it doesn't have to be recreated every time new data is received.

    _serialPort.setParent(this);
    connect(&_serialPort, &QSerialPort::readyRead, this, &ArduinoSensors::serialPort_dataReceived);
}

ArduinoSensors::~ArduinoSensors()
{
    if (_serialPort.isOpen())
        _serialPort.close();

#ifdef WRITE_TO_FILE
    if (_outFile.isOpen())
        _outFile.close();
#endif  // WRITE_TO_FILE
}

//----------------------------------------------------------------------
// Locate arduino by ID and connect to port
//----------------------------------------------------------------------
bool ArduinoSensors::initialise()
{
    if (_serialPort.isOpen())
        _serialPort.close();
#ifdef WRITE_TO_FILE
    if (_outFile.isOpen())
        _outFile.close();
#endif  // WRITE_TO_FILE

    bool portFound = false;
    QSerialPortInfo portInfo;

    QList<QSerialPortInfo> portList = QSerialPortInfo::availablePorts();
    for (int i = 0; i < portList.length(); i++) {
        //std::cerr << portList[i].description().toStdString() << " | " << portList[i].manufacturer().toStdString() << " | " << portList[i].serialNumber().toStdString() << std::endl;
        if (QString::compare(portList[i].serialNumber(), _arduinoId) == 0) {
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
// Access functions
//----------------------------------------------------------------------
void ArduinoSensors::getSensorValues(std::vector<int>& sensorValues) const
{
    sensorValues.resize(_nSensorValues);

    QMutexLocker locker(&_mutex);
    for (int i = 0; i < _nSensorValues; i++)
        sensorValues[i] = _sensorValues[i];
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Thread process function for receiving sensor data
//----------------------------------------------------------------------
void ArduinoSensors::run()
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
void ArduinoSensors::stop()
{
    _running = false;
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Port connection
//----------------------------------------------------------------------
void ArduinoSensors::connectToPort(QSerialPortInfo portInfo)
{
    _serialPort.setPortName(portInfo.portName());
    std::cerr << "Connecting to port " << _serialPort.portName().toStdString() << std::endl;

    if (!_serialPort.open(QIODevice::ReadWrite)) {
        std::cerr << "Failed to open port " << _serialPort.portName().toStdString() << ". " << _serialPort.errorString().toStdString() << std::endl;
        return;
    }

    if (_serialPort.isOpen()) {
        _serialPort.setBaudRate(QSerialPort::Baud115200);
        //_serialPort.setBaudRate(250000);
        _serialPort.setParity(QSerialPort::NoParity);
        _serialPort.setDataBits(QSerialPort::Data8);
        _serialPort.setStopBits(QSerialPort::OneStop);
#ifdef WRITE_TO_FILE
        QString outFilename("./Sensors.txt");
        _outFile.setFileName(outFilename);
        bool success = _outFile.open(QFile::WriteOnly | QFile::Text);
        if (!success) {
            std::cerr << "Error in ArduinoSensors::connectToPort: unable to open file " << outFilename.toStdString() << "." << std::endl;
            return;
        }
        _fout.setDevice(&_outFile);
#endif  // WRITE_TO_FILE
    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Serial port receive slot.
//----------------------------------------------------------------------
void ArduinoSensors::serialPort_dataReceived()
{
    while (_serialPort.canReadLine()) {

        QByteArray data = _serialPort.readLine();
        QString dataStr(data);
        if (dataStr.startsWith("<")) {
            QList<QString> words = dataStr.split(_delimiters);

#ifdef WRITE_TO_FILE
            int64_t time = QDateTime::currentMSecsSinceEpoch();
            _fout << time << ": " << dataStr << endl;
#endif  // WRITE_TO_FILE

            if (words.length() < _nSensorValues)
                continue;

            //qDebug() << dataStr;

            QMutexLocker locker(&_mutex);

            bool ok;
            int iTmp;
            for (int i = 0; i < _nSensorValues; i++) {
                iTmp = words[i+1].toInt(&ok);
                if (ok)
                    _sensorValues[i] = iTmp;
            }
        }
    }
}
