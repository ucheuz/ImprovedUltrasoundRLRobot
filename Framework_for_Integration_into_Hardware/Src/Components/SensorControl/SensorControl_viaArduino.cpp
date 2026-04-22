#include <iostream>

#include "SensorControl_viaArduino.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SensorControl_viaArduino::SensorControl_viaArduino(const QString& robotId)
{
    _arduinoThread = nullptr;
    _arduinoSensors = nullptr;

    initialise(robotId);
}

SensorControl_viaArduino::~SensorControl_viaArduino()
{
    disconnectSensors();
}

//======================================================================
// Public functions inplemented from ISensorControl
//======================================================================
//----------------------------------------------------------------------
// Connect sensor Arduino
//----------------------------------------------------------------------
void SensorControl_viaArduino::connectSensors()
{
    if (_status == INVALID_SETUP)
        return;

    disconnectSensors();

    _arduinoThread = new QThread();
    _arduinoSensors = new ArduinoSensors(_arduinoId, _nTotalRawValues);

    bool success = _arduinoSensors->initialise();

    if (!success) {
        std::cerr << "Error in SensorControl_viaArduino::connectSensors: failed to connect to sensor Arduino (" << _arduinoId.toStdString() << ")." << std::endl;
        delete _arduinoSensors;
        delete _arduinoThread;
    }
    else {
        _arduinoSensors->moveToThread(_arduinoThread);

        // Thread signals
        connect(_arduinoThread, &QThread::started, _arduinoSensors, &ArduinoSensors::run);
        connect(this, &SensorControl_viaArduino::stopArduinoProcess, _arduinoSensors, &ArduinoSensors::stop);
        connect(_arduinoSensors, &ArduinoSensors::finished, _arduinoThread, &QThread::quit);
        connect(_arduinoSensors, &ArduinoSensors::finished, _arduinoSensors, &ArduinoSensors::deleteLater);
        connect(_arduinoThread, &QThread::finished, _arduinoThread, &QThread::deleteLater);

        _arduinoThread->start();
    }

    if (isConnected()) {
        _status = CONNECTED;
        emit connectionStatus("OK");
    }
    else {
        _status = CONNECT_FAILED;
        emit connectionStatus("Connect failed");
    }

    emit connectionChanged();
}

//----------------------------------------------------------------------
// Get Arduino connection status
//----------------------------------------------------------------------
bool SensorControl_viaArduino::isConnected() const
{
    return _arduinoSensors != nullptr;
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the Arduino and clear variables ready to reconnect
//----------------------------------------------------------------------
void SensorControl_viaArduino::disconnectSensors()
{
    // Signal Arduino to disconnect
    emit stopArduinoProcess();

    // Wait for Arduino to disconnect fully so it's safe to reconnect if needed
    while (_arduinoSensors != nullptr)
        qApp->processEvents();

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    emit connectionChanged();
}

//======================================================================
// Private functions implemented from ISensorControl
//======================================================================
//----------------------------------------------------------------------
// Get the current sensor readings from the Arduino thread class
//----------------------------------------------------------------------
void SensorControl_viaArduino::getSensorValues(std::vector<int>& rawValues) const
{
    _arduinoSensors->getSensorValues(rawValues);
}
