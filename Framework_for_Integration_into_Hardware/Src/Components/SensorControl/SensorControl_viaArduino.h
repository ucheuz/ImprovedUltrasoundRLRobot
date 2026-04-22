#ifndef SENSORCONTROL_VIAARDUINO_H
#define SENSORCONTROL_VIAARDUINO_H

#include <vector>

#include <QObject>
#include <QString>
#include <QThread>
#include <QPointer>

#include "ArduinoSensors.h"
#include "ISensorControl.h"

//======================================================================
// SensorControl_viaArduino class implements the functions of
// ISensorControl for the case of sensor readings received through an
// Arduino serial connection.
//======================================================================
class SensorControl_viaArduino : public ISensorControl
{
    Q_OBJECT
    Q_INTERFACES(ISensorControl)

public:
    SensorControl_viaArduino(const QString& robotId);
    ~SensorControl_viaArduino();

    // --- Functions implemented from ISensorControl ---
    void connectSensors();
    bool isConnected() const;

signals:
    void stopArduinoProcess();

private:
    QPointer<QThread> _arduinoThread;
    QPointer<ArduinoSensors> _arduinoSensors;

    void disconnectSensors();

    // --- Function implemented from ISensorControl ---
    void getSensorValues(std::vector<int>& rawValues) const;
};

#endif // SENSORCONTROL_VIAARDUINO_H
