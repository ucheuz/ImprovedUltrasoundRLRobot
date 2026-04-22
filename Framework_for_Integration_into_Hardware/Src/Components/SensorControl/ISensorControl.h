#ifndef ISENSORCONTROL_H
#define ISENSORCONTROL_H

#include <vector>

#include "CalibratedSensor.h"

#include <QObject>
#include <QtPlugin>
#include <QString>

//======================================================================
// ISensorControl is a base class for various types of sensor control.
// It provides functions for directly accessing sensor readings and also
// the force and distance values derived from these.

// Currently available sensor controllers are:
//   SensorControl_viaArduino

// *** Proposed sensor controllers:
//   SensorControl_simulated
//======================================================================
class ISensorControl : public QObject
{
    Q_OBJECT

public:
    ISensorControl();
    virtual ~ISensorControl();

    enum ConnectionStatus_t {INVALID_SETUP=0, DISCONNECTED, CONNECT_FAILED, CONNECTED};

    virtual void connectSensors() = 0;
    virtual bool isConnected() const = 0;

    int getNForceLocations() const;
    QString getForceLocationName(int locationIdx) const;
    int getNForceRawValues(int locationIdx) const;
    int getNForceComponents(int locationIdx) const;
    QString getForceComponentName(int locationIdx, int componentIdx) const;

    int getNDistanceLocations() const;
    QString getDistanceLocationName(int locationIdx) const;
    int getNDistanceRawValues(int locationIdx) const;
    int getNDistanceComponents(int locationIdx) const;
    QString getDistanceComponentName(int locationIdx, int componentIdx) const;

    void zeroForces();

    void getCurrentForces(int locationIdx, std::vector<double>& forces, std::vector<double>& voltages, bool& isInRange) const;
    void getCurrentForces(int locationIdx, std::vector<double>& forces, bool& isInRange) const;
    double getCurrentForceComponent(int locationIdx, int componentIdx, bool& isInRange) const;
    void getCurrentDistances(int locationIdx, std::vector<double>& distances, std::vector<double>& distanceMeasures, bool& isInRange) const;
    void getCurrentDistances(int locationIdx, std::vector<double>& distances, bool& isInRange) const;
    double getCurrentDistanceComponent(int locationIdx, int componentIdx, bool& isInRange) const;

signals:
    void connectionChanged();  // *** Should these be in the ISensorControl class?
    void connectionStatus(QString status);

protected:
    void initialise(const QString& robotId);

    QString _arduinoId;

    int _nTotalRawValues;

    std::vector<CalibratedSensor> _forceLocations;
    std::vector<CalibratedSensor> _distanceLocations;

    ConnectionStatus_t _status;

private:
    virtual void getSensorValues(std::vector<int>& rawValues) const = 0;
};

Q_DECLARE_INTERFACE(ISensorControl, "Interface.ISensorControl")

#endif // ISENSORCONTROL_H
