#include <iostream>

#include "ISensorControl.h"

#include <QDomDocument>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
// Parameters are initialised in initialise function called from
// constructor in derived class
//----------------------------------------------------------------------
ISensorControl::ISensorControl()
{
}

ISensorControl::~ISensorControl()
{
}

//----------------------------------------------------------------------
// Get force sensor information
//----------------------------------------------------------------------
int ISensorControl::getNForceLocations() const
{
    return _forceLocations.size();
}

QString ISensorControl::getForceLocationName(int locationIdx) const
{
    try {
        return _forceLocations.at(locationIdx).getName();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getForceLocationName: index is out of range." << std::endl;
        return QString();
    }
}

int ISensorControl::getNForceRawValues(int locationIdx) const
{
    try {
        return _forceLocations.at(locationIdx).getNRawValues();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getNForceRawValues: index is out of range." << std::endl;
        return 0;
    }
}

int ISensorControl::getNForceComponents(int locationIdx) const
{
    try {
        return _forceLocations.at(locationIdx).getNComponents();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getNForceComponents: index is out of range." << std::endl;
        return 0;
    }
}

QString ISensorControl::getForceComponentName(int locationIdx, int componentIdx) const
{
    try {
        return _forceLocations.at(locationIdx).getComponentName(componentIdx);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getForceComponentName: index is out of range." << std::endl;
        return QString("??");
    }
}

//----------------------------------------------------------------------
// Get distance sensor information
//----------------------------------------------------------------------
int ISensorControl::getNDistanceLocations() const
{
    return _distanceLocations.size();
}

QString ISensorControl::getDistanceLocationName(int locationIdx) const
{
    try {
        return _distanceLocations.at(locationIdx).getName();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getDistanceLocationName: index is out of range." << std::endl;
        return QString();
    }
}

int ISensorControl::getNDistanceRawValues(int locationIdx) const
{
    try {
        return _distanceLocations.at(locationIdx).getNRawValues();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getNDistanceRawValues: index is out of range." << std::endl;
        return 0;
    }
}

int ISensorControl::getNDistanceComponents(int locationIdx) const
{
    try {
        return _distanceLocations.at(locationIdx).getNComponents();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getNDistanceComponents: index is out of range." << std::endl;
        return 0;
    }
}

QString ISensorControl::getDistanceComponentName(int locationIdx, int componentIdx) const
{
    try {
        return _distanceLocations.at(locationIdx).getComponentName(componentIdx);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getDistanceComponentName: index is out of range." << std::endl;
        return QString("??");
    }
}

//----------------------------------------------------------------------
// Set offsets to current values, for force sensors only
//----------------------------------------------------------------------
void ISensorControl::zeroForces()
{
    std::vector<int> allRawValues;
    std::vector<double> voltages;

    if (isConnected()) {
        getSensorValues(allRawValues);
        for (CalibratedSensor& cs : _forceLocations) {
            cs.extractScaledValues(allRawValues, voltages);
            cs.setCalibrationScaledValueOffsets(voltages);
        }
    }
}

//----------------------------------------------------------------------
// Access force sensor values
//----------------------------------------------------------------------
void ISensorControl::getCurrentForces(int locationIdx, std::vector<double>& forces, std::vector<double>& voltages, bool& isInRange) const
{
    isInRange = false;

    try {
        forces.resize(_forceLocations.at(locationIdx).getNComponents(), 0.0);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getCurrentForces: sensor index is out of range." << std::endl;
        return;
    }

    if (isConnected()) {
        std::vector<int> allRawValues;
        getSensorValues(allRawValues);
        _forceLocations[locationIdx].extractScaledValues(allRawValues, voltages);
        isInRange = true;
        for (int f = 0; f < _forceLocations[locationIdx].getNComponents(); f++) {
            bool inRange;
            forces[f] = _forceLocations[locationIdx].applyCalibration(f, voltages, inRange);
            isInRange &= inRange;
        }
    }
}

void ISensorControl::getCurrentForces(int locationIdx, std::vector<double>& forces, bool& isInRange) const
{
    std::vector<double> voltages;
    getCurrentForces(locationIdx, forces, voltages, isInRange);
}

double ISensorControl::getCurrentForceComponent(int locationIdx, int componentIdx, bool& isInRange) const
{
    isInRange = false;

    if (componentIdx < 0 || componentIdx >= _forceLocations[locationIdx].getNComponents()) {  // *** Check that locationIdx is in range
        std::cerr << "Error in ISensorControl::getCurrentForceComponent: index is out of range." << std::endl;
        return 0.0;
    }

    double force = 0.0;
    if (isConnected()) {
        std::vector<int> allRawValues;
        std::vector<double> voltages;
        getSensorValues(allRawValues);

        try {
            _forceLocations.at(locationIdx).extractScaledValues(allRawValues, voltages);
        }
        catch (const std::out_of_range& e) {
            std::cerr << "Error in ISensorControl::getCurrentForceComponent: sensor index is out of range." << std::endl;
            return 0.0;
        }

        force = _forceLocations[locationIdx].applyCalibration(componentIdx, voltages, isInRange);
    }

    return force;
}

//----------------------------------------------------------------------
// Access distance sensor values
//----------------------------------------------------------------------
void ISensorControl::getCurrentDistances(int locationIdx, std::vector<double>& distances, std::vector<double>& distanceMeasures, bool& isInRange) const
{
    isInRange = false;

    try {
        distances.resize(_distanceLocations.at(locationIdx).getNComponents(), 0.0);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in ISensorControl::getCurrentDistances: sensor index is out of range." << std::endl;
        return;
    }

    if (isConnected()) {
        std::vector<int> allRawValues;
        getSensorValues(allRawValues);
        _distanceLocations[locationIdx].extractScaledValues(allRawValues, distanceMeasures);
        isInRange = true;
        for (int d = 0; d < _distanceLocations[locationIdx].getNComponents(); d++) {
            bool inRange;
            distances[d] = _distanceLocations[locationIdx].applyCalibration(d, distanceMeasures, inRange);
            isInRange &= inRange;
        }
    }
}

void ISensorControl::getCurrentDistances(int locationIdx, std::vector<double>& distances, bool& isInRange) const
{
    std::vector<double> distanceMeasures;
    getCurrentDistances(locationIdx, distances, distanceMeasures, isInRange);
}

double ISensorControl::getCurrentDistanceComponent(int locationIdx, int componentIdx, bool& isInRange) const
{
    isInRange = false;

    if (componentIdx >= _distanceLocations[locationIdx].getNComponents()) {
        std::cerr << "Error in ISensorControl::getCurrentDistanceComponent: index is out of range." << std::endl;
        return 0.0;
    }

    double distance = 0.0;
    if (isConnected()) {
        std::vector<int> allRawValues;
        std::vector<double> distanceMeasures;
        getSensorValues(allRawValues);

        try {
            _distanceLocations.at(locationIdx).extractScaledValues(allRawValues, distanceMeasures);
        }
        catch (const std::out_of_range& e) {
            std::cerr << "Error in ISensorControl::getCurrentDistanceComponent: sensor index is out of range." << std::endl;
            return 0.0;
        }

        distance = _distanceLocations[locationIdx].applyCalibration(componentIdx, distanceMeasures, isInRange);
    }

    return distance;
}

//======================================================================
// Protected functions
//======================================================================
//----------------------------------------------------------------------
// Initialise variables from xml file for the specified robot
//----------------------------------------------------------------------
void ISensorControl::initialise(const QString& robotId)
{
    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl, sensorsEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", robotId);
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Sensors", sensorsEl);

    xmlParseSuccess &= XmlUtils::readValFromElement(sensorsEl, "sensorArduinoId", _arduinoId);
    xmlParseSuccess &= XmlUtils::readValFromElement(sensorsEl, "nTotalRawValues", _nTotalRawValues);

    // Read information for individual sensor locations (force vectors or distance measurements) from file
    QDomNodeList sensorLocationList = sensorsEl.elementsByTagName("SensorLocation");
    for (int s = 0; s < sensorLocationList.length(); s++) {

        QString sensorType = sensorLocationList.item(s).attributes().namedItem("type").nodeValue();

        CalibratedSensor sensor;
        sensor.setName(sensorLocationList.item(s).attributes().namedItem("location").nodeValue());

        std::vector<QString> componentNames;
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "componentNames", componentNames);
        sensor.setNComponents(componentNames.size());
        sensor.setComponentNames(componentNames);

        int nRawValues, rawValuesListIdx, rawValueMin, rawValueMax;
        double rawValueScale;
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "nRawValues", nRawValues);
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "rawValueListIdx", rawValuesListIdx);
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "rawValueScale", rawValueScale);
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "rawValueMin", rawValueMin);
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "rawValueMax", rawValueMax);
        sensor.setRawValueParams(nRawValues, rawValuesListIdx, rawValueScale, rawValueMin, rawValueMax);

        std::vector<double> sensorWeights;
        for (QString& componentName : componentNames)
            xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "componentWeights", sensorWeights, "dir", componentName);
        sensor.setCalibrationWeights(sensorWeights);

        std::vector<double> sensorOffsets;
        xmlParseSuccess &= XmlUtils::readValFromElement(sensorLocationList.item(s).toElement(), "offsets", sensorOffsets);
        sensor.setCalibrationScaledValueOffsets(sensorOffsets);

        if (sensorType.compare("force") == 0)
            _forceLocations.push_back(sensor);
        else if (sensorType.compare("distance") == 0)
            _distanceLocations.push_back(sensor);
        else {
            _status = INVALID_SETUP;
            std::cerr << "Error in SensorControl_viaArduino: unknown sensor type." << std::endl;
            return;
        }

        if (!xmlParseSuccess) {
            _status = INVALID_SETUP;
            std::cerr << "Error in SensorControl_viaArduino: failed to read valid sensor specification from xml file." << std::endl;
            return;
        }
    }

    _status = DISCONNECTED;
}
