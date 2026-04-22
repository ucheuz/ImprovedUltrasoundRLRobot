#include <iostream>
#include <utility>

#include "CalibratedSensor.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructors and destructor
//----------------------------------------------------------------------
CalibratedSensor::CalibratedSensor()
{
    _nComponents = 0;
    _nRawValues = 0;
    _rawValueListIdx = 0;

    _calibrationWeights.resize(0);
    _calibrationOffsets.resize(0);
}

CalibratedSensor::CalibratedSensor(const CalibratedSensor& calibratedSensor)
{
    _name = calibratedSensor._name;
    _nComponents = calibratedSensor._nComponents;
    _componentNames = calibratedSensor._componentNames;
    _nRawValues = calibratedSensor._nRawValues;
    _rawValueListIdx = calibratedSensor._rawValueListIdx;
    _rawValueScale = calibratedSensor._rawValueScale;
    _rawValueMin = calibratedSensor._rawValueMin;
    _rawValueMax = calibratedSensor._rawValueMax;
    _calibrationWeights = calibratedSensor._calibrationWeights;
    _calibrationOffsets = calibratedSensor._calibrationOffsets;
}

CalibratedSensor::~CalibratedSensor()
{
}

//----------------------------------------------------------------------
// Get / set the name of this sensor
//----------------------------------------------------------------------
void CalibratedSensor::setName(const QString& sensorName)
{
    _name.clear();
    _name.append(sensorName);
}

QString CalibratedSensor::getName() const
{
    return _name;
}

//----------------------------------------------------------------------
// Get / set the number of components
//----------------------------------------------------------------------
void CalibratedSensor::setNComponents(const int nComponents)
{
    if (nComponents < 0) {
        std::cerr << "Error in CalibratedSensor::setNComponents: number of components must be >= 0." << std::endl;
        return;
    }

    _nComponents = nComponents;
}

int CalibratedSensor::getNComponents() const
{
    return _nComponents;
}

//----------------------------------------------------------------------
// Get / set the name of the components
//----------------------------------------------------------------------
void CalibratedSensor::setComponentNames(const std::vector<QString>& componentNames)
{
    if (std::cmp_not_equal(componentNames.size(), _nComponents)) {
        std::cerr << "Error in CalibratedSensor::setComponentNames: wrong array length." << std::endl;
        return;
    }

    _componentNames = componentNames;
}

QString CalibratedSensor::getComponentName(int componentIdx) const
{
    try {
        return _componentNames.at(componentIdx);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in CalibratedSensor::getComponentName: index is out of range." << std::endl;
        return QString("??");
    }
}

//----------------------------------------------------------------------
// Get / set raw value size, starting index and scale
//----------------------------------------------------------------------
void CalibratedSensor::setRawValueParams(int nRawValues, int rawValuesListIdx, double rawValueScale, int rawValueMin, int rawValueMax)
{
    if (nRawValues < 0) {
        std::cerr << "Error in CalibratedSensor::setRawValueParams: number of raw values must be >= 0." << std::endl;
        return;
    }
    if (rawValuesListIdx < 0) {
        std::cerr << "Error in CalibratedSensor::setRawValueParams: raw values index must be >= 0." << std::endl;
        return;
    }

    _nRawValues = nRawValues;
    _rawValueListIdx = rawValuesListIdx;
    _rawValueScale = rawValueScale;
    _rawValueMin = rawValueMin;
    _rawValueMax = rawValueMax;
}

int CalibratedSensor::getNRawValues() const
{
    return _nRawValues;
}

//----------------------------------------------------------------------
// Get / set calibration parameters
//----------------------------------------------------------------------
void CalibratedSensor::setCalibrationWeights(const std::vector<double>& calibrationWeights)
{
    if (std::cmp_not_equal(calibrationWeights.size(), _nComponents * _nRawValues)) {
        std::cerr << "Error in CalibratedSensor::setCalibrationWeights: calibration length is inconsistent with number of raw values." << std::endl;
        return;
    }

    _calibrationWeights = calibrationWeights;
}

void CalibratedSensor::setCalibrationScaledValueOffsets(const std::vector<double>& scaledOffsets)
{
    if (std::cmp_not_equal(scaledOffsets.size(), _nRawValues)) {
        std::cerr << "Error in CalibratedSensor::setCalibrationScaledValueOffsets: offset length is inconsistent with number of raw values." << std::endl;
        return;
    }

    _calibrationOffsets = scaledOffsets;
}

//----------------------------------------------------------------------
// Extract the raw values relevant to this sensor from the full list
//----------------------------------------------------------------------
void CalibratedSensor::extractScaledValues(const std::vector<int>& allRawValues, std::vector<double>& sensorScaledValues) const
{
    sensorScaledValues.resize(_nRawValues);

    for (int v = 0; v < _nRawValues; v++) {
        if (allRawValues[v + _rawValueListIdx] <= _rawValueMin || allRawValues[v + _rawValueListIdx] >= _rawValueMax)
            sensorScaledValues[v] = 2.0 * _rawValueScale * _rawValueMax;  // Out of range
        else
            sensorScaledValues[v] = _rawValueScale * allRawValues[v + _rawValueListIdx];
    }
}

//----------------------------------------------------------------------
// Convert raw values to one component
//----------------------------------------------------------------------
double CalibratedSensor::applyCalibration(int componentIdx, const std::vector<double>& scaledValues, bool& ok) const
{
    if (componentIdx < 0 || componentIdx >= _nComponents) {
        std::cerr << "Error in CalibratedSensor::applyCalibration: index is out of range." << std::endl;
        ok = false;
        return 0.0;
    }

    ok = true;
    double output = 0.0;
    for (int v = 0; v < _nRawValues; v++) {
        if (_calibrationWeights[v + componentIdx * _nRawValues] == 0.0)
            continue;
        if (scaledValues[v] > _rawValueScale * _rawValueMax) {  // Out of range
            ok = false;
            return 0.0;
        }
        else {
            //if (_calibrationWeights[v + componentIdx * _nRawValues] != 0.0)
            output += _calibrationWeights[v + componentIdx * _nRawValues] * (scaledValues[v] - _calibrationOffsets[v]) / _rawValueScale;
        }
    }

    return output;
}
