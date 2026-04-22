#ifndef CALIBRATEDSENSOR_H
#define CALIBRATEDSENSOR_H

#include <vector>

#include <QObject>
#include <QString>
#include <QStringList>

//======================================================================
// CalibratedSensor class handles conversion between raw sensor values
// received from the Arduino, scaled sensor values (e.g. converted from
// ADC value to voltage) and calibrated output values (e.g. force in N).
// One instance represents a sensor location, e.g. 1 force vector
// measured at a point, derived from a number of individual voltage
// measurements.
//======================================================================
class CalibratedSensor : public QObject
{
    Q_OBJECT

public:
    CalibratedSensor();
    CalibratedSensor(const CalibratedSensor& calibratedSensor);
    ~CalibratedSensor();

    // Get / set functions
    void setName(const QString& sensorName);
    QString getName() const;

    void setNComponents(const int nComponents);
    int getNComponents() const;

    void setComponentNames(const std::vector<QString>& componentNames);
    QString getComponentName(int componentIdx) const;

    void setRawValueParams(int nRawValues, int rawValuesListIdx, double rawValueScale, int rawValueMin, int rawValueMax);
    int getNRawValues() const;

    void setCalibrationWeights(const std::vector<double>& calibrationWeights);
    void setCalibrationScaledValueOffsets(const std::vector<double>& scaledOffsets);

    void extractScaledValues(const std::vector<int>& allRawValues, std::vector<double>& sensorScaledValues) const;

    // Get a calibrated component value
    double applyCalibration(int componentIdx, const std::vector<double>& scaledValues, bool& ok) const;

private:
    QString _name;

    int _nComponents;
    std::vector<QString> _componentNames;

    int _nRawValues;
    int _rawValueListIdx;

    double _rawValueScale;
    int _rawValueMin;
    int _rawValueMax;

    std::vector<double> _calibrationWeights;  // Row-major calibration matrix from voltages to force vector components
    std::vector<double> _calibrationOffsets;
};

#endif // CALIBRATEDSENSOR_H
