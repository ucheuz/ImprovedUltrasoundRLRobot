#ifndef ARDUINOSENSORS_H
#define ARDUINOSENSORS_H

#include <vector>

#include <QObject>
#include <QString>
#include <QtSerialPort/QtSerialPort>
#include <QMutex>

//#define WRITE_TO_FILE
#ifdef WRITE_TO_FILE
#include <QFile>
#include <QTextStream>
#endif  // WRITE_TO_FILE

//======================================================================
// ArduinoSensors class handles communication with the sensor Arduino
// via a QSerialPort. Its only function is to run in a separate thread
// and continually receive and store voltage values for force and
// proximity data.
//======================================================================
class ArduinoSensors : public QObject
{
    Q_OBJECT

public:
    ArduinoSensors(QString arduinoId, int nSensorValues);
    ~ArduinoSensors();

    bool initialise();

    // Access functions
    void getSensorValues(std::vector<int>& sensorValues) const;

public slots:
    // Thread functions
    void run();
    void stop();

signals:
    void finished();

private:
    QSerialPort _serialPort;
    QString _arduinoId;

    QMutex mutable _mutex;

    bool _running;

    int _nSensorValues;
    std::vector<int> _sensorValues;

    QRegularExpression _delimiters;

#ifdef WRITE_TO_FILE
    QFile _outFile;
    QTextStream _fout;
#endif  // WRITE_TO_FILE

    void connectToPort(QSerialPortInfo portInfo);

private slots:
    void serialPort_dataReceived();
};

#endif // ARDUINOSENSORS_H
