#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <vector>
#include <QObject>
#include <QString>
#include <QtSerialPort/QtSerialPort>
#include <QMutex>

//======================================================================
// ControlPanel class handles communication with the external control
// panel via a QSerialPort. Functionality is to continually receive data
// and store the latest button states on the control panel. It also
// sends back force data to be displayed on the panel.
//======================================================================
class ControlPanel : public QObject
{
    Q_OBJECT

public:
    ControlPanel();
    ~ControlPanel();

    static const int _nDofs = 6;

    enum ButtonCode_t {BUTTON_PROBE_RIGHT=0, BUTTON_PROBE_LEFT, BUTTON_X, BUTTON_Y, BUTTON_Z, BUTTON_AX, BUTTON_STOP, BUTTON_JOYSTICK, BUTTON_NONE};

    bool initialise();

    // Access function
    ButtonCode_t getButtonCode(double& value1, double& value2);

public slots:
    // Thread functions
    void run();
    void stop();

    void updateForces(const std::vector<double>& forces);

signals:
    void finished();
    void cancelSequence();

private:
    QSerialPort _serialPort;
    QString _arduinoId;

    QMutex mutable _mutex;

    bool _running;

    static const int _nProbeButtons = 2;
    static const int _nButtons = 9;
    static const int _nJoystick = 2;
    bool _prevProbeButtons[_nProbeButtons];
    bool _probeButtons[_nProbeButtons];
    bool _buttons[_nButtons];
    float _joystick[_nJoystick];

    QRegularExpression _delimiters;

    void connectToPort(QSerialPortInfo portInfo);

private slots:
    void serialPort_dataReceived();
};


#endif // CONTROLPANEL_H

