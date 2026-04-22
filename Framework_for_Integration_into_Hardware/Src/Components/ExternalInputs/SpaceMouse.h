#ifndef SPACEMOUSE_H
#define SPACEMOUSE_H

#include <vector>

#include <QApplication>
#include <QObject>
#include <QMutex>

#include <hidapi/hidapi.h>

//======================================================================
// SpaceMouse class handles communication with the 3DConnexion
// SpaceMouse six-axis joystick. It directly connects using the hidapi
// library. Functionality is to continually receive data and store the
// latest six-axis values, as well as monitoring joystick button
// presses.
//======================================================================
class SpaceMouse : public QObject
{
    Q_OBJECT

public:
    explicit SpaceMouse(QObject *parent = nullptr);
    ~SpaceMouse();

    bool initialise();

    // Access functions
    bool getPoseData(std::vector<double>& poseData) const;

    static const int _nDofs = 6;

public slots:
    // Thread functions
    void run();
    void stop();

signals:
    void finished();
    void button1Pressed();
    void button2Pressed();

private:
    hid_device* _device;

    QMutex mutable _mutex;

    bool _running;

    double _poseData[_nDofs];

    bool checkForNewData();
};

#endif // SPACEMOUSE_H
