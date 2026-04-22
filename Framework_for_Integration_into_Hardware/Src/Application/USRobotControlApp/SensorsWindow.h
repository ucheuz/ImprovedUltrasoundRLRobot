#ifndef SENSORSWINDOW_H
#define SENSORSWINDOW_H

#include <vector>

#include <QDialog>
#include <QTimer>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>

#include "SensorControl/SensorControl_viaArduino.h"

namespace Ui {
class SensorsWindow;
}

class SensorsWindow : public QDialog
{
    Q_OBJECT

public:
    explicit SensorsWindow(QWidget *parent = 0, ISensorControl* sensorControl = nullptr);
    ~SensorsWindow();

public slots:
    void updateConnectionStatus();

private:
    Ui::SensorsWindow *ui;

    QTimer* _timer;

    SensorControl_viaArduino* _pSensorControl;

    // Force sensor voltage list UI
    std::vector<QLabel*> _label_forceLocationName;
    std::vector<std::vector<QLabel*> > _label_forceVoltage;
    std::vector<std::vector<QProgressBar*> > _progressBar_forceVoltage;
    std::vector<std::vector<QLabel*> > _label_forceVoltageVal;

    // Force values list UI
    QPushButton* _pushButton_zeroOffset;
    std::vector<QLabel*> _label_forceLocationName2;
    std::vector<std::vector<QLabel*> > _label_forceComponentName;
    std::vector<std::vector<QLabel*> > _label_forceComponentVal;

    // Distance sensor list UI
    std::vector<QLabel*> _label_distanceLocationName;
    std::vector<std::vector<QLabel*> > _label_distanceMeasure;
    std::vector<std::vector<QProgressBar*> > _progressBar_distanceMeasure;
    std::vector<std::vector<QLabel*> > _label_distanceMeasureVal;
    std::vector<std::vector<QLabel*> > _label_distanceComponentName;
    std::vector<std::vector<QLabel*> > _label_distanceComponentVal;

    void addForceVoltageListUi();
    void addForceListUi();
    void addDistanceListUi();

private slots:
    void updateSensorStatus();

    // Manual GUI slots
    void pushButton_zeroOffset_clicked();
};

#endif // SENSORSWINDOW_H
