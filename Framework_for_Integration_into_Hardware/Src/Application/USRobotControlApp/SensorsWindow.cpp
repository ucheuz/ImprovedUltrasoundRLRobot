#include <iostream>

#include "SensorsWindow.h"
#include "ui_SensorsWindow.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SensorsWindow::SensorsWindow(QWidget *parent, ISensorControl* sensorControl) :
    QDialog(parent),
    ui(new Ui::SensorsWindow)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    this->setSizeGripEnabled(true);

    if (sensorControl == nullptr) {
        _timer = nullptr;
        _pSensorControl = nullptr;
        return;
    }

    _pSensorControl = dynamic_cast<SensorControl_viaArduino*>(sensorControl);

    addForceVoltageListUi();  // Add variable UI, dependent on number of force sensors and voltages
    addForceListUi();  // Add variable UI, dependent on number of force sensor locations and components
    addDistanceListUi();  // Add variable UI, dependent on number of distance sensor locations

    connect(_pSensorControl, &SensorControl_viaArduino::connectionChanged, this, &SensorsWindow::updateConnectionStatus);

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &SensorsWindow::updateSensorStatus);
    _timer->start(0);

    updateConnectionStatus();
}

SensorsWindow::~SensorsWindow()
{
    delete ui;
    delete _timer;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Slot for connection status update
//----------------------------------------------------------------------
void SensorsWindow::updateConnectionStatus()
{
    // Possibly enable controls
    ui->groupBox_forceVoltageList->setEnabled(_pSensorControl->isConnected());
    ui->groupBox_forceList->setEnabled(_pSensorControl->isConnected());
    ui->groupBox_distanceList->setEnabled(_pSensorControl->isConnected());
    //ui->groupBox_forceVoltageList->setEnabled(true);
    //ui->groupBox_forceList->setEnabled(true);
    //ui->groupBox_distanceList->setEnabled(true);
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Manually create some GUI components for given number of sensors, etc.
// Assumes the functions will only be called once, in the constructor.
// Otherwise existing widgets will need deleting first.
//----------------------------------------------------------------------
void SensorsWindow::addForceVoltageListUi()
{
    int nForceLocations = _pSensorControl->getNForceLocations();

    if (nForceLocations == 0) {
        _label_forceLocationName.resize(1, nullptr);
        _label_forceLocationName[0] = new QLabel(ui->scrollAreaWidgetContents_forceVoltageList);
        _label_forceLocationName[0]->setText("No force sensors available");
        _label_forceLocationName[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        ui->gridLayout_forceVoltageList->addWidget(_label_forceLocationName[0], 0, 0, 1, 1);
        return;
    }

    // Create new widgets, in groups for each sensor location
    _label_forceLocationName.resize(nForceLocations, nullptr);
    _label_forceVoltage.resize(nForceLocations);
    _progressBar_forceVoltage.resize(nForceLocations);
    _label_forceVoltageVal.resize(nForceLocations);

    int vOffset = 0;
    for (int s = 0; s < nForceLocations; s++) {
        int nForceVoltages = _pSensorControl->getNForceRawValues(s);

        _label_forceVoltage[s].resize(nForceVoltages, nullptr);
        _progressBar_forceVoltage[s].resize(nForceVoltages, nullptr);
        _label_forceVoltageVal[s].resize(nForceVoltages, nullptr);

        _label_forceLocationName[s] = new QLabel(ui->scrollAreaWidgetContents_forceVoltageList);
        _label_forceLocationName[s]->setText(_pSensorControl->getForceLocationName(s));
        _label_forceLocationName[s]->setStyleSheet("font-weight: bold");
        ui->gridLayout_forceVoltageList->addWidget(_label_forceLocationName[s], 0+vOffset, 0, 1, 1);
        vOffset++;

        // Arrange in one row for every two voltages
        for (int v = 0; v < nForceVoltages; v += 2) {
            for (int i = 0; i < 2; i++) {
                if (v+i >= nForceVoltages)
                    break;

                _label_forceVoltage[s][v+i] = new QLabel(ui->scrollAreaWidgetContents_forceVoltageList);
                _label_forceVoltage[s][v+i]->setText(QString("V%1:").arg(v+i));
                _label_forceVoltage[s][v+i]->setAlignment(Qt::AlignRight);
                ui->gridLayout_forceVoltageList->addWidget(_label_forceVoltage[s][v+i], v/2+vOffset, 0+3*i, 1, 1);

                _progressBar_forceVoltage[s][v+i] = new QProgressBar(ui->scrollAreaWidgetContents_forceVoltageList);
                _progressBar_forceVoltage[s][v+i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                _progressBar_forceVoltage[s][v+i]->setRange(0, 100);
                _progressBar_forceVoltage[s][v+i]->setValue(0);
                ui->gridLayout_forceVoltageList->addWidget(_progressBar_forceVoltage[s][v+i], v/2+vOffset, 1+3*i, 1, 1);

                _label_forceVoltageVal[s][v+i] = new QLabel(ui->scrollAreaWidgetContents_forceVoltageList);
                _label_forceVoltageVal[s][v+i]->setText(QString("--.--"));
                _label_forceVoltageVal[s][v+i]->setMinimumWidth(70);
                ui->gridLayout_forceVoltageList->addWidget(_label_forceVoltageVal[s][v+i], v/2+vOffset, 2+3*i, 1, 1);
            }
        }

        vOffset += (nForceVoltages+1)/2 + 1;
    }
}

void SensorsWindow::addForceListUi()
{
    int nForceLocations = _pSensorControl->getNForceLocations();

    if (nForceLocations == 0) {
        _label_forceLocationName2.resize(1, nullptr);
        _label_forceLocationName2[0] = new QLabel(ui->scrollAreaWidgetContents_forceList);
        _label_forceLocationName2[0]->setText("No force sensors available");
        _label_forceLocationName2[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        ui->gridLayout_forceList->addWidget(_label_forceLocationName2[0], 0, 0, 1, 1);
        return;
    }

    // Create new widgets, in groups for each sensor location
    _label_forceLocationName2.resize(nForceLocations, nullptr);
    _label_forceComponentName.resize(nForceLocations);
    _label_forceComponentVal.resize(nForceLocations);

    int vOffset = 1;
    for (int s = 0; s < nForceLocations; s++) {
        _label_forceComponentName[s].resize(_pSensorControl->getNForceComponents(s), nullptr);
        _label_forceComponentVal[s].resize(_pSensorControl->getNForceComponents(s), nullptr);

        _label_forceLocationName2[s] = new QLabel(ui->scrollAreaWidgetContents_forceList);
        _label_forceLocationName2[s]->setText(_pSensorControl->getForceLocationName(s));
        _label_forceLocationName2[s]->setStyleSheet("font-weight: bold");
        ui->gridLayout_forceList->addWidget(_label_forceLocationName2[s], 0+vOffset, 0, 1, 1);

        // Force component names and values, arranged with one row for every three components
        for (int c = 0; c < _pSensorControl->getNForceComponents(s); c += 3) {
            for (int i = 0; i < 3; i++) {
                if (c+i >= _pSensorControl->getNForceComponents(s))
                    break;
                _label_forceComponentName[s][c+i] = new QLabel(ui->scrollAreaWidgetContents_forceList);
                _label_forceComponentName[s][c+i]->setText(QString("%1:").arg(_pSensorControl->getForceComponentName(s, c+i)));
                _label_forceComponentName[s][c+i]->setAlignment(Qt::AlignRight);
                ui->gridLayout_forceList->addWidget(_label_forceComponentName[s][c+i], c/3+vOffset, 1+2*i, 1, 1);

                _label_forceComponentVal[s][c+i] = new QLabel(ui->scrollAreaWidgetContents_forceList);
                _label_forceComponentVal[s][c+i]->setText(QString("--.--"));
                _label_forceComponentVal[s][c+i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                _label_forceComponentVal[s][c+i]->setMinimumWidth(70);
                ui->gridLayout_forceList->addWidget(_label_forceComponentVal[s][c+i], c/3+vOffset, 2+2*i, 1, 1);
            }
        }

        vOffset += (_pSensorControl->getNForceComponents(s)+2)/3 + 1;
    }

    // Push button
    _pushButton_zeroOffset = new QPushButton("Zero Offset", ui->scrollAreaWidgetContents_forceList);
    _pushButton_zeroOffset->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(_pushButton_zeroOffset, &QPushButton::clicked, this, &SensorsWindow::pushButton_zeroOffset_clicked);
    ui->gridLayout_forceList->addWidget(_pushButton_zeroOffset, 0, 0, 1, 7);
}

void SensorsWindow::addDistanceListUi()
{
    int nDistanceLocations = _pSensorControl->getNDistanceLocations();

    if (nDistanceLocations == 0) {
        _label_distanceLocationName.resize(1, nullptr);
        _label_distanceLocationName[0] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
        _label_distanceLocationName[0]->setText("No distance sensors available");
        _label_distanceLocationName[0]->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        ui->gridLayout_distanceList->addWidget(_label_distanceLocationName[0], 0, 0, 1, 1);
        return;
    }

    // Create new widgets, in groups for each sensor location
    _label_distanceLocationName.resize(nDistanceLocations, nullptr);
    _label_distanceMeasure.resize(nDistanceLocations);
    _progressBar_distanceMeasure.resize(nDistanceLocations);
    _label_distanceMeasureVal.resize(nDistanceLocations);
    _label_distanceComponentName.resize(nDistanceLocations);
    _label_distanceComponentVal.resize(nDistanceLocations);

    int vOffset = 1;
    for (int s = 0; s < nDistanceLocations; s++) {
        int nDistanceMeasures = _pSensorControl->getNDistanceRawValues(s);
        int nComponents = _pSensorControl->getNDistanceComponents(s);

        _label_distanceMeasure[s].resize(nDistanceMeasures, nullptr);
        _progressBar_distanceMeasure[s].resize(nDistanceMeasures, nullptr);
        _label_distanceMeasureVal[s].resize(nDistanceMeasures, nullptr);
        _label_distanceComponentName[s].resize(nComponents, nullptr);
        _label_distanceComponentVal[s].resize(nComponents, nullptr);

        _label_distanceLocationName[s] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
        _label_distanceLocationName[s]->setText(_pSensorControl->getDistanceLocationName(s));
        _label_distanceLocationName[s]->setStyleSheet("font-weight: bold");
        ui->gridLayout_distanceList->addWidget(_label_distanceLocationName[s], 0+vOffset, 0, 1, 1);
        vOffset++;

        // Distance measures form individual sensors: arrange in one row for every two measurements
        for (int m = 0; m < nDistanceMeasures; m += 2) {
            for (int i = 0; i < 2; i++) {
                if (m+i >= nDistanceMeasures)
                    break;

                _label_distanceMeasure[s][m+i] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
                _label_distanceMeasure[s][m+i]->setText(QString("D%1:").arg(m+i));
                _label_distanceMeasure[s][m+i]->setAlignment(Qt::AlignRight);
                ui->gridLayout_distanceList->addWidget(_label_distanceMeasure[s][m+i], m/2+vOffset, 0+3*i, 1, 1);

                _progressBar_distanceMeasure[s][m+i] = new QProgressBar(ui->scrollAreaWidgetContents_distanceList);
                _progressBar_distanceMeasure[s][m+i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
                _progressBar_distanceMeasure[s][m+i]->setRange(0, 255);
                _progressBar_distanceMeasure[s][m+i]->setValue(0);
                ui->gridLayout_distanceList->addWidget(_progressBar_distanceMeasure[s][m+i], m/2+vOffset, 1+3*i, 1, 1);

                _label_distanceMeasureVal[s][m+i] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
                _label_distanceMeasureVal[s][m+i]->setText(QString("--.--"));
                _label_distanceMeasureVal[s][m+i]->setMinimumWidth(70);
                ui->gridLayout_distanceList->addWidget(_label_distanceMeasureVal[s][m+i], m/2+vOffset, 2+3*i, 1, 1);
            }
        }
        vOffset += (nDistanceMeasures+1)/2 + 1;

        // Distance component names and values, arranged with one row for every three components
        for (int c = 0; c < nComponents; c += 3) {
            for (int i = 0; i < 3; i++) {
                if (c+i >= nComponents)
                    break;
                _label_distanceComponentName[s][c+i] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
                _label_distanceComponentName[s][c+i]->setText(QString("%1:").arg(_pSensorControl->getDistanceComponentName(s, c+i)));
                _label_distanceComponentName[s][c+i]->setAlignment(Qt::AlignRight);
                ui->gridLayout_distanceList->addWidget(_label_distanceComponentName[s][c+i], c/3+vOffset, 2*i, 1, 1);

                _label_distanceComponentVal[s][c+i] = new QLabel(ui->scrollAreaWidgetContents_distanceList);
                _label_distanceComponentVal[s][c+i]->setText(QString("--.--"));
                _label_distanceComponentVal[s][c+i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                _label_distanceComponentVal[s][c+i]->setMinimumWidth(70);
                ui->gridLayout_distanceList->addWidget(_label_distanceComponentVal[s][c+i], c/3+vOffset, 1+2*i, 1, 1);
            }
        }
        vOffset += (nComponents+2)/3 + 1;
    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Idle slot, for GUI update
//----------------------------------------------------------------------
void SensorsWindow::updateSensorStatus()
{
    std::vector<double> voltages;
    std::vector<double> forces;
    std::vector<double> distanceMeasures;
    std::vector<double> distances;
    bool isInRange;

    if (_pSensorControl->isConnected()) {
        for (int s = 0; s < _pSensorControl->getNForceLocations(); s++) {
            _pSensorControl->getCurrentForces(s, forces, voltages, isInRange);
            voltages.resize(_label_forceVoltageVal[s].size(), 0.0);
            forces.resize(_label_forceComponentVal[s].size(), 0.0);

            for (int v = 0; v < voltages.size(); v++) {
                _label_forceVoltageVal[s][v]->setText(QString::number(voltages[v], 'f', 3) + " V");
                _progressBar_forceVoltage[s][v]->setValue(int(voltages[v] * _progressBar_forceVoltage[s][v]->maximum() / 5.0 + 0.5));  // Scale 5V to progress bar maximum
            }

            for (int f = 0; f < forces.size(); f++) {
                if (isInRange)
                    _label_forceComponentVal[s][f]->setText(QString::number(forces[f], 'f', 3) + " N");
                else
                    _label_forceComponentVal[s][f]->setText("Out of range");
            }
        }

        for (int s = 0; s < _pSensorControl->getNDistanceLocations(); s++) {
            _pSensorControl->getCurrentDistances(s, distances, distanceMeasures, isInRange);
            distanceMeasures.resize(_label_distanceMeasureVal[s].size(), 0.0);
            distances.resize(_label_distanceComponentVal[s].size(), 0.0);

            for (int d = 0; d < distanceMeasures.size(); d++) {
                if (distanceMeasures[d] >= 255) {
                    _label_distanceMeasureVal[s][d]->setText(">= 255");
                    _progressBar_distanceMeasure[s][d]->setValue(_progressBar_distanceMeasure[s][d]->maximum());
                }
                else {
                    _label_distanceMeasureVal[s][d]->setText(QString::number(distanceMeasures[d]) + " mm");
                    _progressBar_distanceMeasure[s][d]->setValue(int(distanceMeasures[d] * _progressBar_distanceMeasure[s][d]->maximum() / 255.0 + 0.5));  // Scale 255mm to progress bar maximum
                }
            }

            for (int d = 0; d < distances.size(); d++) {
                if (isInRange)
                    _label_distanceComponentVal[s][d]->setText(QString::number(distances[d]) + " mm");
                else
                    _label_distanceComponentVal[s][d]->setText("Out of range");
            }
        }

    }
    else {
        for (int s = 0; s < _pSensorControl->getNForceLocations(); s++) {
            for (int v = 0; v < _label_forceVoltageVal[s].size(); v++) {
                _label_forceVoltageVal[s][v]->setText("--.--");
                _progressBar_forceVoltage[s][v]->setValue(0);
            }
            for (int f = 0; f < _label_forceComponentVal[s].size(); f++)
                _label_forceComponentVal[s][f]->setText("--.--");
        }

        for (int s = 0; s < _pSensorControl->getNDistanceLocations(); s++) {
            for (int d = 0; d < _label_distanceMeasureVal[s].size(); d++) {
                _label_distanceMeasureVal[s][d]->setText("--");
                _progressBar_distanceMeasure[s][d]->setValue(0);
            }
            for (int f = 0; f < _label_distanceComponentVal[s].size(); f++)
                _label_distanceComponentVal[s][f]->setText("--");
        }
    }
}

//----------------------------------------------------------------------
// Manual GUI slots
//----------------------------------------------------------------------
void SensorsWindow::pushButton_zeroOffset_clicked()
{
    if (_pSensorControl != nullptr && _pSensorControl->isConnected())
        _pSensorControl->zeroForces();
}
