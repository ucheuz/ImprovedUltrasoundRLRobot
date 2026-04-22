#include <iostream>
#include <typeinfo>

#include "MotorsWindow.h"
#include "ui_MotorsWindow.h"

#include "MotorControl/MotorControl_accelStepper.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
MotorsWindow::MotorsWindow(QWidget *parent, IMotorControl* motorControl) :
    QDialog(parent),
    ui(new Ui::MotorsWindow)
{
    ui->setupUi(this);  // Create fixed UI
    this->setAttribute(Qt::WA_DeleteOnClose);

    if (motorControl == nullptr) {
        _timer = nullptr;
        _pMotorControl = nullptr;
        return;
    }

    _pMotorControl = motorControl;

    addMotorListUi(motorControl->getNMotors());  // Add variable UI, dependent on number of motors

    connect(_pMotorControl, &IMotorControl::connectionChanged, this, &MotorsWindow::updateConnectionStatus);

    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &MotorsWindow::updateMotorStatus);
    _timer->start(0);

    updateConnectionStatus();

    on_horizontalSlider_speedScale_valueChanged(ui->horizontalSlider_speedScale->value());
}

MotorsWindow::~MotorsWindow()
{
    disconnect(_pushButton_moveAbsAll, nullptr, this, nullptr);
    delete ui;
    delete _timer;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Slot for connection status update
//----------------------------------------------------------------------
void MotorsWindow::updateConnectionStatus()
{
    // Possibly enable controls
    bool enable = _pMotorControl->isConnected();

    ui->pushButton_pause->setEnabled(enable);
    ui->pushButton_cancel->setEnabled(enable);

    ui->groupBox_motorsList->setEnabled(enable);
    ui->scrollArea_motorsList->setEnabled(enable);
    //ui->groupBox_speedScale->setEnabled(enable);

    ui->groupBox_directSend->setEnabled(enable && (typeid(_pMotorControl) == typeid(MotorControl_accelStepper)));
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Manually create motors list GUI for given number of motors.
// Assumes this function will only be called once, in the constructor.
// Otherwise existing widgets will need deleting first.
//----------------------------------------------------------------------
void MotorsWindow::addMotorListUi(unsigned int nMotors)
{
    // Create new widgets, top row
    _pushButton_moveAbsAll = new QPushButton("Abs. All", ui->scrollAreaWidgetContents_motorsList);
    _pushButton_moveAbsAll->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(_pushButton_moveAbsAll, &QPushButton::clicked, this, &MotorsWindow::pushButton_moveAbsAll_clicked);
    ui->gridLayout_motorsList->addWidget(_pushButton_moveAbsAll, 0, 2, 1, 1);

    _pushButton_moveHomeAll = new QPushButton("Home All", ui->scrollAreaWidgetContents_motorsList);
    _pushButton_moveHomeAll->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    connect(_pushButton_moveHomeAll, &QPushButton::clicked, this, &MotorsWindow::pushButton_moveHomeAll_clicked);
    ui->gridLayout_motorsList->addWidget(_pushButton_moveHomeAll, 0, 3, 1, 1);

    /*bool allHomeable = true;
    for (unsigned int i = 0; i < nMotors; i++) {
        allHomeable &= _pMotorControl->isHomeable(i);
        if (!allHomeable)
            break;
    }
    _pushButton_moveHomeAll->setEnabled(allHomeable);*/
    _pushButton_moveHomeAll->setEnabled(true);

    _label_position = new QLabel("Position", ui->scrollAreaWidgetContents_motorsList);
    ui->gridLayout_motorsList->addWidget(_label_position, 0, 4, 1, 1);
    _label_speed = new QLabel("Speed", ui->scrollAreaWidgetContents_motorsList);
    ui->gridLayout_motorsList->addWidget(_label_speed, 0, 5, 1, 1);
    _label_maxSpeed = new QLabel("Max speed", ui->scrollAreaWidgetContents_motorsList);
    ui->gridLayout_motorsList->addWidget(_label_maxSpeed, 0, 6, 1, 1);
    _label_acceleration = new QLabel("Acceleration", ui->scrollAreaWidgetContents_motorsList);
    ui->gridLayout_motorsList->addWidget(_label_acceleration, 0, 7, 1, 1);
    _label_status = new QLabel("Status", ui->scrollAreaWidgetContents_motorsList);
    ui->gridLayout_motorsList->addWidget(_label_status, 0, 8, 1, 1);

    // Create new widgets, one row for each motor
    _label_motorNum.resize(nMotors, nullptr);
    _lineEdit_motorPos.resize(nMotors, nullptr);
    _pushButton_motorMoveAbs.resize(nMotors, nullptr);
    _pushButton_motorMoveHome.resize(nMotors, nullptr);
    _label_motorPos.resize(nMotors, nullptr);
    _label_motorSpeed.resize(nMotors, nullptr);
    _label_motorMaxSpeed.resize(nMotors, nullptr);
    _label_motorAcceleration.resize(nMotors, nullptr);
    _label_motorStatus.resize(nMotors, nullptr);

    for (unsigned int m = 0; m < nMotors; m++) {
        _label_motorNum[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorNum[m]->setText(QString("%1:").arg(m+1));
        ui->gridLayout_motorsList->addWidget(_label_motorNum[m], m+1, 0, 1, 1);

        _lineEdit_motorPos[m] = new QLineEdit("0", ui->scrollAreaWidgetContents_motorsList);
        _lineEdit_motorPos[m]->setMinimumWidth(70);
        _lineEdit_motorPos[m]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        ui->gridLayout_motorsList->addWidget(_lineEdit_motorPos[m], m+1, 1, 1, 1);

        _pushButton_motorMoveAbs[m] = new QPushButton("Abs.", ui->scrollAreaWidgetContents_motorsList);
        _pushButton_motorMoveAbs[m]->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        connect(_pushButton_motorMoveAbs[m], &QPushButton::clicked, this, &MotorsWindow::pushButton_motorMoveAbs_clicked);
        ui->gridLayout_motorsList->addWidget(_pushButton_motorMoveAbs[m], m+1, 2, 1, 1);

        _pushButton_motorMoveHome[m] = new QPushButton("Home", ui->scrollAreaWidgetContents_motorsList);
        _pushButton_motorMoveHome[m]->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        connect(_pushButton_motorMoveHome[m], &QPushButton::clicked, this, &MotorsWindow::pushButton_motorMoveHome_clicked);
        ui->gridLayout_motorsList->addWidget(_pushButton_motorMoveHome[m], m+1, 3, 1, 1);
        _pushButton_motorMoveHome[m]->setEnabled(_pMotorControl->isHomeable(m));

        _label_motorPos[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorPos[m]->setText(QString("--.--"));
        ui->gridLayout_motorsList->addWidget(_label_motorPos[m], m+1, 4, 1, 1);

        _label_motorSpeed[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorSpeed[m]->setText(QString("--.--"));
        ui->gridLayout_motorsList->addWidget(_label_motorSpeed[m], m+1, 5, 1, 1);

        _label_motorMaxSpeed[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorMaxSpeed[m]->setText(QString("--.--"));
        ui->gridLayout_motorsList->addWidget(_label_motorMaxSpeed[m], m+1, 6, 1, 1);

        _label_motorAcceleration[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorAcceleration[m]->setText(QString("--.--"));
        ui->gridLayout_motorsList->addWidget(_label_motorAcceleration[m], m+1, 7, 1, 1);

        _label_motorStatus[m] = new QLabel(ui->scrollAreaWidgetContents_motorsList);
        _label_motorStatus[m]->setText(QString("--"));
        ui->gridLayout_motorsList->addWidget(_label_motorStatus[m], m+1, 8, 1, 1);
    }
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Idle slot, for GUI update
//----------------------------------------------------------------------
void MotorsWindow::updateMotorStatus()
{
    if (_pMotorControl != nullptr && _pMotorControl->isConnected()) {
        std::vector<double> positions, speeds;
        std::vector<float> maxSpeed, acceleration;
        std::vector<QString> status;

        _pMotorControl->getAllMotorPositions(positions);
        positions.resize(_label_motorNum.size(), 0.0);
        _pMotorControl->getAllMotorSpeeds(speeds);
        speeds.resize(_label_motorNum.size(), 0.0);
        _pMotorControl->getAllMotorParams(maxSpeed, acceleration);
        maxSpeed.resize(_label_motorNum.size(), 0.0);
        acceleration.resize(_label_motorNum.size(), 0.0);
        _pMotorControl->getAllStatusStrings(status);
        status.resize(_label_motorNum.size(), "No motor");

        for (int m = 0; m < _label_motorNum.size(); m++) {
            _label_motorPos[m]->setText(QString::number(positions[m], 'f', 1));
            _label_motorSpeed[m]->setText(QString::number(speeds[m], 'f', 1));
            _label_motorMaxSpeed[m]->setText(QString::number(maxSpeed[m], 'f', 1));
            _label_motorAcceleration[m]->setText(QString::number(acceleration[m], 'f', 1));
            _label_motorStatus[m]->setText(status[m]);
        }
    }
    else {
        for (int m = 0; m < _label_motorNum.size(); m++) {
            _label_motorPos[m]->setText("--.--");
            _label_motorSpeed[m]->setText("--.--");
            _label_motorMaxSpeed[m]->setText("--.--");
            _label_motorAcceleration[m]->setText("--.--");
            _label_motorStatus[m]->setText("--");
        }
    }
}

//----------------------------------------------------------------------
// Manual GUI slots
//----------------------------------------------------------------------
void MotorsWindow::pushButton_moveAbsAll_clicked()
{
    bool ok, allOk = true;
    std::vector<double> dVal;
    std::vector<double> currentPosition, position;
    _pMotorControl->getAllMotorPositions(currentPosition);
    int nMotors = (currentPosition.size() < _lineEdit_motorPos.size()) ? int(currentPosition.size()) : _lineEdit_motorPos.size();
    dVal.resize(nMotors, 0.0);
    position.resize(nMotors, 0.0);
    // Target positions in GUI
    for (int m = 0; m < nMotors; m++) {
        dVal[m] = _lineEdit_motorPos[m]->text().toDouble(&ok);
        position[m] = ok ? dVal[m] : currentPosition[m];
        allOk &= ok;
    }
    if (allOk)
        _pMotorControl->moveAllAbsolute(position);
}

void MotorsWindow::pushButton_motorMoveAbs_clicked()
{
    QObject* obj = sender();

    for (int m = 0; m < _pushButton_motorMoveAbs.size(); m++) {
        if (obj == _pushButton_motorMoveAbs[m]) {
            bool ok;
            double position = _lineEdit_motorPos[m]->text().toDouble(&ok);
            if (ok)
                _pMotorControl->moveAbsolute(m, position);
            return;
        }
    }

    std::cerr << "Error in MotorsWindow_AccelStepper::pushButton_motorMoveAbs_clicked: cannot identify sending push button." << std::endl;
}

void MotorsWindow::pushButton_moveHomeAll_clicked()
{
    _pMotorControl->moveAllHome();
}

void MotorsWindow::pushButton_motorMoveHome_clicked()
{
    QObject* obj = sender();

    for (unsigned int m = 0; m < _pushButton_motorMoveHome.size(); m++) {
        if (obj == _pushButton_motorMoveHome[m]) {
            _pMotorControl->moveHome(m);
            return;
        }
    }

    std::cerr << "Error in MotorsWindow_AccelStepper::pushButton_motorHome_clicked: cannot identify sending push button." << std::endl;
}

//----------------------------------------------------------------------
// Automatic GUI slots
//----------------------------------------------------------------------
void MotorsWindow::on_horizontalSlider_speedScale_valueChanged(int value)
{
    double speedScale = double(value) / double(ui->horizontalSlider_speedScale->maximum());
    _pMotorControl->setSpeedScale(speedScale);
    ui->label_speedScaleVal->setText(QString::number(speedScale, 'f', 3));
}

void MotorsWindow::on_pushButton_directSend_clicked()
{
    if (typeid(_pMotorControl) == typeid(MotorControl_accelStepper)) {
        MotorControl_accelStepper* ptr = dynamic_cast<MotorControl_accelStepper*>(_pMotorControl);
        ptr->send(ui->lineEdit_directSend->text());
        ui->lineEdit_directSend->setText(QString());
    }
}

void MotorsWindow::on_pushButton_pause_clicked()
{
    _pMotorControl->togglePause();
}

void MotorsWindow::on_pushButton_cancel_clicked()
{
    _pMotorControl->cancelMove();
}
