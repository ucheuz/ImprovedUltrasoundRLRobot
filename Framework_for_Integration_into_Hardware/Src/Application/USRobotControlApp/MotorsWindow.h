#ifndef MOTORSWINDOW_ACCELSTEPPER_H
#define MOTORSWINDOW_ACCELSTEPPER_H

#include <vector>

#include <QDialog>
#include <QTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include "MotorControl/IMotorControl.h"

namespace Ui {
class MotorsWindow;
}

class MotorsWindow : public QDialog
{
    Q_OBJECT

public:
    explicit MotorsWindow(QWidget *parent = nullptr, IMotorControl* motorControl = nullptr);
    ~MotorsWindow();

public slots:
    void updateConnectionStatus();

private:
    Ui::MotorsWindow *ui;

    QTimer* _timer;

    IMotorControl* _pMotorControl;

    // Motor list UI
    QPushButton* _pushButton_moveAbsAll;
    QPushButton* _pushButton_moveHomeAll;
    QLabel* _label_position;
    QLabel* _label_speed;
    QLabel* _label_maxSpeed;
    QLabel* _label_acceleration;
    QLabel* _label_status;
    std::vector<QLabel*> _label_motorNum;
    std::vector<QLineEdit*> _lineEdit_motorPos;
    std::vector<QPushButton*> _pushButton_motorMoveAbs;
    std::vector<QPushButton*> _pushButton_motorMoveHome;
    std::vector<QLabel*> _label_motorPos;
    std::vector<QLabel*> _label_motorSpeed;
    std::vector<QLabel*> _label_motorMaxSpeed;
    std::vector<QLabel*> _label_motorAcceleration;
    std::vector<QLabel*> _label_motorStatus;

    void addMotorListUi(unsigned int nMotors);

private slots:
    void updateMotorStatus();

    // Manual GUI slots
    void pushButton_moveAbsAll_clicked();
    void pushButton_motorMoveAbs_clicked();
    void pushButton_moveHomeAll_clicked();
    void pushButton_motorMoveHome_clicked();

    // Automatic GUI slots
    void on_horizontalSlider_speedScale_valueChanged(int value);
    void on_pushButton_directSend_clicked();
    void on_pushButton_pause_clicked();
    void on_pushButton_cancel_clicked();
};

#endif // MOTORSWINDOW_ACCELSTEPPER_H
