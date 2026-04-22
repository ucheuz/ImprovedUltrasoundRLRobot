#ifndef INPUTSWINDOW_H
#define INPUTSWINDOW_H

#include <QDialog>
#include <QTimer>

#include "USRobotControlConfig.h"

#include "RobotControl/RobotControl.h"

namespace Ui {
class InputsWindow;
}

class InputsWindow : public QDialog
{
    Q_OBJECT

public:
    explicit InputsWindow(QWidget *parent = nullptr, RobotControl* robotControl = nullptr);
    ~InputsWindow();

public slots:
    void updateControlPanelConnectionStatus(QString status);
    void updateJoystickConnectionStatus(QString status);
#ifdef USE_INPUT_IGTL
    void updateIgtlConnectionStatus(QString status);
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    void updateSpaceMouseConnectionStatus(QString status);
#endif  // USE_INPUT_SPACEMOUSE
    void updateSurfPointsDataConnectionStatus(QString status);
    void updateRLConnectionStatus(QString status);

private:
    Ui::InputsWindow *ui;

    QTimer* _timer;

    RobotControl* _pRobotControl;

    ControlPanelInput* _pControlPanelInput;
    JoystickInput* _pJoystickInput;
#ifdef USE_INPUT_IGTL
    IGTLInput* _pIgtlInput;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    SpaceMouseInput* _pSpaceMouseInput;
#endif  // USE_INPUT_SPACEMOUSE
    SurfacePointsInput* _pSurfPointsInput;
    RLInput* _pRLInput;

private slots:
    void updateInputsData();

    // Joystick slots
    void on_lineEdit_joystickAddress_editingFinished();
    void on_lineEdit_joystickPort_editingFinished();

    // IGTL slots
#ifdef USE_INPUT_IGTL
    void on_lineEdit_igtlPort_editingFinished();
#endif  // USE_INPUT_IGTL

    // SpaceMouse slots

    // Surface points slots
    void on_lineEdit_surfacePointsAddress_editingFinished();
    void on_lineEdit_surfacePointsPort_editingFinished();
    
    // RL Input slots
    void on_lineEdit_rlPort_editingFinished();

    // Kinematics test slots
    void on_pushButton_goToNeutral_clicked();
    void on_pushButton_alignToSurface_clicked();

    void on_pushButton_kinematicsMoveLatPlus_clicked();
    void on_pushButton_kinematicsMoveLatMinus_clicked();
    void on_pushButton_kinematicsMoveElevPlus_clicked();
    void on_pushButton_kinematicsMoveElevMinus_clicked();
    void on_pushButton_kinematicsMoveAxPlus_clicked();
    void on_pushButton_kinematicsMoveAxMinus_clicked();

    void on_pushButton_kinematicsTiltLatPlus_clicked();
    void on_pushButton_kinematicsTiltLatMinus_clicked();
    void on_pushButton_kinematicsTiltElevPlus_clicked();
    void on_pushButton_kinematicsTiltElevMinus_clicked();
    void on_pushButton_kinematicsRotateAxPlus_clicked();
    void on_pushButton_kinematicsRotateAxMinus_clicked();
    void on_pushButton_kinematicsRotateAxPlus90_clicked();
    void on_pushButton_kinematicsRotateAxMinus90_clicked();

    void on_pushButton_kinematicsMoveXPlus_clicked();
    void on_pushButton_kinematicsMoveXMinus_clicked();
    void on_pushButton_kinematicsMoveYPlus_clicked();
    void on_pushButton_kinematicsMoveYMinus_clicked();
    void on_pushButton_kinematicsMoveZPlus_clicked();
    void on_pushButton_kinematicsMoveZMinus_clicked();

    void on_pushButton_runTestSequence_clicked();

    void on_tabWidget_inputs_currentChanged(int index);

    // General slots
    void on_checkBox_probeW_stateChanged(int arg1);
    void on_checkBox_probeX_stateChanged(int arg1);
    void on_checkBox_probeY_stateChanged(int arg1);
    void on_checkBox_probeZ_stateChanged(int arg1);
};

#endif // INPUTSWINDOW_H
