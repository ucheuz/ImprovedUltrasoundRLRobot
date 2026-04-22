#include "InputsWindow.h"
#include "ui_InputsWindow.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
InputsWindow::InputsWindow(QWidget *parent, RobotControl* robotControl) :
    QDialog(parent),
    ui(new Ui::InputsWindow)
{
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);

    if (robotControl == nullptr) {
        _pJoystickInput = nullptr;
        return;
    }

    _pRobotControl = robotControl;

    _pRobotControl->clearActiveProbes();

    if (_pRobotControl->getNProbes() >= 1) {
        ui->checkBox_probeW->setEnabled(true);
        if (ui->checkBox_probeW->isChecked())
            _pRobotControl->setActiveProbe(0, true);
    }
    if (_pRobotControl->getNProbes() >= 2) {
        ui->checkBox_probeX->setEnabled(true);
        if (ui->checkBox_probeX->isChecked())
            _pRobotControl->setActiveProbe(1, true);
    }
    if (_pRobotControl->getNProbes() >= 3) {
        ui->checkBox_probeY->setEnabled(true);
        if (ui->checkBox_probeY->isChecked())
            _pRobotControl->setActiveProbe(2, true);
    }
    if (_pRobotControl->getNProbes() >= 4) {
        ui->checkBox_probeZ->setEnabled(true);
        if (ui->checkBox_probeZ->isChecked())
            _pRobotControl->setActiveProbe(3, true);
    }

    // Control panel
    _pControlPanelInput = robotControl->getControlPanelInputPtr();

    connect(_pControlPanelInput, &ControlPanelInput::connectionStatus, this, &InputsWindow::updateControlPanelConnectionStatus);
    updateControlPanelConnectionStatus("Disconnected");

    // Joystick
    _pJoystickInput = robotControl->getJoystickInputPtr();

    // Set default address and port values
    on_lineEdit_joystickAddress_editingFinished();
    on_lineEdit_joystickPort_editingFinished();

    connect(_pJoystickInput, &JoystickInput::connectionStatus, this, &InputsWindow::updateJoystickConnectionStatus);
    //connect(_pJoystickInput, &JoystickInput::connectionChanged, this, &InputsWindow::updateJoystickConnectionStatus);
    updateJoystickConnectionStatus("Disconnected");

    // IGTL
#ifdef USE_INPUT_IGTL
    _pIgtlInput = robotControl->getIGTLInputPtr();

    on_lineEdit_igtlPort_editingFinished();

    connect(_pIgtlInput, &IGTLInput::connectionStatus, this, &InputsWindow::updateIgtlConnectionStatus);
    updateIgtlConnectionStatus("Disconnected");
#else
    ui->tabWidget_inputs->setTabVisible(2, false);  // Tab still exists in memory, but is not visible in GUI
    ui->tabWidget_inputs->setTabEnabled(2, false);
#endif  // USE_INPUT_IGTL

    // SpaceMouse
#ifdef USE_INPUT_SPACEMOUSE
    _pSpaceMouseInput = robotControl->getSpaceMouseInputPtr();

    connect(_pSpaceMouseInput, &SpaceMouseInput::connectionStatus, this, &InputsWindow::updateSpaceMouseConnectionStatus);
    updateSpaceMouseConnectionStatus("Disconnected");
#else
    ui->tabWidget_inputs->setTabVisible(3, false);  // Tab still exists in memory, but is not visible in GUI
    ui->tabWidget_inputs->setTabEnabled(3, false);
#endif  // USE_INPUT_SPACEMOUSE

    // Surface points data
    _pSurfPointsInput = robotControl->getSurfPointsInputPtr();

    // Set default address and port values
    on_lineEdit_surfacePointsAddress_editingFinished();
    on_lineEdit_surfacePointsPort_editingFinished();

    connect(_pSurfPointsInput, &SurfacePointsInput::connectionStatus, this, &InputsWindow::updateSurfPointsDataConnectionStatus);
    updateSurfPointsDataConnectionStatus("Disconnected");
    
    // RL Input
    _pRLInput = robotControl->getRLInputPtr();

    on_lineEdit_rlPort_editingFinished();  // Apply default port from UI

    connect(_pRLInput, &RLInput::connectionStatus, this, &InputsWindow::updateRLConnectionStatus);
    updateRLConnectionStatus("Disconnected");

    // Timer for GUI update
    _timer = new QTimer(this);
    connect(_timer, &QTimer::timeout, this, &InputsWindow::updateInputsData);
    _timer->start(50);
}

InputsWindow::~InputsWindow()
{
    //if (_pJoystickInput != nullptr)
    //    disconnect(_pJoystickInput, nullptr, this, nullptr);  // Not necessary and causes a crash if the original _joystickInput is already deleted
    disconnect(_timer, nullptr, this, nullptr);

    _pJoystickInput = nullptr;
#ifdef USE_INPUT_IGTL
    _pIgtlInput = nullptr;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    _pSpaceMouseInput = nullptr;
#endif  // USE_INPUT_SPACEMOUSE
    _pSurfPointsInput = nullptr;
    _pRLInput = nullptr;
    _pRobotControl = nullptr;

    delete ui;
    delete _timer;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Slots for connection status updates
//----------------------------------------------------------------------
void InputsWindow::updateControlPanelConnectionStatus(QString status)
{
    ui->label_controlPanelConnectionStatus->setText(status);
    // Possibly enable control panel controls
    //bool enabled = _pControlPanelInput != nullptr && _pControlPanelInput->isConnected();
}

void InputsWindow::updateJoystickConnectionStatus(QString status)
{
    ui->label_joystickConnectionStatus->setText(status);
    // Possibly enable joystick controls
    bool enabled = _pJoystickInput != nullptr && _pJoystickInput->isConnected();

    ui->groupBox_joystickPoseData->setEnabled(enabled);
}

#ifdef USE_INPUT_IGTL
void InputsWindow::updateIgtlConnectionStatus(QString status)
{
    ui->label_igtlConnectionStatus->setText(status);
    // Possibly enable igtl controls
    bool enabled = _pIgtlInput->isConnected();

    ui->groupBox_igtlPoseData->setEnabled(enabled);
}
#endif  // USE_INPUT_IGTL

#ifdef USE_INPUT_SPACEMOUSE
void InputsWindow::updateSpaceMouseConnectionStatus(QString status)
{
    ui->label_SpaceMouseConnectionStatus->setText(status);
    // Possibly enable SpaceMouse controls
    bool enabled = _pSpaceMouseInput->isConnected();

    ui->groupBox_SpaceMousePoseData->setEnabled(enabled);
}
#endif  // USE_INPUT_SPACEMOUSE

void InputsWindow::updateSurfPointsDataConnectionStatus(QString status)
{
    ui->label_surfacePointsConnectionStatus->setText(status);
    // Possibly enable Surface Points controls
    bool enabled = _pSurfPointsInput->isConnected();

    ui->groupBox_surfacePointsPoseData->setEnabled(enabled);
}

void InputsWindow::updateRLConnectionStatus(QString status)
{
    ui->label_rlConnectionStatus->setText(status);
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Idle slot, for GUI update
//----------------------------------------------------------------------
void InputsWindow::updateInputsData()
{
    if (_pJoystickInput != nullptr && _pJoystickInput->isConnected()) {
        
        std::vector<double> poseData;
        bool valid = _pJoystickInput->getCurrentPoseData(poseData);
        poseData.resize(7, 0.0);
        ui->label_joystickXVal->setText(QString::number(poseData[0], 'f', 2));
        ui->label_joystickYVal->setText(QString::number(poseData[1], 'f', 2));
        ui->label_joystickZVal->setText(QString::number(poseData[2], 'f', 2));
        ui->label_joystickQ0Val->setText(QString::number(poseData[3], 'f', 4));
        ui->label_joystickQXVal->setText(QString::number(poseData[4], 'f', 4));
        ui->label_joystickQYVal->setText(QString::number(poseData[5], 'f', 4));
        ui->label_joystickQZVal->setText(QString::number(poseData[6], 'f', 4));
        if (valid) {
            ui->label_joystickValid->setText("Valid pose");
            ui->label_joystickValid->setStyleSheet("QLabel { background-color : green; color : black; }");
        }
        else {
            ui->label_joystickValid->setText("Invalid pose");
            ui->label_joystickValid->setStyleSheet("QLabel { background-color : red; color : black; }");
        }
    }
    else {
        ui->label_joystickXVal->setText("--.--");
        ui->label_joystickYVal->setText("--.--");
        ui->label_joystickZVal->setText("--.--");
        ui->label_joystickQ0Val->setText("--.--");
        ui->label_joystickQXVal->setText("--.--");
        ui->label_joystickQYVal->setText("--.--");
        ui->label_joystickQZVal->setText("--.--");
        ui->label_joystickValid->setText("No data");
        ui->label_joystickValid->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
    
#ifdef USE_INPUT_IGTL
    if (_pIgtlInput != nullptr && _pIgtlInput->isConnected()) {
        
        std::vector<double> poseData;
        bool valid = _pIgtlInput->getCurrentPoseData(ui->spinBox_probeNum->value()-1, poseData);
        poseData.resize(7, 0.0);
        ui->label_igtlXVal->setText(QString::number(poseData[0], 'f', 2));
        ui->label_igtlYVal->setText(QString::number(poseData[1], 'f', 2));
        ui->label_igtlZVal->setText(QString::number(poseData[2], 'f', 2));
        ui->label_igtlQ0Val->setText(QString::number(poseData[3], 'f', 4));
        ui->label_igtlQXVal->setText(QString::number(poseData[4], 'f', 4));
        ui->label_igtlQYVal->setText(QString::number(poseData[5], 'f', 4));
        ui->label_igtlQZVal->setText(QString::number(poseData[6], 'f', 4));
        if (valid) {
            ui->label_igtlValid->setText("Valid pose");
            ui->label_igtlValid->setStyleSheet("QLabel { background-color : green; color : black; }");
        }
        else {
            ui->label_igtlValid->setText("Invalid pose");
            ui->label_igtlValid->setStyleSheet("QLabel { background-color : red; color : black; }");
        }
    }
    else {
        ui->label_igtlXVal->setText("--.--");
        ui->label_igtlYVal->setText("--.--");
        ui->label_igtlZVal->setText("--.--");
        ui->label_igtlQ0Val->setText("--.--");
        ui->label_igtlQXVal->setText("--.--");
        ui->label_igtlQYVal->setText("--.--");
        ui->label_igtlQZVal->setText("--.--");
        ui->label_igtlValid->setText("No data");
        ui->label_igtlValid->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
#endif  // USE_INPUT_IGTL
    
#ifdef USE_INPUT_SPACEMOUSE
    if (_pSpaceMouseInput != nullptr && _pSpaceMouseInput->isConnected()) {
        
        std::vector<double> poseData;
        bool valid = _pSpaceMouseInput->getCurrentPoseData(poseData);
        poseData.resize(6, 0.0);
        ui->label_SpaceMouseXVal->setText(QString::number(poseData[0], 'f', 2));
        ui->label_SpaceMouseYVal->setText(QString::number(poseData[1], 'f', 2));
        ui->label_SpaceMouseZVal->setText(QString::number(poseData[2], 'f', 2));
        ui->label_SpaceMousePitchVal->setText(QString::number(poseData[3], 'f', 4));
        ui->label_SpaceMouseRollVal->setText(QString::number(poseData[4], 'f', 4));
        ui->label_SpaceMouseYawVal->setText(QString::number(poseData[5], 'f', 4));
        if (valid) {
            ui->label_SpaceMouseValid->setText("Valid pose");
            ui->label_SpaceMouseValid->setStyleSheet("QLabel { background-color : green; color : black; }");
        }
        else {
            ui->label_SpaceMouseValid->setText("Invalid pose");
            ui->label_SpaceMouseValid->setStyleSheet("QLabel { background-color : red; color : black; }");
        }
    }
    else {
        ui->label_SpaceMouseXVal->setText("--.--");
        ui->label_SpaceMouseYVal->setText("--.--");
        ui->label_SpaceMouseZVal->setText("--.--");
        ui->label_SpaceMousePitchVal->setText("--.--");
        ui->label_SpaceMouseRollVal->setText("--.--");
        ui->label_SpaceMouseYawVal->setText("--.--");
        ui->label_SpaceMouseValid->setText("No data");
        ui->label_SpaceMouseValid->setStyleSheet("QLabel { background-color : red; color : black; }");
    }
#endif  // USE_INPUT_SPACEMOUSE

    if (_pSurfPointsInput != nullptr && _pSurfPointsInput->isConnected()) {
        
        std::vector<double> poseData;
        bool valid = _pSurfPointsInput->getCurrentPoseData(poseData);
        poseData.resize(6, 0.0);
        ui->label_surfacePointsXVal->setText(QString::number(poseData[0], 'f', 2));
        ui->label_surfacePointsYVal->setText(QString::number(poseData[1], 'f', 2));
        ui->label_surfacePointsZVal->setText(QString::number(poseData[2], 'f', 2));
        ui->label_surfacePointsNXVal->setText(QString::number(poseData[3], 'f', 4));
        ui->label_surfacePointsNYVal->setText(QString::number(poseData[4], 'f', 4));
        ui->label_surfacePointsNZVal->setText(QString::number(poseData[5], 'f', 4));
        if (valid) {
            ui->label_surfacePointsValid->setText("Valid pose");
            ui->label_surfacePointsValid->setStyleSheet("QLabel { background-color : green; color : black; }");
        }
        else {
            ui->label_surfacePointsValid->setText("Invalid pose");
            ui->label_surfacePointsValid->setStyleSheet("QLabel { background-color : red; color : black; }");
        }
    }

    else {
        ui->label_surfacePointsXVal->setText("--.--");
        ui->label_surfacePointsYVal->setText("--.--");
        ui->label_surfacePointsZVal->setText("--.--");
        ui->label_surfacePointsNXVal->setText("--.--");
        ui->label_surfacePointsNYVal->setText("--.--");
        ui->label_surfacePointsNZVal->setText("--.--");
        ui->label_surfacePointsValid->setText("No data");
        ui->label_surfacePointsValid->setStyleSheet("QLabel { background-color : red; color : black; }");
    }

    // RL Input status
    if (_pRLInput != nullptr && _pRLInput->isConnected()) {
        ui->label_rlLastCommand->setText("Last command: RL model connected");
        ui->label_rlLastCommand->setStyleSheet("QLabel { color : green; }");
    }
    else {
        ui->label_rlLastCommand->setText("Last command: none (not connected)");
        ui->label_rlLastCommand->setStyleSheet("QLabel { color : grey; }");
    }
}

//----------------------------------------------------------------------
// Joystick GUI slots
//----------------------------------------------------------------------
void InputsWindow::on_lineEdit_joystickAddress_editingFinished()
{
    if (_pJoystickInput == nullptr)
        return;

    QHostAddress address;
    if (address.setAddress(ui->lineEdit_joystickAddress->text())) {
        _pJoystickInput->setIpAddress(address);
        ui->lineEdit_joystickAddress->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_joystickAddress->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}

void InputsWindow::on_lineEdit_joystickPort_editingFinished()
{
    if (_pJoystickInput == nullptr)
        return;

    bool ok;
    unsigned short port = ui->lineEdit_joystickPort->text().toUShort(&ok);
    if (ok) {
        _pJoystickInput->setPort(port);
        ui->lineEdit_joystickPort->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_joystickPort->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}

//----------------------------------------------------------------------
// IGTL GUI slots
//----------------------------------------------------------------------
#ifdef USE_INPUT_IGTL
void InputsWindow::on_lineEdit_igtlPort_editingFinished()
{
    bool ok;
    unsigned int port = ui->lineEdit_igtlPort->text().toUInt(&ok);
    if (ok) {
        _pIgtlInput->setPort(port);
        ui->lineEdit_igtlPort->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_igtlPort->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}
#endif  // USE_INPUT_IGTL

//----------------------------------------------------------------------
// Surface points GUI slots
//----------------------------------------------------------------------
void InputsWindow::on_lineEdit_surfacePointsAddress_editingFinished()
{
    if (_pSurfPointsInput == nullptr)
        return;

    QHostAddress address;
    if (address.setAddress(ui->lineEdit_surfacePointsAddress->text())) {
        _pSurfPointsInput->setIpAddress(address);
        ui->lineEdit_surfacePointsAddress->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_surfacePointsAddress->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}

//----------------------------------------------------------------------
// RL Input GUI slots
//----------------------------------------------------------------------
void InputsWindow::on_lineEdit_rlPort_editingFinished()
{
    if (_pRLInput == nullptr)
        return;

    bool ok;
    quint16 port = ui->lineEdit_rlPort->text().toUShort(&ok);
    if (ok && port > 0) {
        _pRLInput->stopListening();
        _pRLInput->startListening(port);
        ui->lineEdit_rlPort->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_rlPort->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}


void InputsWindow::on_lineEdit_surfacePointsPort_editingFinished()
{
    if (_pSurfPointsInput == nullptr)
        return;

    bool ok;
    unsigned short port = ui->lineEdit_surfacePointsPort->text().toUShort(&ok);
    if (ok) {
        _pSurfPointsInput->setPort(port);
        ui->lineEdit_surfacePointsPort->setStyleSheet("QLineEdit { background-color : white; color : black; }");
    }
    else {
        ui->lineEdit_surfacePointsPort->setStyleSheet("QLineEdit { background-color : red; color : black; }");
    }
}

//----------------------------------------------------------------------
// Kinematics test GUI slots
//----------------------------------------------------------------------
void InputsWindow::on_pushButton_goToNeutral_clicked()
{
    _pRobotControl->goToNeutral();
}

void InputsWindow::on_pushButton_alignToSurface_clicked()
{
    _pRobotControl->alignToSurface();  // Move probes to nearest point on surface and align to surface normal
}

void InputsWindow::on_pushButton_kinematicsMoveLatPlus_clicked()
{
    _pRobotControl->surfaceFollowX(10);  // 10 mm lateral movement along surface
}

void InputsWindow::on_pushButton_kinematicsMoveLatMinus_clicked()
{
    _pRobotControl->surfaceFollowX(-10);  // -10 mm lateral movement along surface
}

void InputsWindow::on_pushButton_kinematicsMoveElevPlus_clicked()
{
    _pRobotControl->surfaceFollowZ(10);  // 10 mm elevational movement
}

void InputsWindow::on_pushButton_kinematicsMoveElevMinus_clicked()
{
    _pRobotControl->surfaceFollowZ(-10);  // -10 mm elevational movement
}

void InputsWindow::on_pushButton_kinematicsMoveAxPlus_clicked()
{
    _pRobotControl->moveProbeY(10);  // Advance 10 mm axially
}

void InputsWindow::on_pushButton_kinematicsMoveAxMinus_clicked()
{
    _pRobotControl->moveProbeY(-10);  // Retract 10 mm axially
}

void InputsWindow::on_pushButton_kinematicsTiltLatPlus_clicked()
{
    _pRobotControl->rotateZ(5);  // 5 degrees lateral tilt
}

void InputsWindow::on_pushButton_kinematicsTiltLatMinus_clicked()
{
    _pRobotControl->rotateZ(-5);  // -5 degrees lateral tilt
}

void InputsWindow::on_pushButton_kinematicsTiltElevPlus_clicked()
{
    _pRobotControl->rotateX(5);  // 5 degrees elevational tilt
}

void InputsWindow::on_pushButton_kinematicsTiltElevMinus_clicked()
{
    _pRobotControl->rotateX(-5);  // -5 degrees elevational tilt
}

void InputsWindow::on_pushButton_kinematicsRotateAxPlus_clicked()
{
    _pRobotControl->rotateY(10);  // 10 degrees axial rotation
}

void InputsWindow::on_pushButton_kinematicsRotateAxMinus_clicked()
{
    _pRobotControl->rotateY(-10);  // -10 degrees axial rotation
}

void InputsWindow::on_pushButton_kinematicsRotateAxPlus90_clicked()
{
    _pRobotControl->rotateY(90);  // 90 degrees axial rotation
}

void InputsWindow::on_pushButton_kinematicsRotateAxMinus90_clicked()
{
    _pRobotControl->rotateY(-90);  // -90 degrees axial rotation
}

void InputsWindow::on_pushButton_kinematicsMoveXPlus_clicked()
{
    _pRobotControl->moveX(10);  // 10 mm X movement
}

void InputsWindow::on_pushButton_kinematicsMoveXMinus_clicked()
{
    _pRobotControl->moveX(-10);  // -10 mm X movement
}

void InputsWindow::on_pushButton_kinematicsMoveYPlus_clicked()
{
    _pRobotControl->moveY(10);  // 10 mm Y movement
}

void InputsWindow::on_pushButton_kinematicsMoveYMinus_clicked()
{
    _pRobotControl->moveY(-10);  // -10 mm Y movement
}

void InputsWindow::on_pushButton_kinematicsMoveZPlus_clicked()
{
    _pRobotControl->moveZ(10);  // 10 mm Z movement
}

void InputsWindow::on_pushButton_kinematicsMoveZMinus_clicked()
{
    _pRobotControl->moveZ(-10);  // -10 mm Z movement
}

//----------------------------------------------------------------------
// Plan and run a pre-programmed sequence of moves.
// This is intended only as a testing function.
//----------------------------------------------------------------------
void InputsWindow::on_pushButton_runTestSequence_clicked()
{
    std::vector<std::vector<TRTrans3D> > sequence;
    std::vector<TRTrans3D> probeCurrentPoses, probeTargetPoses;
    _pRobotControl->getCurrentProbePoses(probeCurrentPoses);

    // Plan the sequence:

    // Align to surface
    _pRobotControl->planAlignToSurface(probeCurrentPoses, probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Move elevationally 8 * +10mm steps
    /*for (int i = 0; i < 8; i++) {
        _pRobotControl->planSurfaceFollowZ(10.0, sequence.back(), probeTargetPoses);
        sequence.push_back(probeTargetPoses);
    }

    // Move elevationally 18 * -10mm steps
    for (int i = 0; i < 18; i++) {
        _pRobotControl->planSurfaceFollowZ(-10.0, sequence.back(), probeTargetPoses);
        sequence.push_back(probeTargetPoses);
    }

    // Move elevationally 10 * +10mm steps
    for (int i = 0; i < 10; i++) {
        _pRobotControl->planSurfaceFollowZ(10.0, sequence.back(), probeTargetPoses);
        sequence.push_back(probeTargetPoses);
    }*/

    // Elevational sweeping
    _pRobotControl->planSurfaceFollowZ(80.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    _pRobotControl->planSurfaceFollowZ(-160.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    _pRobotControl->planSurfaceFollowZ(80.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Tilting
    _pRobotControl->planRotateX(30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    _pRobotControl->planRotateX(-60.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    _pRobotControl->planRotateX(30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Axial rotation
    for (int i = 0; i < 3; i++) {
        _pRobotControl->planRotateY(-30.0, sequence.back(), probeTargetPoses);
        sequence.push_back(probeTargetPoses);
    }

    // Lateral movement
    _pRobotControl->planSurfaceFollowX(60.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Axial rotation
    _pRobotControl->planRotateY(30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Elevational movement
    _pRobotControl->planSurfaceFollowZ(-30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Lateral movement
    _pRobotControl->planSurfaceFollowX(-30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Elevational movement
    _pRobotControl->planSurfaceFollowZ(30.0, sequence.back(), probeTargetPoses);
    sequence.push_back(probeTargetPoses);

    // Run the sequence
    _pRobotControl->runSequence(sequence);
}

//----------------------------------------------------------------------
// Control mode selection GUI slot
//----------------------------------------------------------------------
void InputsWindow::on_tabWidget_inputs_currentChanged(int index)
{
    switch (index) {
    case 1:
        _pRobotControl->selectInput(ExternalInputs::CONTROL_PANEL);
        break;

    case 2:
#ifdef USE_INPUT_IGTL
        _pRobotControl->selectInput(ExternalInputs::IGTL);
#else
        _pRobotControl->selectInput(ExternalInputs::NONE);
#endif  // USE_INPUT_IGTL
        break;

    case 3:
#ifdef USE_INPUT_SPACEMOUSE
        _pRobotControl->selectInput(ExternalInputs::SPACE_MOUSE);
#else
        _pRobotControl->selectInput(ExternalInputs::NONE);
#endif  // USE_INPUT_SPACEMOUSE
        break;

    case 4:
        _pRobotControl->selectInput(ExternalInputs::JOYSTICK);
        break;

    case 5:
        _pRobotControl->selectInput(ExternalInputs::SURF_POINT);
        break;
            
    case 6:
        _pRobotControl->selectInput(ExternalInputs::RL_INPUT);
        break;

    case 0:
    default:
        _pRobotControl->selectInput(ExternalInputs::NONE);
        break;
    }
}

//----------------------------------------------------------------------
// Probe selection GUI slots
//----------------------------------------------------------------------

void InputsWindow::on_checkBox_probeW_stateChanged(int arg1)
{
    _pRobotControl->setActiveProbe(0, arg1 > 0);
}

void InputsWindow::on_checkBox_probeX_stateChanged(int arg1)
{
    _pRobotControl->setActiveProbe(1, arg1 > 0);
}

void InputsWindow::on_checkBox_probeY_stateChanged(int arg1)
{
    _pRobotControl->setActiveProbe(2, arg1 > 0);
}

void InputsWindow::on_checkBox_probeZ_stateChanged(int arg1)
{
    _pRobotControl->setActiveProbe(3, arg1 > 0);
}
