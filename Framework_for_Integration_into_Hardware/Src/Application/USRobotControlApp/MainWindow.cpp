#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

#include "MainWindow.h"
#include "ui_MainWindow.h"

//#include "TRTrans3D.h"  // From TransMatrix library

#include <vtkXMLPolyDataReader.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>

//#define KINEMATICS_SCREENSHOTS  // For recording screenshots of each move, e.g. for kinematics videos
#ifdef KINEMATICS_SCREENSHOTS
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#endif

#include <QFileInfo>
#include <QMessageBox>
#include <QFileDialog>

#include <QDomDocument>
#include <QString>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
MainWindow::MainWindow(QWidget *parent, RobotVer_t robotVersion, bool simulationMode) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //ui->menuBar->setNativeMenuBar(false);

    _robotVersion = robotVersion;

    _motorsWin = nullptr;
    _sensorsWin = nullptr;
    _inputsWin = nullptr;

    _robotControl = new RobotControl(this, _robotVersion, simulationMode);
    if (!_robotControl->isValidRobotConfig()) {
        QMessageBox msgBox;
        msgBox.setText("Error intialising position control.\nUnknown robot configuration.\nExiting...");
        msgBox.exec();
        _setupValid = false;
        return;
    }

    _robotControl->setUseJointLimits(ui->checkBox_useJointLimits->isChecked());
    _robotControl->setUseCollisionDetection(ui->checkBox_useCollisionDetection->isChecked());

    _robotControl->setTargetTracking(false);
    QStringList controlModes;
    controlModes << "Free space" << "Proximity surface follow" << "Force surface follow" << "Force fixed point";
    ui->comboBox_controlMode->addItems(controlModes);
    ui->comboBox_controlMode->setCurrentIndex(0);
    _robotControl->setControlMode(RobotControl::FREE_CONTROL);

    connect(_robotControl, &RobotControl::robotPoseUpdated, this, &MainWindow::updateRobotPose);
    connect(_robotControl, &RobotControl::motorConnectionStatus, this, &MainWindow::updateMotorConnectionStatus);
    connect(_robotControl, &RobotControl::sensorConnectionStatus, this, &MainWindow::updateSensorConnectionStatus);
    connect(_robotControl, &RobotControl::externalInputsConnectionStatus, this, &MainWindow::updateExternalInputsConnectionStatus);
    connect(_robotControl, &RobotControl::surfaceMeshStatus, this, &MainWindow::updateSurfaceMeshStatus);

    connect(_robotControl, &RobotControl::surfaceMeshChanged, this, &MainWindow::updateWorkspaceViewer);

    if (_robotVersion == V1_ROBOT)
        ui->pushButton_connectSensors->setEnabled(false);

    initialiseWorkspaceViewer();

    _setupValid = true;
}

MainWindow::~MainWindow()
{
    disconnect(_robotControl, nullptr, this, nullptr);
    if (_motorsWin != nullptr)
        _motorsWin->close();  // Close also deletes the window, eventually
    if (_sensorsWin != nullptr)
        _sensorsWin->close();
    if (_inputsWin != nullptr)
        _inputsWin->close();

    delete _robotControl;
    delete ui;
}

//----------------------------------------------------------------------
// Check that MainWindow was setup with a valid robot configuration
//----------------------------------------------------------------------
bool MainWindow::setupOk() const
{
    return _setupValid;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Slot called when _currentPoseModel calculates a new probe and robot
// pose
//----------------------------------------------------------------------
void MainWindow::updateRobotPose()
{
    //std::cerr << "Display update" << std::endl;

    std::vector<bool> jointsInRange;
    _robotControl->isTargetPoseInRange(jointsInRange);
    std::vector<bool> linkCollisions;
    _robotControl->isTargetPoseColliding(linkCollisions);

    for (int k = 1; k < _robotControl->getNLinks(); k++) {
        if (_robotLinkCurrentActor[k] == nullptr)
            continue;
        if (jointsInRange[k-1] && !linkCollisions[k])  // *** Check the k-1 and k are correct
            _robotLinkCurrentActor[k]->GetProperty()->SetColor(0.5, 0.5, 1.0);
        else if (!jointsInRange[k-1])
            _robotLinkCurrentActor[k]->GetProperty()->SetColor(1.0, 0.5, 0.5);
        else // Collision
            _robotLinkCurrentActor[k]->GetProperty()->SetColor(1.0, 1.0, 0.5);
    }

    updateWorkspaceViewer();
}

//----------------------------------------------------------------------
// Slots called to indicate motor and force connection states
//----------------------------------------------------------------------
void MainWindow::updateMotorConnectionStatus(QString status)
{
    ui->label_MotorsStatus->setText(status);
}

void MainWindow::updateSensorConnectionStatus(QString status)
{
    ui->label_SensorStatus->setText(status);
}

void MainWindow::updateExternalInputsConnectionStatus(QString status)
{
    ui->label_ExternalInputsStatus->setText(status);
}

void MainWindow::updateSurfaceMeshStatus(QString status)
{
    ui->label_surfaceStatus->setText(status);
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Display functions for VTK workspace viewer
//----------------------------------------------------------------------
void MainWindow::initialiseWorkspaceViewer()
{
    // Current probe position
    _probeCurrentMapper.resize(_robotControl->getNProbes(), nullptr);
    _probeCurrentActor.resize(_robotControl->getNProbes(), nullptr);

    for (int p = 0; p < _robotControl->getNProbes(); p++) {
        if (_robotControl->getProbeCurrentPort(p) == nullptr)
            continue;

        _probeCurrentMapper[p] = vtkSmartPointer<vtkPolyDataMapper>::New();
        _probeCurrentMapper[p]->SetInputConnection(_robotControl->getProbeCurrentPort(p));

        _probeCurrentActor[p] = vtkSmartPointer<vtkActor>::New();
        _probeCurrentActor[p]->SetMapper(_probeCurrentMapper[p]);
        _probeCurrentActor[p]->GetProperty()->SetColor(1.0, 1.0, 1.0);
        _probeCurrentActor[p]->GetProperty()->SetOpacity(1.0);
    }

    // Target probe position
    _probeTargetMapper.resize(_robotControl->getNProbes(), nullptr);
    _probeTargetActor.resize(_robotControl->getNProbes(), nullptr);

    for (int p = 0; p < _robotControl->getNProbes(); p++) {
        if (_robotControl->getProbeTargetPort(p) == nullptr)
            continue;

        _probeTargetMapper[p] = vtkSmartPointer<vtkPolyDataMapper>::New();
        _probeTargetMapper[p]->SetInputConnection(_robotControl->getProbeTargetPort(p));

        _probeTargetActor[p] = vtkSmartPointer<vtkActor>::New();
        _probeTargetActor[p]->SetMapper(_probeTargetMapper[p]);
        _probeTargetActor[p]->GetProperty()->SetColor(1.0, 1.0, 1.0);
        _probeTargetActor[p]->GetProperty()->SetOpacity(0.5);
    }

    // Robot links
    _robotLinkCurrentMapper.resize(_robotControl->getNLinks(), nullptr);
    _robotLinkCurrentActor.resize(_robotControl->getNLinks(), nullptr);

    for (int k = 0; k < _robotControl->getNLinks(); k++) {
        if (_robotControl->getRobotLinkCurrentPort(k) == nullptr)
            continue;

        _robotLinkCurrentMapper[k] = vtkSmartPointer<vtkPolyDataMapper>::New();
        _robotLinkCurrentMapper[k]->SetInputConnection(_robotControl->getRobotLinkCurrentPort(k));

        _robotLinkCurrentActor[k] = vtkSmartPointer<vtkActor>::New();
        _robotLinkCurrentActor[k]->SetMapper(_robotLinkCurrentMapper[k]);
        _robotLinkCurrentActor[k]->GetProperty()->SetColor(0.5, 0.5, 1.0);
        _robotLinkCurrentActor[k]->GetProperty()->SetOpacity(1.0);
    }

    // Surface, representing the abdomen
    _surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    _surfaceMapper->SetInputConnection(_robotControl->getSurfacePort());

    _surfaceActor = vtkSmartPointer<vtkActor>::New();
    _surfaceActor->SetMapper(_surfaceMapper);
    _surfaceActor->GetProperty()->SetColor(0.7, 0.5, 0.4);
    _surfaceActor->GetProperty()->SetOpacity(1.0);

    QString robotId;
    switch (_robotVersion) {
    case V1_ROBOT:
        robotId = "1.0";
        break;
    case V2_ROBOT:
        robotId = "2.2";
        break;
    case V3P3_ROBOT:
        robotId = "3.3";
        break;
    case V3P4_ROBOT:
        robotId = "3.4";
        break;
    default:
        break;
    }

    // Bed plane - location is in xml file
    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl;
    std::vector<double> bedSize, bedPosition;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", robotId);
    xmlParseSuccess &= XmlUtils::readValFromElement(robotEl, "bedSize", bedSize);
    xmlParseSuccess &= bedSize.size() == 3;
    xmlParseSuccess &= XmlUtils::readValFromElement(robotEl, "bedPosition", bedPosition);
    xmlParseSuccess &= bedPosition.size() == 3;

    if (xmlParseSuccess) {
        _bedPlane = vtkSmartPointer<vtkCubeSource>::New();
        _bedPlane->SetCenter(bedPosition[0], bedPosition[1], bedPosition[2] - bedSize[2]/2.0);
        _bedPlane->SetXLength(bedSize[0]);
        _bedPlane->SetYLength(bedSize[1]);
        _bedPlane->SetZLength(bedSize[2]);

        _bedPlaneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        _bedPlaneMapper->SetInputConnection(_bedPlane->GetOutputPort());

        _bedPlaneActor = vtkSmartPointer<vtkActor>::New();
        _bedPlaneActor->SetMapper(_bedPlaneMapper);
        _bedPlaneActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
        _bedPlaneActor->GetProperty()->SetOpacity(1.0);
    }

    // Renderer
    _workspaceRenderer = vtkSmartPointer<vtkRenderer>::New();
    for (int i = 0; i < _robotControl->getNProbes(); i++) {
        if (_probeCurrentActor[i] != nullptr)
            _workspaceRenderer->AddActor(_probeCurrentActor[i]);
        if (_probeTargetActor[i] != nullptr)
            _workspaceRenderer->AddActor(_probeTargetActor[i]);
    }
    _workspaceRenderer->AddActor(_surfaceActor);
    if (xmlParseSuccess)
        _workspaceRenderer->AddActor(_bedPlaneActor);
    for (int i = 0; i < _robotControl->getNLinks(); i++)
        if (_robotLinkCurrentActor[i] != nullptr)
            _workspaceRenderer->AddActor(_robotLinkCurrentActor[i]);

    _workspaceRenderer->GetActiveCamera()->Roll(180.0);
    _workspaceRenderer->ResetCamera();

    // The correct type of render window is now created automatically by QVTKOpenGLNativeWidget, so it doesn't need to be set manually
    //_workspaceRenderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    //ui->vtkOpenGLNativeWidget_workspaceViewer->setRenderWindow(_workspaceRenderWindow);

    ui->vtkOpenGLNativeWidget_workspaceViewer->renderWindow()->AddRenderer(_workspaceRenderer);

    // This is the default and doesn't need to be set
    //vtkRenderWindowInteractor* renderWindowInteractor = ui->vtkOpenGLNativeWidget_workspaceViewer->renderWindow()->GetInteractor();
    //_interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    //renderWindowInteractor->SetInteractorStyle(_interactorStyle);
    //_interactorStyle->SetDefaultRenderer(_workspaceRenderer);

    // *** There is currently an issue with Qt6 where the mouse being inside the viewer window is treated as a click when using
    // the mac trackpad, so that the view rotates without actually clicking. It seems to be since Qt6 and only on mac when using
    // the trackpad in a QVTKOpenGLNativeWidget. The fix would be to use a mouse or wait for Qt/VTK to correct the problem (first
    // appeared in 2023).
}

void MainWindow::updateWorkspaceViewer()
{
    //_workspaceRenderer->ResetCamera();
    ui->vtkOpenGLNativeWidget_workspaceViewer->renderWindow()->Render();

#ifdef KINEMATICS_SCREENSHOTS
    static int count = 0;
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(_workspaceRenderWindow);
    windowToImageFilter->SetScale(1);
    windowToImageFilter->SetInputBufferTypeToRGBA();
    windowToImageFilter->ReadFrontBufferOff();
    windowToImageFilter->Update();

    std::ostringstream oss;
    oss << "screenshots/screenshot_" << count << ".png";
    count++;

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(oss.str().c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
    std::cerr << "Saving image " << oss.str() << std::endl;
#endif
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Private GUI slots
//----------------------------------------------------------------------
void MainWindow::on_actionFileExit_triggered()
{
    this->close();
}

void MainWindow::on_actionControlMotors_triggered()
{
    if (_robotVersion != UNKNOWN_ROBOT) {
        if (_motorsWin == nullptr) {
            _motorsWin = new MotorsWindow(this, _robotControl->getMotorControlPtr(_robotVersion));
            _motorsWin->show();
        }
    }
}

void MainWindow::on_actionControlSensors_triggered()
{
    switch (_robotVersion) {
    case V1_ROBOT:
    {
        QMessageBox msgBox;
        msgBox.setText("Sensors monitoring not implemented for Robot v1.0.");
        msgBox.exec();
        break;
    }

    case V2_ROBOT:
    case V3P3_ROBOT:
    case V3P4_ROBOT:
    {
        if (_sensorsWin == nullptr) {
            _sensorsWin = new SensorsWindow(this, _robotControl->getSensorControlPtr(_robotVersion));
            _sensorsWin->show();
        }
        break;
    }

    default:
        break;
    }
}

void MainWindow::on_actionControlInputs_triggered()
{
    if (_inputsWin == nullptr) {
        _inputsWin = new InputsWindow(this, _robotControl);
        _inputsWin->show();
    }
}

void MainWindow::on_actionDevMeasureWorkspace_triggered()
{
    _robotControl->runThroughWorkspace();
}

void MainWindow::on_pushButton_connectMotors_clicked()
{
    _robotControl->connectMotors();
}

void MainWindow::on_pushButton_connectSensors_clicked()
{
    _robotControl->connectSensors();
}

void MainWindow::on_pushButton_connectExternalInputs_clicked()
{
    _robotControl->connectExternalInputs();
}

void MainWindow::on_pushButton_loadSurface_clicked()
{
    //QString filename = QFileDialog::getOpenFileName(this, tr("Open Surface Map"), "./Resources/SurfaceMaps", tr("VTK surface mesh files (*.vtp)"));
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Surface Map"), "./Resources/SurfaceMaps", tr("VTK surface mesh files (*.vtp)"), nullptr, QFileDialog::DontUseNativeDialog);
    // *** Temporary fix, because native file dialog is not appearing in macos with Qt 6.9.1.

    _robotControl->loadSurfaceMap(filename);
}

void MainWindow::on_checkBox_useJointLimits_stateChanged(int arg1)
{
    if (arg1 == 0)
        _robotControl->setUseJointLimits(false);
    else
        _robotControl->setUseJointLimits(true);

    updateRobotPose();  // Updates colours of links
}

void MainWindow::on_checkBox_useCollisionDetection_stateChanged(int arg1)
{
    if (arg1 == 0)
        _robotControl->setUseCollisionDetection(false);
    else
        _robotControl->setUseCollisionDetection(true);

    updateRobotPose();  // Updates colours of links
}

void MainWindow::on_checkBox_targetTracking_stateChanged(int arg1)
{
    if (arg1 == 0) {
        _robotControl->setTargetTracking(false);
        ui->comboBox_controlMode->setEnabled(false);
    }
    else {
        _robotControl->setTargetTracking(true);
        ui->comboBox_controlMode->setEnabled(true);
    }
}

void MainWindow::on_comboBox_controlMode_currentIndexChanged(int index)
{
    RobotControl::ControlMode_t controlMode;

    switch (index) {
    case 0:
        controlMode = RobotControl::FREE_CONTROL;
        break;

    case 1:
        controlMode = RobotControl::SURFACE_FOLLOW_PROXIMITY_CONTROL;
        break;

    case 2:
        controlMode = RobotControl::SURFACE_FOLLOW_FORCE_CONTROL;
        break;

    case 3:
        controlMode = RobotControl::FIXED_POINT_FORCE_CONTROL;
        break;

    default:
        controlMode = RobotControl::UNKNOWN_CONTROL;
        break;
    }

    _robotControl->setControlMode(controlMode);
}
