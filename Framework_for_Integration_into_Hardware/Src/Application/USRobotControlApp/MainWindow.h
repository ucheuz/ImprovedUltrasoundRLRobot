#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointer>

#include "MotorsWindow.h"
#include "SensorsWindow.h"
#include "InputsWindow.h"

#include "RobotControl/RobotControl.h"

#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
//#include <vtkGenericOpenGLRenderWindow.h>
//#include <vtkInteractorStyleTrackballCamera.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr, RobotVer_t robotVersion = UNKNOWN_ROBOT, bool simulationMode = true);
    ~MainWindow();

    bool setupOk() const;

public slots:
    void updateRobotPose();
    void updateMotorConnectionStatus(QString status);
    void updateSensorConnectionStatus(QString status);
    void updateExternalInputsConnectionStatus(QString status);
    void updateSurfaceMeshStatus(QString status);

private:
    Ui::MainWindow *ui;

    RobotVer_t _robotVersion;
    bool _setupValid;

    QPointer<MotorsWindow> _motorsWin;

    QPointer<SensorsWindow> _sensorsWin;

    QPointer<InputsWindow> _inputsWin;

    RobotControl* _robotControl;

    // Functions for 3D viewer
    void initialiseWorkspaceViewer();
    void updateWorkspaceViewer();

    // vtk rendering objects for 3D viewer
    // Current probe position
    std::vector<vtkSmartPointer<vtkPolyDataMapper> > _probeCurrentMapper;
    std::vector<vtkSmartPointer<vtkActor> > _probeCurrentActor;
    // Target probe position
    std::vector<vtkSmartPointer<vtkPolyDataMapper> > _probeTargetMapper;
    std::vector<vtkSmartPointer<vtkActor> > _probeTargetActor;
    // Current robot position
    std::vector<vtkSmartPointer<vtkPolyDataMapper> > _robotLinkCurrentMapper;
    std::vector<vtkSmartPointer<vtkActor> > _robotLinkCurrentActor;
    // Surface
    vtkSmartPointer<vtkPolyDataMapper> _surfaceMapper;
    vtkSmartPointer<vtkActor> _surfaceActor;
    // Bed plane
    vtkSmartPointer<vtkCubeSource> _bedPlane;
    vtkSmartPointer<vtkPolyDataMapper> _bedPlaneMapper;
    vtkSmartPointer<vtkActor> _bedPlaneActor;
    // Renderer
    vtkSmartPointer<vtkRenderer> _workspaceRenderer;
    // Render window
    //vtkSmartPointer<vtkGenericOpenGLRenderWindow> _workspaceRenderWindow;  // The correct type of render window is created automatically in QVTKOpenGLNativeWidget, so this variable is not needed.
    // Interactor Style
    //vtkSmartPointer<vtkInteractorStyleTrackballCamera> _interactorStyle;  // This is the default and doesn't need to be set

private slots:
    // Menu items
    void on_actionFileExit_triggered();
    void on_actionControlMotors_triggered();
    void on_actionControlSensors_triggered();
    void on_actionControlInputs_triggered();
    void on_actionDevMeasureWorkspace_triggered();

    // GUI controls
    void on_pushButton_connectMotors_clicked();
    void on_pushButton_connectSensors_clicked();
    void on_pushButton_connectExternalInputs_clicked();
    void on_pushButton_loadSurface_clicked();

    void on_checkBox_useJointLimits_stateChanged(int arg1);
    void on_checkBox_useCollisionDetection_stateChanged(int arg1);
    void on_checkBox_targetTracking_stateChanged(int arg1);
    void on_comboBox_controlMode_currentIndexChanged(int index);
};

#endif // MAINWINDOW_H
