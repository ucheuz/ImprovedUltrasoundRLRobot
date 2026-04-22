#ifndef INPUTS_H
#define INPUTS_H

#include <QObject>

#include "USRobotControlConfig.h"

#include "JoystickInput.h"
#ifdef USE_INPUT_IGTL
  #include "IGTLInput.h"
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
  #include "SpaceMouseInput.h"
#endif  // USE_INPUT_SPACEMOUSE
#include "SurfacePointsInput.h"
#include "ControlPanelInput.h"
#include "RLInput.h"
#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// ExternalInputs class provides a single interface to multiple input
// sources and a function to select between them. It is also possible to
// send information back to some inputs, e.g. sensor data or current
// pose.
// Currently available inputs are:
//   Joystick
//   IGTL (optional)
//   SpaceMouse (optional)
//   Target surface points from external software
//   External control panel
//======================================================================
class ExternalInputs : public QObject
{
    Q_OBJECT
public:
    explicit ExternalInputs(QObject *parent = nullptr);
    ~ExternalInputs();

    enum Input_t {NONE=0, JOYSTICK, IGTL, SPACE_MOUSE, SURF_POINT, CONTROL_PANEL, RL_INPUT, UNKNOWN};

    void connectInputs();
    int nConnected() const;

    void selectInput(Input_t input);

    //void getCurrentTarget(TRTrans3D targetPose);

    void setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses);
    void setCentrePose(const TRTrans3D& centrePose);

    void setCurrentPoses(const std::vector<TRTrans3D>& probePoses);
    void setCurrentForce(const std::vector<double>& forceData);
    void setCurrentDistances(const std::vector<double>& distData);
    void setCurrentJointPositions(const std::vector<double>& jointPositions);
    void setRLPort(quint16 port) { _rlPort = port; }

    // Input access for GUI
    JoystickInput* getJoystickInputPtr() const;
#ifdef USE_INPUT_IGTL
    IGTLInput* getIGTLInputPtr() const;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    SpaceMouseInput* getSpaceMouseInputPtr() const;
#endif  // USE_INPUT_SPACEMOUSE
    SurfacePointsInput* getSurfPointsInputPtr() const;
    ControlPanelInput* getControlPanelInputPtr() const;
    RLInput* getRLInputPtr() const;

signals:
    void singleProbePoseRequest(int idx, const TRTrans3D& pose);
    void singleProbeRelativePoseRequest(int idx, const TRTrans3D& relativePose);
    void multiProbePoseRequest(const std::vector<TRTrans3D>& poses);
    void multiProbeRelativePoseRequest(const std::vector<TRTrans3D>& relativePoses);
    void poseAdjustRequest(const TRTrans3D& pose, bool isLocal);
    void connectionStatus(QString status);

    void setActiveProbe(int idx, bool isActive);
    void cancel();
    void jointAnglesFromRL(const std::vector<double>& jointAngles);


private:
    JoystickInput* _joystickInput;
#ifdef USE_INPUT_IGTL
    IGTLInput* _igtlInput;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    SpaceMouseInput* _spaceMouseInput;
#endif  // USE_INPUT_SPACEMOUSE
    SurfacePointsInput* _surfPointsInput;
    ControlPanelInput* _controlPanelInput;
    RLInput* _rlInput;
    Input_t _currentInput;
    quint16 _rlPort = 12345;
};

#endif // INPUTS_H
