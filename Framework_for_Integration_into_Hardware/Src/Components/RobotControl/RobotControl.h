#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QPointer>
#include <QString>

#include <vector>

#include "USRobotControlConfig.h"

#include "MotorControl/IMotorControl.h"
#include "SensorControl/ISensorControl.h"
#include "RobotPoseModel/RobotPoseModel.h"
#include "ScanSpace/ScanSpace.h"
#include "ExternalInputs/ExternalInputs.h"

//======================================================================
// RobotControl class is the master controller for the whole system
// (motors, sensors, inputs). It maintains an updating record of the
// current robot and probe pose based on motor readings and the robot
// model. Target poses are requested by various sources (inputs),
// combined with sensor information in the robot model, passed through
// some software safety algorithms, the target pose is adjusted, and
// appropriate signals are then sent to the motors.
//
// Some of this information is accessible by the GUI.
//======================================================================
class RobotControl : public QObject
{
    Q_OBJECT

public:
    explicit RobotControl(QObject *parent = nullptr, RobotVer_t robotVersion = UNKNOWN_ROBOT, bool simulationMode = true);
    ~RobotControl();

    bool isValidRobotConfig() const;

    void connectMotors();
    void connectSensors();
    void connectExternalInputs();
    void loadSurfaceMap(const QString& filename);

    // Automatic control modes
    enum ControlMode_t {FREE_CONTROL=0, SURFACE_FOLLOW_PROXIMITY_CONTROL,
                        SURFACE_FOLLOW_FORCE_CONTROL, FIXED_POINT_FORCE_CONTROL, UNKNOWN_CONTROL};

    // Direct access for GUI controls
    IMotorControl* getMotorControlPtr(RobotVer_t robotVersion);
    ISensorControl* getSensorControlPtr(RobotVer_t robotVersion);
    ControlPanelInput* getControlPanelInputPtr() const;
    JoystickInput* getJoystickInputPtr() const;
#ifdef USE_INPUT_IGTL
    IGTLInput* getIGTLInputPtr() const;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    SpaceMouseInput* getSpaceMouseInputPtr() const;
#endif  // USE_INPUT_SPACEMOUSE
    SurfacePointsInput* getSurfPointsInputPtr() const;
    RLInput* getRLInputPtr() const;

    // Display access
    int getNLinks() const;
    int getNProbes() const;
    vtkAlgorithmOutput* getProbeCurrentPort(int idx);
    vtkAlgorithmOutput* getProbeTargetPort(int idx);
    vtkAlgorithmOutput* getRobotLinkCurrentPort(int idx);
    vtkAlgorithmOutput* getSurfacePort();

    // Apply settings
    void setUseJointLimits(bool useJointLimits);
    void setUseCollisionDetection(bool useCollisionDetection);
    void setControlMode(ControlMode_t controlMode);
    void setTargetTracking(bool targetTracking);

    bool isTargetPoseInRange(std::vector<bool>& jointsOk) const;
    bool isTargetPoseColliding(std::vector<bool>& LinksOk) const;

    bool isIdle() const;

    // Select the input source
    void selectInput(ExternalInputs::Input_t input);
    // Set which probes to move using external input source
    void clearActiveProbes();
    void setActiveProbe(int idx, bool isActive);

    // Direct movement functions - mainly for kinematics test
    void goToNeutral();
    void alignToSurface();  // Position probe in contact and normal to the surface
    void planAlignToSurface(const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void rotateX(double angle);  // Elevational tilt
    void planRotateX(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void rotateY(double angle);  // Axial rotation
    void planRotateY(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void rotateZ(double angle);  // Lateral tilt
    void planRotateZ(double angle, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void surfaceFollowX(double distance);  // Lateral movement
    void planSurfaceFollowX(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void surfaceFollowZ(double distance);  // Elevational movement
    void planSurfaceFollowZ(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void moveProbeY(double distance);  // Axial movement
    void planMoveProbeY(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void moveX(double distance);  // X movement in base coordinates
    void planMoveX(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void moveY(double distance);  // Y movement in base coordinates
    void planMoveY(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void moveZ(double distance);  // Z movement in base coordinates
    void planMoveZ(double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);

    void getCurrentProbePoses(std::vector<TRTrans3D>& currentProbePoses);
    void runSequence(std::vector<std::vector<TRTrans3D> >& sequence);

    void runThroughWorkspace();  // Development function

public slots:
    void setCancelSequence();

    void setSingleProbeTarget(int idx, const TRTrans3D& pose);
    void setSingleProbeRelativeTarget(int idx, const TRTrans3D& relativePose);
    void setMultiProbeTarget(const std::vector<TRTrans3D>& poses);
    void setMultiProbeRelativeTarget(const std::vector<TRTrans3D>& relativePoses);
    void setPoseOffsetTarget(const TRTrans3D& poseAdjust, bool isLocal);
    
    //This slot receives raw joint angles from rl model and sends them
    //directly to motors, skip inverse kinematics
    void setJointAnglesFromRL(const std::vector<double>& jointAngles);

    void cancelMove();

signals:
    void targetTrackingStopProcess();
    void robotPoseUpdated();
    void motorConnectionStatus(QString status);
    void sensorConnectionStatus(QString status);
    void externalInputsConnectionStatus(QString status);
    void surfaceMeshStatus(QString status);

    void surfaceMeshChanged();

private:
    QTimer* _currentPositionUpdateTimer;
    QTimer* _targetTrackingTimer;

    RobotVer_t _robotVersion;
    QString _robotId;
    bool _validRobotConfig;

    ControlMode_t _controlMode;
    bool _targetTracking;

    //QPointer<QThread> _targetTrackingThread;
    //QPointer<TargetTracking> _targetTracking;

    LinkData* _linkData;
    RobotPoseModel* _currentPoseModel;
    RobotPoseModel* _targetPoseModel;

    IMotorControl* _motorControl;
    ISensorControl* _sensorControl;

    ScanSpace* _scanSpace;

    ExternalInputs* _externalInputs;

    int _nActiveProbes;
    std::vector<bool> _activeProbes;
    std::vector<TRTrans3D> _requestedPose;

    std::vector<int> _jointToPort;
    std::vector<int> _portToJoint;

    bool _useJointLimits;
    bool _useCollisionDetection;

    bool _movementPending;

    bool _cancelSequence;

    int firstActiveProbe() const;

    void surfaceFollow(const Point3D& direction, double distance);  // Surface following movement in probe coordinates
    void planSurfaceFollow(const Point3D& direction, double distance, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void baseMove(const TRTrans3D& move);  // Movement in base coordinates
    void planBaseMove(const TRTrans3D& move, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void tilt(const TRTrans3D& rotation);  // Tilt about current position
    void planTilt(const TRTrans3D& rotation, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);
    void rotate(const TRTrans3D& rotation);  // Surface following rotation
    void planRotate(const TRTrans3D& rotation, const std::vector<TRTrans3D>& startingProbePoses, std::vector<TRTrans3D>& newProbePoses);

    void sensorAdjust(std::vector<TRTrans3D>& pose, const std::vector<Point3D>& targetSurfacePoints, const std::vector<Point3D>& targetSurfaceNormals);

private slots:
    void updateCurrentPosition();

    // Main movement control function with error checking
    void trackTarget();
};

#endif // ROBOTCONTROL_H
