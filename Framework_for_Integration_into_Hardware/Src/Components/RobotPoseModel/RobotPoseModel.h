#ifndef ROBOTPOSEMODEL_H
#define ROBOTPOSEMODEL_H

#include <vector>

#include <QObject>

#include "LinkData.h"
#include "GJKCollisionDetection.h"
#include "Kinematics/IKinematics.h"

#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkIntersectionPolyDataFilter.h>

enum RobotVer_t {V1_ROBOT=0, V2_ROBOT, V3P3_ROBOT, V3P4_ROBOT, UNKNOWN_ROBOT};

//======================================================================
// RobotPoseModel class stores the robot link and probe positions and
// has a function to check for intersections between the links and other
// vtkPolyData objects. It also contains the kinematics calculation
// classes.
//======================================================================
class RobotPoseModel : public QObject
{
    Q_OBJECT

public:
    RobotPoseModel(RobotVer_t robotVersion);
    ~RobotPoseModel();

    // Initialise with LinkData
    void setLinkData(LinkData* pLinkData);

    void removeCollisionCheck(vtkAlgorithmOutput* const collisionCheckObject);
    void addCollisionCheck(vtkAlgorithmOutput* const collisionCheckObject);

    // Access functions
    int getNProbes() const;
    int getNJoints() const;
    int getNLinks() const;

    void setCollisionDetection(bool detectCollisions);

    // Kinematics access
    void setJointPositions(const std::vector<double>& jointPositions, bool recalcPose = true);
    void getJointPositions(std::vector<double>& jointPositions) const;
    void getJointNeutralPositions(std::vector<double>& jointPositions) const;

    void setProbePoses(const std::vector<TRTrans3D>& poses, const std::vector<Point3D>& surfaceNormals, bool recalcPose = true);
    void getProbePoses(std::vector<TRTrans3D>& poses) const;
    void getProbeNeutralPoses(std::vector<TRTrans3D>& neutralPoses) const;

    bool isPoseInRange() const;
    bool isPoseInRange(std::vector<bool>& jointsOk) const;

    // Check for collisions
    bool isCollision() const;
    bool isCollision(std::vector<bool>& linkCollisions) const;

    // Display access
    vtkAlgorithmOutput* getProbePort(int idx);
    vtkAlgorithmOutput* getRobotLinkPort(int idx);

    // Development function for testing workspace
    void runThroughWorkspace();

signals:
    void robotPoseUpdated();

private:    
    RobotVer_t _robotVersion;

    LinkData* _pLinkData;

    IKinematics* _kinematics;

    // Position of objects
    std::vector<vtkSmartPointer<vtkTransform> > _probeTransform;
    std::vector<vtkSmartPointer<vtkTransformFilter> > _probeTransformFilter;
    std::vector<vtkSmartPointer<vtkTransformFilter> > _probeCollisionBoxTransformFilter;
    std::vector<vtkSmartPointer<vtkTransform> > _robotLinkTransform;
    std::vector<vtkSmartPointer<vtkTransformFilter> > _robotLinkTransformFilter;
    std::vector<vtkSmartPointer<vtkTransformFilter> > _robotLinkCollisionBoxTransformFilter;

    // List of objects with which to check for collisions
    std::vector<vtkAlgorithmOutput*> _collisionCheckObjects;
    std::vector<std::vector<vtkSmartPointer<vtkIntersectionPolyDataFilter> > > _collisionCheckIntersectionFilter;
    std::vector<GJKCollisionDetection*> _linkCollisionChecks;
    bool _detectCollisions;
    std::vector<bool> _linkCollisionStatus;  // true = collision, false = ok
    bool _collisionDetected;

    // Update internal collision status
    void updateCollisionStatus();

private slots:
    void updateRobotPose();
};

#endif // ROBOTPOSEMODEL_H
