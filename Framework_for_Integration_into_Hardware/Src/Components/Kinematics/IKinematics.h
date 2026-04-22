#ifndef IKINEMATICS_H
#define IKINEMATICS_H

#include <vector>
#include <string>

#include <QObject>
#include <QtPlugin>
#include <QString>
#include <QTextStream>

#include "TRTrans3D.h"  // From TransMatrix library

//======================================================================
// IKinematics is a base class for various kinematic models. The class
// stores the robot position as both joint positions and the robot tip
// pose. Specific implementations of the forward and inverse kinematics
// to convert between the two should be provided in derived classes.
// Note that all public functions work in degrees.

// Currently available derived classes are:
//   Kinematics_1p0 for robot v1.0
//   Kinematics_2p2 for robot v2.2 (Single arm)
//   Kinematics_3p3 for robot V3.3 (Two arm side mounted)
//   Kinematics_3p4 for robot V3.4 (Two arm end mounted)
//======================================================================
class IKinematics : public QObject
{
    Q_OBJECT

public:
    IKinematics();
    virtual ~IKinematics();

    bool isValidModel() const;

    int getNProbes() const;
    int getNJoints() const;

    // Set neutral joint positions
    //void setJointNeutral(unsigned int idx, double val);
    //void setJointNeutral(const std::vector<double>& neutralJointPos);

    // Get/set joint positions
    void setJointPosition(int idx, double position, bool recalcProbePoses);
    void setJointPositions(const std::vector<double>& jointPositions, bool recalcProbePoses);
    double getJointPosition(int idx) const;
    void getJointPositions(std::vector<double>& jointPositions) const;
    void getJointNeutralPositions(std::vector<double>& jointPositions) const;

    // Get/set probe pose
    void setProbePose(unsigned int idx, const TRTrans3D& pose, const Point3D& surfaceNormal, bool recalcPose);
    void setProbePoses(const std::vector<TRTrans3D>& poses, const std::vector<Point3D>& surfaceNormals, bool recalcPose);
    void getProbePose(unsigned int idx, TRTrans3D& pose) const;
    void getProbePoses(std::vector<TRTrans3D>& poses) const;
    void getProbeNeutralPoses(std::vector<TRTrans3D>& neutralPoses) const;
    void getLinkPoses(std::vector<TRTrans3D>& linkPoses) const;

    bool isPoseInRange(std::vector<bool>& jointsOk) const;

    // Development function for measuring workspace
    void runThroughWorkspace();

signals:
    void kinematicsPoseUpdated();
    void kinematicsJointsUpdated();

protected:
    void initialise(QString robotId);

//private:
    bool _validModel;

    int _nJoints;
    int _nProbes;

    // Neutral positions of joints in degrees and mm
    std::vector<double> _jointNeutral;

    // Current positions of joints in degrees and mm
    std::vector<double> _jointPositions;

    // Limits on _jointPositions for each joint
    std::vector<double> _jointMin;
    std::vector<double> _jointMax;

    // Whether joints are independent or determined by the previous joint
    std::vector<bool> _dependentJoint;

    // Probe and robot pose as transformation matrices
    std::vector<TRTrans3D> _neutralPose;
    std::vector<TRTrans3D> _T_probeToBase;
    std::vector<TRTrans3D> _linkPoses;

    // Surface normal at current position
    std::vector<Point3D> _surfaceNormal;

private:
    void recursiveRunThroughJoint(int jointIdx, const int lastJoint, const int nSteps, const std::vector<double>& stepSize, std::vector<double>& jointPos, QTextStream& fout);

    // Kinematics functions, to be implemented in derived class for specific robots
    virtual bool setupModel() = 0;
    virtual void calcForwardKinematics(bool recalcProbePoses = true) = 0;
    virtual void calcInverseKinematics(bool recalcProbePoses) = 0;
};

Q_DECLARE_INTERFACE(IKinematics, "Interface.IKinematics")

#endif // IKINEMATICS_H
