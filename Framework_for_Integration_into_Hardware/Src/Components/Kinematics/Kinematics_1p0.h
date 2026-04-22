#ifndef KINEMATICS_1P0_H
#define KINEMATICS_1P0_H

#include "IKinematics.h"

#include "DHKinematics.h"  // From DHKinematics library

//======================================================================
// Kinematics_1p0 class implements the forward and inverse kinematics
// functions for the IKinematics interface. This implementation is
// specific to the iFIND version 1.0 robot.
//======================================================================
class Kinematics_1p0 : public IKinematics
{
    //Q_OBJECT
    Q_INTERFACES(IKinematics)

public:
    explicit Kinematics_1p0();
    ~Kinematics_1p0();

private:
    DHKinematics _DHModel;  // For the end effector only
    int _nJoints_endEffector;
    std::vector<bool> _linearJoint;
    int _nJoints_base;
    TRTrans3D _baseAdjust;

    // --- Functions implemented from IKinematics ---
    // Kinematics functions
    bool setupModel();
    void calcForwardKinematics(bool recalcProbePoses);
    void calcInverseKinematics(bool recalcProbePoses);
};

#endif // KINEMATICS_1P0_H
