#ifndef KINEMATICS_2P2_H
#define KINEMATICS_2P2_H

#include <vector>

#include "IKinematics.h"

#include "Point3D.h"  // From TransMatrix library
#include "DHKinematics.h"  // From DHKinematics library

//======================================================================
// Kinematics_2p2 class implements the forward and inverse kinematics
// functions for the IKinematics interface. This implementation is
// specific to the iFIND version 2.2 robot.
//======================================================================
class Kinematics_2p2 : public IKinematics
{
    //Q_OBJECT
    Q_INTERFACES(IKinematics)

public:
    explicit Kinematics_2p2();
    ~Kinematics_2p2();

private:
    DHKinematics _DHModel;
    std::vector<double> _alpha;
    std::vector<double> _a;
    std::vector<double> _d;
    std::vector<double> _theta;
    std::vector<bool> _linearJoint;

    // --- Functions implemented from IKinematics ---
    // Kinematics functions
    bool setupModel();
    void calcForwardKinematics(bool recalcProbePoses);
    void calcInverseKinematics(bool recalcProbePoses);
};

#endif // KINEMATICS_2P2_H
