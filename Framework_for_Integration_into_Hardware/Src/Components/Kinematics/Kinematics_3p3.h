#ifndef KINEMATICS_3P3_H
#define KINEMATICS_3P3_H

#include <vector>

#include "IKinematics.h"

#include "Point3D.h"  // From TransMatrix library
#include "DHKinematics.h"  // From DHKinematics library

//======================================================================
// Kinematics_3p3 class implements the forward and inverse kinematics
// functions for the IKinematics interface. This implementation is
// specific to the iFIND version 3.3 robot.
//======================================================================
class Kinematics_3p3 : public IKinematics
{
    //Q_OBJECT
    Q_INTERFACES(IKinematics)

public:
    explicit Kinematics_3p3();
    ~Kinematics_3p3();

private:
    static const unsigned int _nSections = 2;
    unsigned int _idx_section[_nSections];

    DHKinematics _DHModel[_nSections];

    unsigned int _nSectionJoints[_nSections];
    std::vector<double> _alpha[_nSections];
    std::vector<double> _a[_nSections];
    std::vector<double> _d[_nSections];
    std::vector<double> _theta[_nSections];
    std::vector<bool> _linearJoint[_nSections];
    TRTrans3D _sectionBaseAdjust[_nSections];

    // --- Functions implemented from IKinematics ---
    // Kinematics functions
    bool setupModel();
    void calcForwardKinematics(bool recalcProbePoses);
    void calcInverseKinematics(bool recalcProbePoses);

    // Other inverse kinematics functions
    void solveSurfaceAngleTilt(const Point3D& surfaceNormal, double thetaZ, double thetaXY, double neutralXY, Point3D probeSeparation, std::vector<double>& jointPositions);
    void solveTiltOrientation(unsigned int sectionIdx, std::vector<double>& jointPositions);
    void solveVerticalTranslation(unsigned int sectionIdx, std::vector<double>& jointPositions);
    void solveGantryTranslation(std::vector<double>& jointPositions_L, std::vector<double>& jointPositions_R);
    void solveArmHorizontalTranslation(unsigned int sectionIdx, std::vector<double>& jointPositions, double neutralDir, bool leftArm);
};

#endif // KINEMATICS_3P3_H
