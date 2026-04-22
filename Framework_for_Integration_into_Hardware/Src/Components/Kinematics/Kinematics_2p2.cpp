#include <iostream>
#include <cmath>

#include "Kinematics_2p2.h"

#include <QDomDocument>
#include <QString>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

static const double pi = std::atan2(1.0, 1.0) * 4.0;

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
Kinematics_2p2::Kinematics_2p2()
{
    initialise("2.2");
}

Kinematics_2p2::~Kinematics_2p2()
{
}

//======================================================================
// Private functions - implemented from IKinematics
//======================================================================
//----------------------------------------------------------------------
// Kinematics functions
// Uses the DHKinematics library, which multiplies many matrices. It is
// more efficient, but harder to code and debug, to write out direct
// equations for the whole model.
//----------------------------------------------------------------------
bool Kinematics_2p2::setupModel()
{
    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl, kinematicsEl, dhParametersEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", "2.2");
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Kinematics", kinematicsEl);
    xmlParseSuccess &= XmlUtils::getElementFromElement(kinematicsEl, "DHParameters", dhParametersEl, "section", "all");


    int nSectionJoints;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "nSectionJoints", nSectionJoints);
    xmlParseSuccess &= nSectionJoints == _nJoints;

    // Denavit-Hartenberg parameters for each joint. Needs angles in degrees, distances in mm.
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "alphaAngle", _alpha);
    xmlParseSuccess &= _alpha.size() == nSectionJoints;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "aDist", _a);
    xmlParseSuccess &= _a.size() == nSectionJoints;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "dDist", _d);
    xmlParseSuccess &= _d.size() == nSectionJoints;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "linearJoint", _linearJoint);
    xmlParseSuccess &= _linearJoint.size() == nSectionJoints;

    if (!xmlParseSuccess) {
        std::cerr << "Error in Kinematics_2p2::setupModel: failed to read kinematics parameters from xml file." << std::endl;
        return false;
    }

    if (_nJoints != 10) {
        std::cerr << "Error in Kinematics_2p2::setupModel: inverse kinenmatics expects 10 joints." << std::endl;
        return false;
    }

    std::vector<double> theta(nSectionJoints);
    for (unsigned int i = 0; i < theta.size(); i++)
        theta[i] = _jointPositions[i];

    // Denavit-Hartenberg model of the last four axes of the US robot
    _DHModel.clear();

    // Parameter for each joint: radius is 'a' and offset is 'd'
    for (unsigned int i = 0; i < nSectionJoints; i++)
        _DHModel.addJoint(theta[i], _d[i], _a[i], _alpha[i], _linearJoint[i]);

    return true;
}

void Kinematics_2p2::calcForwardKinematics(bool recalcProbePoses)
{
    // Necessary for 8 joint version.
    // Joints 3 and 5 are constrained by joints 2 and 4 through parallel mechanism
    _jointPositions[2] = -(_jointPositions[1] - _jointNeutral[1]) + _jointNeutral[2];
    _jointPositions[4] = -(_jointPositions[3] - _jointNeutral[3]) + _jointNeutral[4];

    // DH model
    for (unsigned int i = 0; i < getNJoints(); i++)
        _DHModel.setAngle(i, _jointPositions[i]);

    _DHModel.getAllJointPoses(_linkPoses);
    if (recalcProbePoses)
        _T_probeToBase[0] = _linkPoses.back();

    emit kinematicsPoseUpdated();
}

// Base coordinates, relative to patient, are X left to right, Y head to feet, Z floor to ceiling.
void Kinematics_2p2::calcInverseKinematics(bool recalcProbePoses)
{
    if (!_validModel) {
        std::cerr << "Error in Kinematics_2p2::calcInverseKinematics: invalid model setup for Robot v2.2." << std::endl;
        return;
    }

    double radToDeg = 180.0 / pi;
    double degToRad = pi / 180.0;
    double thetaY, thetaZ, t8, t9, t10;

    // Surface normal: Joints 6 and 7 align the probe to the surface normal when in a neutral tilt.
    // This assumes Joint 5 is level and t1 is zero (correct for t1 later).
    Point3D unitSurfaceNormal(_surfaceNormal[0]);
    unitSurfaceNormal.normalise();
    thetaY = radToDeg * acos(unitSurfaceNormal.Z()) + 90.0;  // Angle of C7_Z from upward vertical, including initial offset
    double prevThetaZ = -(_jointPositions[5] - _jointPositions[0]);
    //std::cerr << prevThetaZ << " " << _jointOffsets[5] << " " << _jointNeutral[5] << " " << _jointOffsets[0] << std::endl;
    bool prevUpState = prevThetaZ > -180.0 && prevThetaZ < 0.0;
    thetaZ = radToDeg * atan2(unitSurfaceNormal.Y(), unitSurfaceNormal.X());  // Target thetaZ
    // Adjust range for joint limits
    while (thetaZ > 90.0)
        thetaZ -= 360.0;
    while (thetaZ <= -270.0)
        thetaZ += 360.0;
    //std::cerr << thetaZ << std::endl;
    // thetaZ depends on how far from centre the probe is.
    // A weighted sum between centre angle (up or down) and angled with normal (X,Y), becoming more dependent on normal further from the peak.
    double w = unitSurfaceNormal.Z() * unitSurfaceNormal.Z();
    if (unitSurfaceNormal.Y() < -0.1 || (prevUpState && unitSurfaceNormal.Y() <= 0.1)) {
        thetaZ = (1.0 - w) * thetaZ + w * -90.0;
    }
    else { // if (unitSurfaceNormal.Y() > 0.1 || (!prevUpState && unitSurfaceNormal.Y() >= -0.1))
        thetaZ = (1.0 - w) * thetaZ + w * 90.0;
        if (thetaZ < -90)
            thetaZ = (1.0 - w) * thetaZ + w * -270.0;
        else
            thetaZ = (1.0 - w) * thetaZ + w * 90.0;
    }
    // Adjust for range again
    while (thetaZ > 90.0)
        thetaZ -= 360.0;
    while (thetaZ <= -270.0)
        thetaZ += 360.0;
    //std::cerr << unitSurfaceNormal.Y() << " " << prevUpState << " " << thetaZ << std::endl;

    _jointPositions[6] = thetaY;  // t7
    _jointPositions[5] = -thetaZ;  // t6. Axis direction is -Z
    //std::cerr << thetaY << " " << thetaZ << std::endl;

    // Assume Link 5 is level, and all joints before the end effector are in zero position.
    for (unsigned int i = 0; i < 5; i++)
        _DHModel.setAngle(i, _jointNeutral[i]);  // t1 to t5 zero
    _DHModel.setAngle(5, _jointPositions[5]);  // t6
    _DHModel.setAngle(6, _jointPositions[6]);  // t7
    // Remaining orientation, relative to coordinates C7 (last three joints). This is the tilt relative to the surface.
    TRTrans3D T_C7 = _DHModel.getJointPose(7);
    TRTrans3D tiltOrientation = (~T_C7) * _T_probeToBase[0];
    //std::cerr << "Tilt: ";
    //tiltOrientation.printMatrix();

    // Decompose tiltOrientation into three angles for joints t8, t9 and t10
    double R00 = tiltOrientation(0,0);  // cos(t8) * cos(t9) * cos(t10) + sin(t8) * sin(t10)
    double R01 = tiltOrientation(0,1);  // -cos(t8) * sin(t9)
    double R02 = tiltOrientation(0,2);  // cos(t8) * cos(t9) * sin(t10) - sin(t8) * cos(t10)
    double R10 = tiltOrientation(1,0);  // sin(t8) * cos(t9) * cos(t10) - cos(t8) * sin(t10)
    double R11 = tiltOrientation(1,1);  // -sin(t8) * sin(t9)
    double R12 = tiltOrientation(1,2);  // sin(t8) * cos(t9) * sin(t10) + cos(t8) * cos(t10)
    double R20 = tiltOrientation(2,0);  // -sin(t9) * cos(t10)
    double R21 = tiltOrientation(2,1);  // -cos(t9)
    double R22 = tiltOrientation(2,2);  // -sin(t9) * sin(t10)

    // t9 angle
    double s9 = sqrt((R01 * R01 + R11 * R11 + R20 * R20 + R22 * R22) / 2.0);  // sin(t9)
    double t9_a = atan2(s9, -R21);
    double t9_b = atan2(-s9, -R21);
    double t9Rad = (fabs(t9_a + pi/2.0) < fabs(t9_b + pi/2.0)) ? t9_a : t9_b;  // Use the solution nearest to -90 deg (the neutral position). *** Is this always when s9 is -ve?

    // t8 and t10 angles
    if (fabs(fabs(R21) - 1.0) < std::numeric_limits<double>::epsilon()) {
        // t9 == 0 or 180 deg; sin(t9) == 0, cos(t9) == +/-1
        double ttRad = atan2(-R02 - R21 * R10, R12 - R21 * R00);  // tan(tt) = t8 - c9 * t10
        t10 = _jointPositions[9];  // Use previous value, so it doesn't change suddenly
        t9  = t9Rad * radToDeg;
        t8  = ttRad * radToDeg - R21 * t10;
    }
    else {
        // sin(t9) != 0
        s9 = sin(t9Rad);
        t8  = atan2((R02 * R20 - R00 * R22) / s9, (R10 * R22 - R12 * R20) / s9) * radToDeg;
        t9  = t9Rad * radToDeg;
        t10 = atan2((R10 * R01 - R00 * R11) / s9, (R02 * R11 - R12 * R01) / s9) * radToDeg;
    }

    _jointPositions[7] = t8;
    _jointPositions[8] = t9;
    _jointPositions[9] = t10;
    //std::cerr << t8 << " " << t9 << " " << t10 << std::endl;
    while (_jointPositions[9] < _jointMin[9])
        _jointPositions[9] += 360.0;
    while (_jointPositions[9] > _jointMax[9])
        _jointPositions[9] -= 360.0;

    // Required position of Link 5 with end effector now fixed
    for (unsigned int i = 7; i < 10; i++)
        _DHModel.setAngle(i, _jointPositions[i]);
    TRTrans3D T_EE = ~(_DHModel.getJointPose(5)) * _DHModel.getEndPose();
    TRTrans3D T_arm = _T_probeToBase[0] * (~T_EE);
    // Cylindrical coordinates: angle in X-Y base plane, to be provided by Joint 1
    double t1 = atan2(T_arm(1,3), T_arm(0,3));  // atan(Y/X)
    _jointPositions[0] = t1 * radToDeg;
    // Correct Joint 6 for Joint 1 angle - note that Z5 is in the opposite direction to Z0
    _jointPositions[5] += _jointPositions[0];
    // Adjust for range again
    while (_jointPositions[5] > _jointMax[5])
        _jointPositions[5] -= 360.0;
    while (_jointPositions[5] < _jointMin[5])
        _jointPositions[5] += 360.0;

    // Cylindrical coordinates: radius and height, to be provided by Joints 2-5
    double armRadius = sqrt(T_arm(0,3) * T_arm(0,3) + T_arm(1,3) * T_arm(1,3));
    double armHeight = T_arm(2,3);
    // Links 1, 3 and 5 are always level, so a fixed offset from the target
    armHeight -= _d[0] + _a[2] * sin(-(_jointPositions[1] + _jointPositions[2]) * degToRad);
    armRadius -= _a[0] + _a[2] * cos(-(_jointPositions[1] + _jointPositions[2]) * degToRad) + _a[4];
    // Links 2 and 4 are a two-bar mechanism providing the offset

    // Two bar calculation
    double C = -(_a[3]*_a[3] - _a[1]*_a[1] - armHeight*armHeight - armRadius*armRadius) / (2.0 * _a[1]);
    double R = sqrt(armRadius*armRadius + armHeight*armHeight);
    double p = atan2(armHeight, armRadius);
    double t2 = -radToDeg * (acos(C / R) + p);
    if (fabs(C / R) > 1.0)
        t2 = -180.0;  // No solution possible, so set to out of range angle.
    while (t2 >= 90.0)
        t2 -= 360.0;
    while (t2 < -270.0)
        t2 += 360.0;
    _jointPositions[1] = t2;
    double t3 = -(_jointPositions[1] - _jointNeutral[1]) + _jointNeutral[2];
    //double t3 = -_jointPositions[1];
    _jointPositions[2] = t3;

    double ct4 = (armRadius - _a[1] * cos(-t2 * degToRad)) / _a[3];
    double st4 = (armHeight - _a[1] * sin(-t2 * degToRad)) / _a[3];
    double t4 = -radToDeg * atan2(st4, ct4);
    t4 -= t2 + t3;
    _jointPositions[3] = t4;

    double t5 = -t2 - t3 - t4;
    _jointPositions[4] = t5;

    //for (int i = 0; i < 10; i++)
    //    std::cerr << _jointPositions[i] << " ";
    //std::cerr << std::endl;

    //_T_probeToBase.printMatrix();

    emit kinematicsJointsUpdated();

    calcForwardKinematics(recalcProbePoses);  // To update link positions
}
