#include <iostream>
#include <cmath>
#include <vector>

#include "Kinematics_1p0.h"

#include <QDomDocument>
#include <QString>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

// *** Needs updating for standard coordinate system

static const double pi = std::atan2(1.0, 1.0) * 4.0;

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
Kinematics_1p0::Kinematics_1p0()
{
    initialise("1.0");
}

Kinematics_1p0::~Kinematics_1p0()
{
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Kinematics functions
// Uses the DHKinematics library, which multiples many matrices. It is
// more efficient, but harder to code and debug, to write out direct
// equations for the whole model.
// The forward and inverse kinematics use the Denavit-Hartenberg model
// for the end effector only, because the rest is a simple three axis
// translation.
//----------------------------------------------------------------------
bool Kinematics_1p0::setupModel()
{
    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl, kinematicsEl, dhParametersEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", "1.0");
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Kinematics", kinematicsEl);
    xmlParseSuccess &= XmlUtils::getElementFromElement(kinematicsEl, "DHParameters", dhParametersEl, "section", "endEffector");

    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "nSectionJoints", _nJoints_endEffector);
    _nJoints_base = _nJoints - _nJoints_endEffector;

    std::vector<double> alpha;
    std::vector<double> a;
    std::vector<double> d;
    std::vector<double> theta;

    // Denavit-Hartenberg parameters for each joint in the end effector.
    // Needs angles in degrees, distances in mm.
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "alphaAngle", alpha);
    xmlParseSuccess &= alpha.size() == _nJoints_endEffector;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "aDist", a);
    xmlParseSuccess &= a.size() == _nJoints_endEffector;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "dDist", d);
    xmlParseSuccess &= d.size() == _nJoints_endEffector;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "thetaAngle", theta);
    xmlParseSuccess &= theta.size() == _nJoints_endEffector;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "linearJoint", _linearJoint);
    xmlParseSuccess &= _linearJoint.size() == _nJoints_endEffector;

    std::vector<double> params;
    xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "baseAdjustParams", params);
    xmlParseSuccess &= params.size() == 6;
    _baseAdjust.setParams(params[0], params[1], params[2], params[3], params[4], params[5]);

    if (!xmlParseSuccess) {
        std::cerr << "Error in Kinematics_1p0::setupModel: failed to read kinematics parameters from xml file." << std::endl;
        return false;
    }

    if (_nJoints != 8) {
        std::cerr << "Error in Kinematics_1p0::setupModel: inverse kinenmatics expects 8 joints." << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < _nJoints_endEffector; i++) {
        if (_linearJoint[i])
            d[i] = _jointPositions[i+_nJoints_base];
        else
            theta[i] = _jointPositions[i+_nJoints_base];
    }

    // Denavit-Hartenberg model of the last five axes of the US robot (end effector)
    _DHModel.clear();

    // Parameter for each joint: radius is 'a' and offset is 'd'
    for (unsigned int i = 0; i < _nJoints_endEffector; i++)
        _DHModel.addJoint(theta[i], d[i], a[i], alpha[i], _linearJoint[i]);

    return true;
}

void Kinematics_1p0::calcForwardKinematics(bool recalcProbePoses)
{
    // DH model part
    for (unsigned int i = 0; i < _nJoints_endEffector; i++) {
        if (_linearJoint[i])
            _DHModel.setPosition(i, _jointPositions[i+_nJoints_base]);
        else
            _DHModel.setAngle(i, _jointPositions[i+_nJoints_base]);
    }

    // Translation part, including a coordinate rotation to the standard base coordinates:
    // X right to left, Y head to feet, Z floor to ceiling
    TRTrans3D T_EEToBase(_baseAdjust);  // Sets the rotation
    Point3D translation;
    translation.X(_jointPositions[2]);  // d3
    translation.Y(-_jointPositions[0]);  // d1
    translation.Z(_jointPositions[1]);  // d2
    T_EEToBase.setTranslation(translation);

    _DHModel.setBasePose(T_EEToBase);

    std::vector<TRTrans3D> linkPosesEE;
    linkPosesEE.resize(_nJoints_endEffector);

    _DHModel.getAllJointPoses(linkPosesEE);
    for (unsigned int i = 0; i < _nJoints_endEffector; i++)
        _linkPoses[i+_nJoints_base] = linkPosesEE[i];

    if (recalcProbePoses)
        _T_probeToBase[0] = _linkPoses.back();

    emit kinematicsPoseUpdated();
}

void Kinematics_1p0::calcInverseKinematics(bool recalcProbePoses)
{
    if (!_validModel) {
        std::cerr << "Error in Kinematics_1p0::calcInverseKinematics: invalid model setup for Robot v1.0." << std::endl;
        return;
    }

    double t5, t6, t8;
    double radToDeg = 180.0 / pi;

    // Rotation matrix for joints 5 to 8 of end effector, assuming fixed rotation up to Joint 4
    TRTrans3D R = ~_DHModel.getJointPose(1) * _T_probeToBase[0];

    // Decompose rotation matrix
    // Decompose R into three angles for joints t5, t6 and t8
    double R00 = R(0,0);  // -cos(t5) * cos(t6) * sin(t8) - sin(t5) * cos(t8)
    double R01 = R(0,1);  // cos(t5) * sin(t6)
    double R02 = R(0,2);  // cos(t5) * cos(t6) * cos(t8) - sin(t5) * sin(t8)
    double R10 = R(1,0);  // -sin(t5) * cos(t6) * sin(t8) + cos(t5) * cos(t8)
    double R11 = R(1,1);  // sin(t5) * sin(t6)
    double R12 = R(1,2);  // sin(t5) * cos(t6) * cos(t8) + cos(t5) * sin(t8)
    double R20 = R(2,0);  // sin(t6) * sin(t8)
    double R21 = R(2,1);  // cos(t6)
    double R22 = R(2,2);  // -sin(t6) * cos(t8)

    // t6 angle
    double s6 = sqrt((R01 * R01 + R11 * R11 + R20 * R20 + R22 * R22) / 2.0);  // sin(t6)
    double t6_a = atan2(s6, R21);
    double t6_b = atan2(-s6, R21);
    double t6Rad = (fabs(t6_a - pi/2.0) < fabs(t6_b - pi/2.0)) ? t6_a : t6_b;  // Use the solution nearest to 90 deg (the neutral position). *** Is this always when s6 is +ve?

    // t5 and t8 angles
    if (fabs(fabs(R21) - 1.0) < std::numeric_limits<double>::epsilon()) {
        // t6 == 0 or 180 deg; sin(t6) == 0, cos(t6) == +/-1
        double ttRad = atan2(-R00 + R21 * R12, R10 + R21 * R02);  // tan(tt) = t5 - c9 * t8
        t8 = _jointPositions[7];  // Use previous value, so it doesn't change suddenly
        t6  = t6Rad * radToDeg;
        t5  = ttRad * radToDeg + R21 * t8;
    }
    else {
        // sin(t6) != 0
        s6 = sin(t6Rad);
        t5  = atan2((R00 * R22 - R20 * R02) / s6, (R12 * R20 - R10 * R22) / s6) * radToDeg;
        t6  = t6Rad * radToDeg;
        t8 = atan2((R12 * R01 - R02 * R11) / s6, (R01 * R10 - R11 * R00) / s6) * radToDeg;
    }
    //std::cerr << t5 << " " << t6 << " " << t8 << std::endl;

    _jointPositions[4] = t5;
    while (_jointPositions[4] >= 180.0)
        _jointPositions[4] -= 360.0;
    while (_jointPositions[4] < -180.0)
        _jointPositions[4] += 360.0;

    _jointPositions[5] = t6;
    while (_jointPositions[5] >= 180.0)
        _jointPositions[5] -= 360.0;
    while (_jointPositions[5] < -180.0)
        _jointPositions[5] += 360.0;

    _jointPositions[7] = t8;
    while (_jointPositions[7] >= 180.0)
        _jointPositions[7] -= 360.0;
    while (_jointPositions[7] < -180.0)
        _jointPositions[7] += 360.0;

    // Leave d7 unchanged, unless it is changed directly

    // Apply partial forward model for the end effector using these joint positions
    for (unsigned int i = 0; i < _nJoints_endEffector; i++) {
        if (_linearJoint[i])
            _DHModel.setPosition(i, _jointPositions[i+_nJoints_base]);
        else
            _DHModel.setAngle(i, _jointPositions[i+_nJoints_base]);
    }

    TRTrans3D endTransform = _DHModel.getEndPose();

    // The rotation part of this matrix should match the target matrix.
    // The difference in translation is given by d1, d2 and d3.
    Point3D dhModelTranslation, targetTranslation;
    endTransform.getTranslation(dhModelTranslation);
    _T_probeToBase[0].getTranslation(targetTranslation);
    Point3D diffTranslation = targetTranslation - dhModelTranslation;

    double tX = diffTranslation.X();
    double tY = diffTranslation.Y();
    double tZ = diffTranslation.Z();

    _jointPositions[2] += tX;  // d3,  X direction
    _jointPositions[0] -= tY;  // d1, -Y direction
    _jointPositions[1] += tZ;  // d2,  Z direction

    emit kinematicsJointsUpdated();

    calcForwardKinematics(recalcProbePoses);  // To update link positions
}
