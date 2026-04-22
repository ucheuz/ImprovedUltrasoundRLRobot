#include <iostream>
#include <cmath>

#include "Kinematics_3p4.h"

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
Kinematics_3p4::Kinematics_3p4()
{
    initialise("3.4");
}

Kinematics_3p4::~Kinematics_3p4()
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
bool Kinematics_3p4::setupModel()
{
    QStringList sections;
    sections << "leftArm" << "rightArm";

    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl, kinematicsEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", "3.4");
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Kinematics", kinematicsEl);

    for (int s = 0; s < sections.length(); s++) {
        QDomElement dhParametersEl;
        xmlParseSuccess &= XmlUtils::getElementFromElement(kinematicsEl, "DHParameters", dhParametersEl, "section", sections[s]);

        // Denavit-Hartenberg parameters for each joint. Needs angles in degrees, distances in mm.
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "nSectionJoints", _nSectionJoints[s]);
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "alphaAngle", _alpha[s]);
        xmlParseSuccess &= _alpha[s].size() == _nSectionJoints[s];
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "aDist", _a[s]);
        xmlParseSuccess &= _a[s].size() == _nSectionJoints[s];
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "dDist", _d[s]);
        xmlParseSuccess &= _d[s].size() == _nSectionJoints[s];
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "thetaAngle", _theta[s]);
        xmlParseSuccess &= _theta[s].size() == _nSectionJoints[s];
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "linearJoint", _linearJoint[s]);
        xmlParseSuccess &= _linearJoint[s].size() == _nSectionJoints[s];

        // Adjustment from DH base coordinates to world coordinates for each arm
        std::vector<double> params;
        xmlParseSuccess &= XmlUtils::readValFromElement(dhParametersEl, "baseAdjustParams", params);
        xmlParseSuccess &= params.size() == 6;
        _sectionBaseAdjust[s].setParams(params[0], params[1], params[2], params[3], params[4], params[5]);

        if (!xmlParseSuccess) {
            std::cerr << "Error in Kinematics_3p4::setupModel: failed to read kinematics parameters from xml file for section " << sections[s].toStdString() << "." << std::endl;
            return false;
        }

        if (_nSectionJoints[s] != 11) {
            std::cerr << "Error in Kinematics_3p4::setupModel: inverse kinenmatics expects 11 joints in each of 2 arms." << std::endl;
            return false;
        }

        _idx_section[s] = (s == 0) ? 0 : (_idx_section[s-1] + _nSectionJoints[s-1]);

        for (unsigned int i = 0; i < _nSectionJoints[s]; i++) {
            if (_linearJoint[s][i])
                _d[s][i] = _jointPositions[i+_idx_section[s]];
            else
                _theta[s][i] = _jointPositions[i+_idx_section[s]];
        }

        // Denavit-Hartenberg model of this section of the robot
        _DHModel[s].clear();
        // Parameter for each joint: radius is 'a' and offset is 'd'
        for (unsigned int i = 0; i < _nSectionJoints[s]; i++)
            _DHModel[s].addJoint(_theta[s][i], _d[s][i], _a[s][i], _alpha[s][i], _linearJoint[s][i]);
    }

    return true;
}

void Kinematics_3p4::calcForwardKinematics(bool recalcProbePoses)
{
    // DH model for each arm
    for (unsigned int s = 0; s < _nSections; s++) {
        for (unsigned int i = 0; i < _nSectionJoints[s]; i++) {
            if (_linearJoint[s][i])
                _DHModel[s].setPosition(i, _jointPositions[i+_idx_section[s]]);
            else
                _DHModel[s].setAngle(i, _jointPositions[i+_idx_section[s]]);
        }

        // Base pose (reorientation from DH base coordinates to standard world coordinates)
        _DHModel[s].setBasePose(_sectionBaseAdjust[s]);

        // Pose of each link
        std::vector<TRTrans3D> linkPoses;
        _DHModel[s].getAllJointPoses(linkPoses);
        for (unsigned int i = 0; i < linkPoses.size(); i++)
            _linkPoses[i + _idx_section[s]] = linkPoses[i];

        if (recalcProbePoses)
            _T_probeToBase[s] = linkPoses.back();
    }

    emit kinematicsPoseUpdated();
}

// Base coordinates, relative to patient, are X left to right, Y head to feet, Z floor to ceiling.
// Left and right robot arms refer to patient's left and right.
void Kinematics_3p4::calcInverseKinematics(bool recalcProbePoses)
{
    if (!_validModel) {
        std::cerr << "Error in Kinematics_3p4::calcInverseKinematics: invalid model setup for Robot v3.4." << std::endl;
        return;
    }

    const double radToDeg = 180.0 / pi;
    double thetaZ_L, thetaZ_R;  // Surface normal angles
    double thetaXY_L, thetaXY_R;
    std::vector<double> t_L(11, 0.0), t_R(11, 0.0);  // Final result for all joints

    // Initialise to neutral pose
    for (unsigned int i = 0; i < 11; i++) {
        t_L[i] = _jointNeutral[i];
        t_R[i] = _jointNeutral[i+11];
    }

    // Surface normals at targets
    Point3D unitSurfaceNormal_L(_surfaceNormal[0]);
    unitSurfaceNormal_L.normalise();
    Point3D unitSurfaceNormal_R(_surfaceNormal[1]);
    unitSurfaceNormal_R.normalise();
    // Angle of normal from upward vertical (+Z)
    thetaZ_L = radToDeg * std::acos(unitSurfaceNormal_L.Z());
    thetaZ_R = radToDeg * std::acos(unitSurfaceNormal_R.Z());
    // Horizontal angle of normal from +X axis - range is -180 to +180 degrees
    thetaXY_L = radToDeg * std::atan2(unitSurfaceNormal_L.Y(), unitSurfaceNormal_L.X());
    thetaXY_R = radToDeg * std::atan2(unitSurfaceNormal_R.Y(), unitSurfaceNormal_R.X());

    // Direction of end-effector in neutral pose relative to +X axis. Appropriate direction will change depending on angle of passive joints.
    //double neutralDir_L = -17.0;
    //double neutralDir_R = -163.0;
    double neutralDir_L = -45.0;
    double neutralDir_R = -135.0;

    // Detect potential arm collision states based on the target probe positions
    Point3D position[2];
    _T_probeToBase[0].getTranslation(position[0]);
    _T_probeToBase[1].getTranslation(position[1]);
    Point3D probeSeparation = position[1] - position[0];

    // Initially set arm joints 5 and 6 so that C6x faces the neutral direction for the end-effector
    t_L[4] = t_R[4] = 0.0;
    t_L[5] = -(180.0 - t_L[1] + neutralDir_L);  // 180 degrees is because C0x is opposite direction to Base X direction.
    t_R[5] = -(180.0 - t_R[1] + neutralDir_R);
    t_L[6] = t_R[6] = 0.0;

    // Both arms - Joints 7 and 8 according to surface angle
    solveSurfaceAngleTilt(unitSurfaceNormal_L, thetaZ_L, thetaXY_L, neutralDir_L, probeSeparation, true, t_L);  // Left arm
    solveSurfaceAngleTilt(unitSurfaceNormal_R, thetaZ_R, thetaXY_R, neutralDir_R, probeSeparation, false, t_R);  // Right arm

    t_L[6] = -t_L[6];  // Reverse because C6z is in opposite direction to Base Z direction.
    t_R[6] = -t_R[6];

    // Both arms - Joints 9 to 11 for remaining probe orientation
    solveTiltOrientation(0, t_L);  // Left arm
    solveTiltOrientation(1, t_R);  // Right arm

    // Both arms - Joints 3 and 4 for vertical (Z) position of probe
    solveVerticalTranslation(0, t_L);  // Left arm
    solveVerticalTranslation(1, t_R);  // Right arm

    // Both arms - Joint 1 for a gantry position near the centre of the arm ranges
    solveGantryTranslation(t_L, t_R);

    // Both arms - Joints 5 to 7 for horizontal (X,Y) position of the end effector
    solveArmHorizontalTranslation(0, t_L, neutralDir_L, true);
    solveArmHorizontalTranslation(1, t_R, neutralDir_R, false);

    // Convert to joint positions
    for (unsigned int i = 0; i < 11; i++) {
        _jointPositions[i] = t_L[i];
        _jointPositions[i+11] = t_R[i];
    }

    emit kinematicsJointsUpdated();

    calcForwardKinematics(recalcProbePoses);  // To update link positions
}

//----------------------------------------------------------------------
// Given the surface normal, find the joint angles for end effector
// joints 7 and 8 to align the end effector to the surface.
// neutralXY angle is the preferred approach angle of the end effector
// relative to the base coordinates. Left and right arms will be
// different.
//----------------------------------------------------------------------
void Kinematics_3p4::solveSurfaceAngleTilt(const Point3D& surfaceNormal, double thetaZ, double thetaXY, double neutralXY, Point3D probeSeparation, bool isLeftArm, std::vector<double>& jointPositions)
{
    double radToDeg = 180.0 / pi;

    // Joint 8 adjusts the approach angle according to the surface normal angle from vertical.
    // Steeper slope means larger angle at Joint 8 giving the ability to reach around the far side of the abdomen.
    // Assumes C6x is horizontal.
    double wT, tiltWeightPower = 0.5;  // Values <1 tilt more slowly than the surface as thetaZ increases from 0 to 90 degrees; >1 tilts faster.
    wT = std::pow(thetaZ/90.0, 1.0/tiltWeightPower);
    jointPositions[7] = (thetaZ <= 90.0) ? (wT * 90.0) : thetaZ;  // Linear scaling above 90 degrees
    jointPositions[7] += 90.0;  // Kinematics offset

    // Determine whether probe separation indicates a possible collision of the arms
    double probeSeparationDist = probeSeparation.norm();
    double probeSeparationAngle = fabs(radToDeg * std::atan2(probeSeparation.Y(), probeSeparation.X()) - 90.0);

    double collisionAdjustInnerDist = 100.0;  // *** Not very optimised thresholds
    double collisionAdjustOuterDist = 150.0;
    double collisionAdjustOuterAngle = 60.0;
    double collisionAdjustInnerAngle = 45.0;
    double collisionAdjustWeightPower = 1.0;
    // wC is weight between full collision avoidance and normal position.
    // wC_dist increases from 0 to 1 as separation decreases from outerDist to innerDist.
    double wC_dist = 1.0 - (probeSeparationDist - collisionAdjustInnerDist) / (collisionAdjustOuterDist - collisionAdjustInnerDist);
    wC_dist = (wC_dist >= 0.0) ? wC_dist : 0.0;
    wC_dist = (wC_dist <= 1.0) ? wC_dist : 1.0;
    // wC_angle increases from 0 to 1 as angle varies from outer to inner angle.
    double wC_angle = 1.0 - (probeSeparationAngle - collisionAdjustInnerAngle) / (collisionAdjustOuterAngle - collisionAdjustInnerAngle);
    wC_angle = (wC_angle >= 0.0) ? wC_angle : 0.0;
    wC_angle = (wC_angle <= 1.0) ? wC_angle : 1.0;
    double wC = std::pow(wC_dist * wC_angle, collisionAdjustWeightPower);
    wC = 0.0;  // *** Temporarily disabled collision avoidance

    // Side condition weight
    double centreToSideWeightPower = 5.0;  // Higher values move to the side condition sooner
    double wZ = std::pow(surfaceNormal.Z(), centreToSideWeightPower);

    // Joint 7 is neutralXY angle at the peak (thetaZ = 0), and depends on the XY angle on the side (thetaZ = 90).
    // In between it is a weighted sum between the two cases.
    // Initially set assuming C6x is in the neutral direction. To be adjusted later.
    double relativeThetaXY = thetaXY - neutralXY;
    if (!isLeftArm) {
        // Reverse the XY angle direction for arm aproaching from the right side.
        // This means positive angles are towards the feet, as for the left arm.
        relativeThetaXY = -relativeThetaXY;
    }
    while (relativeThetaXY < -180.0)
        relativeThetaXY += 360.0;
    while (relativeThetaXY > 180.0)
        relativeThetaXY -= 360.0;
    if (relativeThetaXY >= -45.0 && relativeThetaXY <= 45.0)
        // Side condition is end effector pointing down surface, according to thetaXY.
        // In full collision adjust mode, it is instead pointing in the neutral direction.
        jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * relativeThetaXY;
    else if (relativeThetaXY >= -135.0 && relativeThetaXY < -45.0)
        // Side condition is end effector pointing down surface at -45 degrees.
        // In full collision adjust mode it is ??? degrees.
        // Angle is a weighted sum of these two cases, depending on wC.
        //jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * (relativeThetaXY - -90.0);
        jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * -45.0;
    else if (relativeThetaXY > 45.0 && relativeThetaXY <= 135.0) {
        // Side condition is as above, at +45 degrees.
        //jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * (relativeThetaXY - 90.0);
        //if (isLeftArm)
            //std::cerr << "Condition C" << std::endl;
        jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * 45.0;
    }
    else {
        // Side condition is normally a weighted sum between the above cases and 0 degrees along the 180 degree line.
        // In full collision adjust mode it is ??? degrees. Angle is a weighted sum of these two cases.
        double wXY = std::fabs(180.0 - fabs(relativeThetaXY)) / 45.0;  // How far from 180 degree line
        if (relativeThetaXY <= -135.0)
            jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * ((1.0 - wXY) * 0.0 + wXY * -45.0);
        else
            jointPositions[6] = (1.0 - wC) * (1.0 - wZ) * ((1.0 - wXY) * 0.0 + wXY * 45.0);
    }

    // If Joint 8 + thetaZ is close to 90(+90) degrees and Joint 7 is close to relativeThetaXY-180, this causes too large a back tilt on Joint 10.
    // Instead, Joint 7 should be shifted to get across this region.
    // The direction of adjustment depends on whether the other arm also needs to be avoided.
    // This case happens on the 180 degree line, and also in collision adjust mode near the -90 degree line.
    /*double backTiltAdjustZThreshold = 60.0;  // Ideal max back tilt is -90(-90) degrees, but actual limit is almost 45 degrees before this
    double backTiltAdjustXYThreshold = 45.0;
    if (fabs(jointPositions[7] + thetaZ - 180) < backTiltAdjustZThreshold && fabs(fabs(jointPositions[6] - relativeThetaXY) - 180.0) < backTiltAdjustXYThreshold) {
        double w1 = fabs(jointPositions[7] + thetaZ - 180) / backTiltAdjustZThreshold;
        double w2 = fabs(fabs(jointPositions[6] - relativeThetaXY) - 180.0) / backTiltAdjustXYThreshold;
        w1 = std::pow(w1, 2.0);
        w2 = std::pow(w2, 2.0);
        std::cerr << "Back tilt adjust: w1 = " << (1.0 - w1) << ", w2 = " << (1.0 - w2);
        if (wC > 0.0) {
            jointPositions[6] += 60.0 * (1.0 - w1) * (1.0 - w2);
            std::cerr << "; " << 60.0 * (1.0 - w1) * (1.0 - w2) << " degs." << std::endl;
        }
        else {
            jointPositions[6] -= 60.0 * (1.0 - w1) * (1.0 - w2);
            std::cerr << "; " << -60.0 * (1.0 - w1) * (1.0 - w2) << " degs." << std::endl;
        }
    }*/

    if (!isLeftArm)
        jointPositions[6] *= -1;
}

//----------------------------------------------------------------------
// Given positions for end effector joints 7 and 8, find the orientation
// required in the last three joints 9 to 11.
//----------------------------------------------------------------------
void Kinematics_3p4::solveTiltOrientation(unsigned int sectionIdx, std::vector<double>& jointPositions)
{
    if (sectionIdx >= _nSections)
        return;

    double t9, t10, t11;  // Angles of three end joints, to be found
    const double radToDeg = 180.0 / pi;

    unsigned int sectionOffset = sectionIdx * 11;

    // Assume all joints before the end effectors are in zero position, and joints 6 and 7 are already set.
    for (unsigned int i = 0; i < 8; i++) {
        if (_linearJoint[sectionIdx][i])
            _DHModel[sectionIdx].setPosition(i, jointPositions[i]);
        else
            _DHModel[sectionIdx].setAngle(i, jointPositions[i]);
    }

    // Remaining orientation, relative to coordinates C8 (last three joints). This is the tilt relative to the surface.
    TRTrans3D T_C8 = _DHModel[sectionIdx].getJointPose(8);
    TRTrans3D tiltOrientation = (~T_C8) * _T_probeToBase[sectionIdx];

    // Decompose tiltOrientation into three angles for joints 9-11
    double R00 = tiltOrientation(0,0);  // cos(t9) * cos(t10) * cos(t11) + sin(t9) * sin(t11)
    double R01 = tiltOrientation(0,1);  // -cos(t9) * sin(t10)
    double R02 = tiltOrientation(0,2);  // cos(t9) * cos(t10) * sin(t11) - sin(t9) * cos(t11)
    double R10 = tiltOrientation(1,0);  // sin(t9) * cos(t10) * cos(t11) - cos(t9) * sin(t11)
    double R11 = tiltOrientation(1,1);  // -sin(t9) * sin(t10)
    double R12 = tiltOrientation(1,2);  // sin(t9) * cos(t10) * sin(t11) + cos(t9) * cos(t11)
    double R20 = tiltOrientation(2,0);  // -sin(t10) * cos(t11)
    double R21 = tiltOrientation(2,1);  // -cos(t10)
    double R22 = tiltOrientation(2,2);  // -sin(t10) * sin(t11)

    // Joint 10 angle
    double s10 = sqrt((R01 * R01 + R11 * R11 + R20 * R20 + R22 * R22) / 2.0);  // sin(t10)
    double t10_a = atan2(s10, -R21);
    double t10_b = atan2(-s10, -R21);
    double t10Rad = (fabs(t10_a + pi/2.0) < fabs(t10_b + pi/2.0)) ? t10_a : t10_b;  // Use the solution nearest to -90 deg (the neutral position). *** Is this always when s10 is -ve?

    // Joints 9 and 11 angles
    double c10 = cos(t10Rad);
    if (fabs(fabs(c10) - 1.0) < std::numeric_limits<double>::epsilon()) {
        // t10 == 0 or 180 deg; sin(t10) == 0, cos(t10) == +/-1
        double ttRad = atan2(-R02 - R21 * R10, R12 - R21 * R00);  // tan(tt) = t9 - c10 * t11
        t11 = _jointPositions[sectionOffset+10];;  // unchanged so it doesn't change suddenly
        t10 = t10Rad * radToDeg;
        t9  = ttRad * radToDeg + c10 * t11;
    }
    else {
        // sin(t10) != 0
        s10 = sin(t10Rad);
        t9  = atan2((R02 * R20 - R00 * R22) / s10, (R10 * R22 - R12 * R20) / s10) * radToDeg;
        t10 = t10Rad * radToDeg;
        t11 = atan2((R10 * R01 - R00 * R11) / s10, (R02 * R11 - R12 * R01) / s10) * radToDeg;
    }

    jointPositions[8]  = t9;
    jointPositions[9]  = t10;
    jointPositions[10] = t11;
    //std::cerr << t9 << " " << t10 << " " << t11 << std::endl;

    // Prevent last joint from rotating multiple cycles
    while (jointPositions[10] < _jointMin[sectionOffset+10])
        jointPositions[10] += 360.0;
    while (jointPositions[10] > _jointMax[sectionOffset+10])
        jointPositions[10] -= 360.0;

    for (unsigned int i = 8; i < 11; i++)
        _DHModel[sectionIdx].setAngle(i, jointPositions[i]);
}

//----------------------------------------------------------------------
// Given positions for end effector joints 7 to 11, find the poses of
// arm joint 3 and dependent joint 4 that give the correct height of the
// probe
//----------------------------------------------------------------------
void Kinematics_3p4::solveVerticalTranslation(unsigned int sectionIdx, std::vector<double>& jointPositions)
{
    if (sectionIdx >= _nSections)
        return;

    const double radToDeg = 180.0 / pi;
    //unsigned int sectionOffset = sectionIdx * 11;

    // Height of Joint 6 when Joints 3 and 4 are level
    _DHModel[sectionIdx].setAngle(2, 0.0);
    _DHModel[sectionIdx].setAngle(3, 0.0);
    TRTrans3D T_arm_levelPose = _DHModel[sectionIdx].getJointPose(6);

    // Required position of Link 6 with end effector now fixed
    TRTrans3D T_EE = ~T_arm_levelPose * _DHModel[sectionIdx].getEndPose();
    TRTrans3D T_arm_target = _T_probeToBase[sectionIdx] * (~T_EE);

    // Height difference to be provided by Joint 3 and dependent Joint 4
    double zDiff = T_arm_target(2,3) - T_arm_levelPose(2,3);

    // Angle from level to give this height difference
    double t3 = radToDeg * asin(zDiff / _a[sectionIdx][2]);  // +ve angle is upward from level
    jointPositions[2] =  t3;
    jointPositions[3] = -t3;

    _DHModel[sectionIdx].setAngle(2, jointPositions[2]);
    _DHModel[sectionIdx].setAngle(3, jointPositions[3]);
}

//----------------------------------------------------------------------
// Given positions for end effector joints 7 to 11, and arm joints 3 and
// 4, find a central position for the gantry joint 1
//----------------------------------------------------------------------
void Kinematics_3p4::solveGantryTranslation(std::vector<double>& jointPositions_L, std::vector<double>& jointPositions_R)
{
    // Maximum y (head-feet) position of targets, compared to current
    // position in zero pose of joints 1, 5 and 6 for the same probe
    double yTarget, yZero;
    /*if (_T_probeToBase[0](1,3) > _T_probeToBase[1](1,3)) {
        yTarget = _T_probeToBase[0](1,3);
        yZero = _DHModel[0].getEndPose()(1,3);
    }
    else {
        yTarget = _T_probeToBase[1](1,3);
        yZero = _DHModel[1].getEndPose()(1,3);
    }*/
    yTarget = 0.5 * (_T_probeToBase[0](1,3) + _T_probeToBase[1](1,3));
    yZero = 0.5 * (_DHModel[0].getEndPose()(1,3) + _DHModel[1].getEndPose()(1,3));

    double yNeutral = 0.5 * (_neutralPose[0].getTranslation().Y() + _neutralPose[1].getTranslation().Y());
    //jointPositions_L[0] = jointPositions_R[0] = -(yTarget - yZero);  // +ve joint movement is -Y direction
    jointPositions_L[0] = jointPositions_R[0] = -(yTarget - yNeutral) - 0.0;  // +ve joint movement is -Y direction
    jointPositions_L[0] += _jointNeutral[0];
    jointPositions_R[0] += _jointNeutral[11];

    _DHModel[0].setPosition(0, jointPositions_L[0]);
    _DHModel[1].setPosition(0, jointPositions_R[0]);
}

//----------------------------------------------------------------------
// Given positions for all joints except 5 and 6, solve the final
// two-bar positioning problem to give the correct horizontal (XY)
// position of the probe

// Complex equations for two-bar calculation:
// z is independent variable - the target position
// r1 and r2 are fixed
// t1 and t2 are variable
// r1*exp(i*t1) + r2*exp(i*t2) = (x, y) = z                  (1)
// x = r1*cos(t1) + r2*cos(t2)                               (2a)
// y = r1*sin(t1) + r2*sin(t2)                               (2b)
// r2^2 = (x - r1*cos(t1))^2 + (y - r1*sin(t1))^2            (3)
// r2^2 = x^2 + y^2 + r1^2 - 2*r1*(x*cos(t1) + y*sin(t1))    (4)
// x*cos(t1) + y*sin(t1) = C                                 (5)
// R*cos(t1 - a) = C                                         (6) - a standard identity
// R = sqrt(x^2 + y^2)                                       (7a)
// a = atan2(y, x)                                           (7b)
// Solve (6) for t1, using the result of (7)
// Then solve for t2 in (2)
//----------------------------------------------------------------------
void Kinematics_3p4::solveArmHorizontalTranslation(unsigned int sectionIdx, std::vector<double>& jointPositions, double neutralDir, bool isLeftArm)
{
    if (sectionIdx >= _nSections)
        return;

    const double radToDeg = 180.0 / pi;
    const double degToRad = pi / 180.0;
    //unsigned int sectionOffset = sectionIdx * 11;

    // Required position of Link 6 with end effector now fixed
    TRTrans3D T_EE = ~(_DHModel[sectionIdx].getJointPose(6)) * _DHModel[sectionIdx].getEndPose();
    TRTrans3D T_arm = _T_probeToBase[sectionIdx] * (~T_EE);
    // Position of Link 6 relative to Link 4, to be set by Joints 5 and 6
    T_arm = ~(_DHModel[sectionIdx].getJointPose(4)) * T_arm;

    // Two bar calculation
    double x = T_arm(0,3);  // X and Y coordinates of C6 relative to C4
    double y = T_arm(1,3);
    double r1 = _a[sectionIdx][4];
    double r2 = _a[sectionIdx][5];
    double C = -(r2*r2 - r1*r1 - x*x - y*y) / (2.0 * r1);
    double R = sqrt(x*x + y*y);
    double p = atan2(y, x);
    double t5;
    if (isLeftArm)
        t5 = radToDeg * (acos(C / R) + p);  // Prefer angle further to the outward side
    else
        t5 = radToDeg * (-acos(C / R) + p);  // Prefer angle further to the outward side
    if (fabs(C / R) > 1.0) {
        std::cerr << "No solution for two-bar calculation" << std::endl;
        t5 = -180.0;  // No solution possible, so set to out of range angle.
    }
    jointPositions[4] = t5;

    double ct5 = (x - r1 * cos(t5 * degToRad)) / r2;
    double st5 = (y - r1 * sin(t5 * degToRad)) / r2;
    double t6 = radToDeg * atan2(st5, ct5);
    t6 -= t5;
    jointPositions[5] = t6;

    // Adjust for new positions of Joints 5 and 6
    // Previously assumed Joint 5 was 0 degrees and Joint 6 was angled so that C6x was in the neutral direction for the end-effector
    jointPositions[6] -= (t5 + t6 + 180.0 - jointPositions[1] + neutralDir);
}
