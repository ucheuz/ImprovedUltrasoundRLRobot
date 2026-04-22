#include <iostream>
#include <cmath>

#include "IKinematics.h"

#include "float_utils.h"  // From TransMatrix library

#include <QDomDocument>
#include <QTextStream>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

#include <QFile>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
// Parameters are initialised in initialise function called from
// constructor in derived class
//----------------------------------------------------------------------
IKinematics::IKinematics()
{
}

IKinematics::~IKinematics()
{
}

//----------------------------------------------------------------------
// Get whether the model was successfully loaded from the xml file
//----------------------------------------------------------------------
bool IKinematics::isValidModel() const
{
    return _validModel;
}

//----------------------------------------------------------------------
// Get number of probes
//----------------------------------------------------------------------
int IKinematics::getNProbes() const
{
    return _nProbes;
}

//----------------------------------------------------------------------
// Get number of joints
//----------------------------------------------------------------------
int IKinematics::getNJoints() const
{
    return _nJoints;
}

//----------------------------------------------------------------------
// Set initial joint positions
//----------------------------------------------------------------------
/*void IKinematics::setJointNeutral(unsigned int idx, double val)
{
    if (idx >= _nJoints) {
        std::cerr << "Error in IKinematics::setJointNeutral: index is out of range." << std::endl;
        return;
    }

    _jointNeutral[idx] = val;
}

void IKinematics::setJointNeutral(const std::vector<double>& neutralJointPos)
{
    if (neutralJointPos.size() != _nJoints)
        std::cerr << "Error in IKinematics::setJointNeutral: wrong array length." << std::endl;
    else {
        for (int i = 0; i < _nJoints; i++)
            _jointNeutral[i] = neutralJointPos[i];
    }
}*/

//----------------------------------------------------------------------
// Get/set joint offset positions
//----------------------------------------------------------------------
void IKinematics::setJointPosition(int idx, double position, bool recalcPose)
{
    if (idx < 0 || idx >= _nJoints) {
        std::cerr << "Error in IKinematics::setJointPosition: index is out of range." << std::endl;
        return;
    }

    if (!float_utils::equal(position, _jointPositions[idx])) {
        _jointPositions[idx] = position;
        if (recalcPose)
            calcForwardKinematics();
    }
}

void IKinematics::setJointPositions(const std::vector<double>& jointPositions, bool recalcPose)
{
    bool positionChanged = false;

    if (jointPositions.size() != _nJoints) {
        std::cerr << "Error in IKinematics::setJointPositions: wrong array length." << std::endl;
        return;
    }
    else {
        for (unsigned int i = 0; i < _nJoints; i++) {
            if (_dependentJoint[i]) {
                _jointPositions[i] = -(jointPositions[i-1] - _jointNeutral[i-1]) + _jointNeutral[i];
                //_jointPositions[i] = -jointPositions[i-1];
                continue;
            }
            if (!float_utils::equal(jointPositions[i], _jointPositions[i])) {
                positionChanged = true;
                _jointPositions[i] = jointPositions[i];
            }
        }
    }

    if (positionChanged && recalcPose)
        calcForwardKinematics();
}

double IKinematics::getJointPosition(int idx) const
{
    if (idx < 0 || idx >= _nJoints) {
        std::cerr << "Error in IKinematics::getJointPosition: index is out of range." << std::endl;
        return 0.0;
    }

    return _jointPositions[idx];
}

void IKinematics::getJointPositions(std::vector<double>& jointPositions) const
{
    jointPositions.resize(_nJoints, 0.0);
    for (unsigned int i = 0; i < _nJoints; i++)
        jointPositions[i] = _jointPositions[i];
}

void IKinematics::getJointNeutralPositions(std::vector<double>& jointPositions) const
{
    jointPositions.resize(_nJoints, 0.0);
    for (unsigned int i = 0; i < _nJoints; i++)
        jointPositions[i] = _jointNeutral[i];
}

//----------------------------------------------------------------------
// Get/set probe pose
//----------------------------------------------------------------------
void IKinematics::setProbePose(unsigned int idx, const TRTrans3D& pose, const Point3D& surfaceNormal, bool recalcProbePoses)
{
    if (idx >= _nProbes) {
        std::cerr << "Error in IKinematics::setProbePose: index is out of range." << std::endl;
        return;
    }

    _T_probeToBase[idx].set(pose);
    _surfaceNormal[idx].set(surfaceNormal);
    calcInverseKinematics(recalcProbePoses);
}

void IKinematics::setProbePoses(const std::vector<TRTrans3D>& poses, const std::vector<Point3D>& surfaceNormals, bool recalcProbePoses)
{
    if (poses.size() != _nProbes || surfaceNormals.size() != _nProbes) {
        std::cerr << "Error in IKinematics::setProbePoses: array size do not match the nubmer of probes." << std::endl;
        return;
    }

    for (unsigned int i = 0; i < _nProbes; i++) {
        _T_probeToBase[i].set(poses[i]);
        _surfaceNormal[i].set(surfaceNormals[i]);
    }
    calcInverseKinematics(recalcProbePoses);
}

void IKinematics::getProbePose(unsigned int idx, TRTrans3D& pose) const
{
    if (idx >= _nProbes) {
        std::cerr << "Error in IKinematics::getProbePose: index is out of range." << std::endl;
        return;
    }

    pose.set(_T_probeToBase[idx]);
}

void IKinematics::getProbePoses(std::vector<TRTrans3D>& poses) const
{
    poses.resize(_nProbes);
    for (unsigned int i = 0; i < _nProbes; i++)
        poses[i] = _T_probeToBase[i];
}

void IKinematics::getProbeNeutralPoses(std::vector<TRTrans3D>& neutralPoses) const
{
    neutralPoses.resize(_nProbes);
    for (unsigned int i = 0; i < _nProbes; i++)
        neutralPoses[i].set(_neutralPose[i]);
}

void IKinematics::getLinkPoses(std::vector<TRTrans3D>& linkPoses) const
{
    linkPoses.resize(_nJoints);
    for (unsigned int i = 0; i < _nJoints; i++)
        linkPoses[i].set(_linkPoses[i]);
}

//----------------------------------------------------------------------
// Check whether requested pose is within the joint limits
//----------------------------------------------------------------------
bool IKinematics::isPoseInRange(std::vector<bool>& jointsOk) const
{
    bool poseInRange = true;
    jointsOk.resize(_nJoints, true);
    for (int i = 0; i < _nJoints; i++) {
        //std::cerr << _jointPositions[i] << " ";
        jointsOk[i] = (_jointPositions[i] >= _jointMin[i]) && (_jointPositions[i] <= _jointMax[i]);
        poseInRange &= jointsOk[i];
    }
    //std::cerr << std::endl;
    return poseInRange;
}

//----------------------------------------------------------------------
// Development function to run through the whole workspace and save the
// end pose to file.
//----------------------------------------------------------------------
void IKinematics::runThroughWorkspace()
{
    QString outFilename("./Workspace.txt");
    QFile outFile(outFilename);
    bool success = outFile.open(QFile::WriteOnly | QFile::Text);
    if (!success) {
        std::cerr << "Error in IKinematics::runThroughWorkspace: unable to open file " << outFilename.toStdString() << "." << std::endl;
        return;
    }

    QTextStream fout(&outFile);

    unsigned int nSteps = 9;
    std::vector<double> jointPos(_nJoints, 0.0);
    std::vector<double> stepSize(_nJoints, 0.0);
    for (unsigned int i = 0; i < _nJoints; i++)
        stepSize[i] = (_jointMax[i] - _jointMin[i]) / (nSteps - 1.0);
    //unsigned int firstJoint = 0;
    //unsigned int lastJoint = _nJoints-2;
    unsigned int firstJoint = 3;
    unsigned int lastJoint = 4;
    while (_dependentJoint[lastJoint])
        lastJoint--;

    recursiveRunThroughJoint(firstJoint, lastJoint, nSteps, stepSize, jointPos, fout);

    outFile.close();
}

void IKinematics::recursiveRunThroughJoint(int jointIdx, const int lastJoint, const int nSteps, const std::vector<double>& stepSize, std::vector<double>& jointPos, QTextStream& fout)
{
    for (int i = 0; i < nSteps; i++) {
        if (jointIdx <= lastJoint - 3) {
            for (int j = 0; j < jointIdx; j++)
                std::cerr << "  ";
            std::cerr << jointIdx << " " << i << std::endl;
        }

        jointPos[jointIdx] = _jointMin[jointIdx] + i * stepSize[jointIdx];

        if (jointIdx < lastJoint)
            recursiveRunThroughJoint(jointIdx + 1, lastJoint, nSteps, stepSize, jointPos, fout);
        else {
            double x, y, z, rx, ry, rz;
            setJointPositions(jointPos, true);
            for (unsigned int p = 0; p < _nProbes; p++) {
                _T_probeToBase[p].getParams(x, y, z, rx, ry, rz);
                fout << x << " " << y << " " << z << " " << rx << " " << ry << " " << rz << "; ";
            }
            fout << Qt::endl;
        }
    }
}

//======================================================================
// Protected functions
//======================================================================
//----------------------------------------------------------------------
// Initialise variables from xml file for the specified robot
//----------------------------------------------------------------------
void IKinematics::initialise(QString robotId)
{
    QDomDocument doc("robotInfo");
    QDomElement robotEl, kinematicsEl;
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", robotId);
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Kinematics", kinematicsEl);

    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "nProbes", _nProbes);
    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "nJoints", _nJoints);

    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "jointNeutralPosition", _jointNeutral);
    xmlParseSuccess &= _jointNeutral.size() == _nJoints;

    // _jointOffsets limits from zero position
    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "jointOffsetMin", _jointMin);
    xmlParseSuccess &= _jointMin.size() == _nJoints;
    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "jointOffsetMax", _jointMax);
    xmlParseSuccess &= _jointMax.size() == _nJoints;

    xmlParseSuccess &= XmlUtils::readValFromElement(kinematicsEl, "dependentJoint", _dependentJoint);
    xmlParseSuccess &= (_dependentJoint.size() == _nJoints) && (_dependentJoint.size() > 0) && !_dependentJoint[0];

    if (!xmlParseSuccess) {
        std::cerr << "Error in IKinematics::initialise: failed to read kinematics parameters from xml file." << std::endl;
        _validModel = false;
        return;
    }

    _jointPositions = _jointNeutral;

    _surfaceNormal.resize(_nProbes);
    for (int i = 0; i < _nProbes; i++)
        _surfaceNormal[i].setZero();

    _T_probeToBase.resize(_nProbes);

    _linkPoses.resize(_nJoints);

    if (!setupModel()) {
        _validModel = false;
        return;
    }
    calcForwardKinematics();

    //_T_probeToBase[0].printParams();
    //_T_probeToBase[0].printMatrix();

    _neutralPose.resize(_nProbes);
    for (unsigned int i = 0; i < _nProbes; i++)
        _neutralPose[i].set(_T_probeToBase[i]);

    _validModel = true;
}
