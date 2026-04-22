#include <iostream>
//#ifdef _WINDOWS
//#include "windows.h"  // For performance counter, because chrono high-def clock is not precise in Windows
//#else
//#include <chrono>
//#endif

#include "RobotPoseModel.h"
#include "Kinematics/Kinematics_1p0.h"
#include "Kinematics/Kinematics_2p2.h"
#include "Kinematics/Kinematics_3p3.h"
#include "Kinematics/Kinematics_3p4.h"

#include <vtkXMLPolyDataReader.h>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
RobotPoseModel::RobotPoseModel(RobotVer_t robotVersion)
{
    _pLinkData = nullptr;

    switch (robotVersion) {

    case V1_ROBOT:
        _robotVersion = V1_ROBOT;
        _kinematics = new Kinematics_1p0();
        break;

    case V2_ROBOT:
        _robotVersion = V2_ROBOT;
        _kinematics = new Kinematics_2p2();
        break;

    case V3P3_ROBOT:
        _robotVersion = V3P3_ROBOT;
        _kinematics = new Kinematics_3p3();
        break;

    case V3P4_ROBOT:
        _robotVersion = V3P4_ROBOT;
        _kinematics = new Kinematics_3p4();
        break;

    default:
        _robotVersion = UNKNOWN_ROBOT;
        break;
    }

    if (!_kinematics->isValidModel())
        _robotVersion = UNKNOWN_ROBOT;

    // Connections need to be done after _robotVersion is finally set so that the disconnection in the destructor happens correctly.
    switch (_robotVersion) {
    case V1_ROBOT:
        connect(dynamic_cast<Kinematics_1p0*>(_kinematics), &Kinematics_1p0::kinematicsPoseUpdated, this, &RobotPoseModel::updateRobotPose);
        break;
    case V2_ROBOT:
        connect(dynamic_cast<Kinematics_2p2*>(_kinematics), &Kinematics_2p2::kinematicsPoseUpdated, this, &RobotPoseModel::updateRobotPose);
        break;
    case V3P3_ROBOT:
        connect(dynamic_cast<Kinematics_3p3*>(_kinematics), &Kinematics_3p3::kinematicsPoseUpdated, this, &RobotPoseModel::updateRobotPose);
        break;
    case V3P4_ROBOT:
        connect(dynamic_cast<Kinematics_3p4*>(_kinematics), &Kinematics_3p4::kinematicsPoseUpdated, this, &RobotPoseModel::updateRobotPose);
        break;
    default:
        break;
    }

    _probeTransform.resize(0);
    _probeTransformFilter.resize(0);
    _probeCollisionBoxTransformFilter.resize(0);
    _robotLinkTransform.resize(0);
    _robotLinkTransformFilter.resize(0);
    _robotLinkCollisionBoxTransformFilter.resize(0);

    _collisionCheckObjects.resize(0);
    _collisionCheckIntersectionFilter.resize(0);

    _detectCollisions = false;
    _linkCollisionStatus.resize(0);
    _collisionDetected = false;
}

RobotPoseModel::~RobotPoseModel()
{
    switch (_robotVersion) {
    case V1_ROBOT:
        disconnect(dynamic_cast<Kinematics_1p0*>(_kinematics), nullptr, this, nullptr);
        break;
    case V2_ROBOT:
        disconnect(dynamic_cast<Kinematics_2p2*>(_kinematics), nullptr, this, nullptr);
        break;
    case V3P3_ROBOT:
        disconnect(dynamic_cast<Kinematics_3p3*>(_kinematics), nullptr, this, nullptr);
        break;
    case V3P4_ROBOT:
        disconnect(dynamic_cast<Kinematics_3p4*>(_kinematics), nullptr, this, nullptr);
        break;
    default:
        break;
    }
}

//----------------------------------------------------------------------
// Initialise the LinkData
//----------------------------------------------------------------------
void RobotPoseModel::setLinkData(LinkData* pLinkData)
{
    _pLinkData = pLinkData;

    _probeTransform.resize(getNProbes(), nullptr);
    _probeTransformFilter.resize(getNProbes(), nullptr);
    _probeCollisionBoxTransformFilter.resize(getNProbes(), nullptr);

    if (pLinkData->getProbeData() != nullptr) {
        for (int p = 0; p < getNProbes(); p++) {
            _probeTransform[p] = vtkSmartPointer<vtkTransform>::New();
            _probeTransformFilter[p] = vtkSmartPointer<vtkTransformFilter>::New();
            _probeTransformFilter[p]->SetTransform(_probeTransform[p]);
            _probeTransformFilter[p]->SetInputData(_pLinkData->getProbeData());
            _probeCollisionBoxTransformFilter[p] = vtkSmartPointer<vtkTransformFilter>::New();
            _probeCollisionBoxTransformFilter[p]->SetTransform(_probeTransform[p]);
            _probeCollisionBoxTransformFilter[p]->SetInputConnection(_pLinkData->getProbeCollisionBoxPort());
        }
    }

    _robotLinkTransform.resize(_pLinkData->getNLinks(), nullptr);
    _robotLinkTransformFilter.resize(_pLinkData->getNLinks(), nullptr);
    _robotLinkCollisionBoxTransformFilter.resize(_pLinkData->getNLinks(), nullptr);
    for (int k = 0; k < _pLinkData->getNLinks(); k++) {
        if (pLinkData->getLinkData(k) == nullptr)
            continue;
        _robotLinkTransform[k] = vtkSmartPointer<vtkTransform>::New();
        _robotLinkTransformFilter[k] = vtkSmartPointer<vtkTransformFilter>::New();
        _robotLinkTransformFilter[k]->SetTransform(_robotLinkTransform[k]);
        _robotLinkTransformFilter[k]->SetInputData(_pLinkData->getLinkData(k));
        _robotLinkCollisionBoxTransformFilter[k] = vtkSmartPointer<vtkTransformFilter>::New();
        _robotLinkCollisionBoxTransformFilter[k]->SetTransform(_robotLinkTransform[k]);
        _robotLinkCollisionBoxTransformFilter[k]->SetInputConnection(_pLinkData->getRobotLinkCollisionBoxPort(k));
    }

    _linkCollisionChecks.resize(_pLinkData->getNLinks(), nullptr);
    for (int k = 0; k < _pLinkData->getNLinks(); k++) {
        if (_robotLinkCollisionBoxTransformFilter[k] == nullptr)
            continue;
        _linkCollisionChecks[k] = new GJKCollisionDetection();
        _linkCollisionChecks[k]->setObjectShape(_robotLinkCollisionBoxTransformFilter[k]->GetOutput());
        for (int j = 0; j < _pLinkData->getNLinks(); j++) {
            if (j >= k-2 && j <= k+2)  // Don't check for collisions with self or links either side
                continue;
            if (_robotLinkCollisionBoxTransformFilter[j] == nullptr)
                continue;
            _linkCollisionChecks[k]->addCollisionCheckShape(_robotLinkCollisionBoxTransformFilter[j]->GetOutput());
        }
    }

    _linkCollisionStatus.resize(_pLinkData->getNLinks(), false);

    updateRobotPose();
}

//----------------------------------------------------------------------
// Add a VTK PolyData object with which to check for a collision
//----------------------------------------------------------------------
void RobotPoseModel::removeCollisionCheck(vtkAlgorithmOutput* const collisionCheckObject)
{
    if (collisionCheckObject == nullptr)
        return;

    int idx = -1;
    for (int i = 0; std::cmp_less(i, _collisionCheckObjects.size()); i++) {
        if (_collisionCheckObjects[i] == collisionCheckObject)
            idx = i;
    }

    if (idx >= 0) {
        _collisionCheckIntersectionFilter.erase(_collisionCheckIntersectionFilter.begin() + idx);
        _collisionCheckObjects.erase(_collisionCheckObjects.begin() + idx);
    }
}

void RobotPoseModel::addCollisionCheck(vtkAlgorithmOutput* const collisionCheckObject)
{
    if (collisionCheckObject == nullptr)
        return;

    _collisionCheckObjects.push_back(collisionCheckObject);

    _collisionCheckIntersectionFilter.push_back(std::vector<vtkSmartPointer<vtkIntersectionPolyDataFilter> >(getNLinks(), nullptr));
    for (int k = 0; k < getNLinks(); k++) {
        if (_pLinkData->getRobotLinkCollisionBoxPort(k) == nullptr)
            continue;
        _collisionCheckIntersectionFilter.back()[k] = vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
        _collisionCheckIntersectionFilter.back()[k]->SplitFirstOutputOff();
        _collisionCheckIntersectionFilter.back()[k]->SplitSecondOutputOff();
        _collisionCheckIntersectionFilter.back()[k]->SetInputConnection(0, _robotLinkCollisionBoxTransformFilter[k]->GetOutputPort());
        _collisionCheckIntersectionFilter.back()[k]->SetInputConnection(1, _collisionCheckObjects.back());
    }
}

//----------------------------------------------------------------------
// Access functions
//----------------------------------------------------------------------
int RobotPoseModel::getNProbes() const
{
    return _kinematics->getNProbes();
}

int RobotPoseModel::getNJoints() const
{
    return _kinematics->getNJoints();
}

int RobotPoseModel::getNLinks() const
{
    return _kinematics->getNJoints() + 1;
}

void RobotPoseModel::setCollisionDetection(bool detectCollisions)
{
    _detectCollisions = detectCollisions;
    updateCollisionStatus();
}

//----------------------------------------------------------------------
// Kinematics access functions
//----------------------------------------------------------------------
void RobotPoseModel::setJointPositions(const std::vector<double>& jointPositions, bool recalcPose)
{
    _kinematics->setJointPositions(jointPositions, recalcPose);
}


void RobotPoseModel::getJointPositions(std::vector<double>& jointPositions) const
{
    _kinematics->getJointPositions(jointPositions);
}

void RobotPoseModel::getJointNeutralPositions(std::vector<double>& jointPositions) const
{
    _kinematics->getJointNeutralPositions(jointPositions);
}

void RobotPoseModel::setProbePoses(const std::vector<TRTrans3D>& poses, const std::vector<Point3D>& surfaceNormals, bool recalcProbePoses)
{
    _kinematics->setProbePoses(poses, surfaceNormals, recalcProbePoses);
}

void RobotPoseModel::getProbePoses(std::vector<TRTrans3D>& poses) const
{
    _kinematics->getProbePoses(poses);
}

void RobotPoseModel::getProbeNeutralPoses(std::vector<TRTrans3D>& neutralPoses) const
{
    _kinematics->getProbeNeutralPoses(neutralPoses);
}

bool RobotPoseModel::isPoseInRange() const
{
    std::vector<bool> jointsOk;  // Not used outside this function
    return _kinematics->isPoseInRange(jointsOk);
}

bool RobotPoseModel::isPoseInRange(std::vector<bool>& jointsOk) const
{
    bool result = _kinematics->isPoseInRange(jointsOk);
    return result;
}

bool RobotPoseModel::isCollision() const
{
    if (!_detectCollisions)
        return false;
    else
        return _collisionDetected;
}

bool RobotPoseModel::isCollision(std::vector<bool>& linkCollisions) const
{
    linkCollisions.resize(getNLinks(), false);

    if (!_detectCollisions)
        return false;
    else {
        for (int i = 0; i < getNLinks(); i++) {
            linkCollisions[i] = _linkCollisionStatus[i];
        }
    }

    return _collisionDetected;
}

//----------------------------------------------------------------------
// Display access
//----------------------------------------------------------------------
vtkAlgorithmOutput* RobotPoseModel::getProbePort(int idx)
{
    try {
        if (_probeTransformFilter.at(idx) == nullptr)
            return nullptr;
        else
            return _probeTransformFilter.at(idx)->GetOutputPort();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in RobotPoseModel::getProbePort: index " << idx << "is out of range (0-" << _probeTransformFilter.size()-1 << std::endl;
        return nullptr;
    }
}

vtkAlgorithmOutput* RobotPoseModel::getRobotLinkPort(int idx)
{
    try {
        if (_robotLinkTransformFilter.at(idx) == nullptr)
            return nullptr;
        else
            return _robotLinkTransformFilter.at(idx)->GetOutputPort();
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in RobotPoseModel::getRobotLinkPort: index " << idx << " is out of range (0-" << _robotLinkTransformFilter.size()-1 << std::endl;
        return nullptr;
    }
}

//----------------------------------------------------------------------
// Development function for testing workspace
//----------------------------------------------------------------------
void RobotPoseModel::runThroughWorkspace()
{
    _kinematics->runThroughWorkspace();
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Check for collisions between robot links
//----------------------------------------------------------------------
void RobotPoseModel::updateCollisionStatus()
{
    _linkCollisionStatus.clear();
    _linkCollisionStatus.resize(getNLinks(), false);
    _collisionDetected = false;

    if (!_detectCollisions)
        return;

    vtkObject::GlobalWarningDisplayOff();

//#ifdef WIN32
//    LARGE_INTEGER ticksPerSecond, startTime, endTime;  // For performance counter, because chrono high-def clock is not precise in Windows
//    QueryPerformanceFrequency(&ticksPerSecond);
//    QueryPerformanceCounter(&startTime);
//#else
//    std::chrono::high_resolution_clock::time_point startTime, endTime;
//    startTime = std::chrono::high_resolution_clock::now();
//#endif
//    double timeDiff;

    for (int i = 0; std::cmp_less(i, _collisionCheckIntersectionFilter.size()); i++) {
        for (int j = 0; std::cmp_less(j, _collisionCheckIntersectionFilter[i].size()); j++) {
            if (_collisionCheckIntersectionFilter[i][j] == nullptr)
                continue;
            _collisionCheckIntersectionFilter[i][j]->Update();
            if (_collisionCheckIntersectionFilter[i][j]->GetNumberOfIntersectionPoints() > 0) {
                _linkCollisionStatus[j] = true;
                _collisionDetected = true;
            }
        }
    }

    for (int k = 0; std::cmp_less(k, _linkCollisionChecks.size()); k++) {
        if (_linkCollisionChecks[k] != nullptr && _linkCollisionChecks[k]->isColliding()) {
            _linkCollisionStatus[k] = true;
            _collisionDetected = true;
        }
    }

//#ifdef _WINDOWS
//    QueryPerformanceCounter(&endTime);
//    timeDiff = double(endTime.QuadPart - startTime.QuadPart) / double(ticksPerSecond.QuadPart);
//#else
//    endTime = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> timeDuration = endTime - startTime;
//    timeDiff = timeDuration.count();
//#endif
//    std::cerr << "Collision calculation took " << timeDiff << " seconds" << std::endl;

    vtkObject::GlobalWarningDisplayOn();

    return;
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Slot called when _kinematics runs forward update. Update the vtk
// models of the probe and robot links.
//----------------------------------------------------------------------
void RobotPoseModel::updateRobotPose()
{
    if (_robotVersion == UNKNOWN_ROBOT)
        return;


    if (std::cmp_not_equal(_probeTransform.size(), getNProbes()) || std::cmp_not_equal(_robotLinkTransform.size(), getNLinks()))
        return;  // Link data not yet set

    double mat[16];
    std::vector<TRTrans3D> linkPoses;
    std::vector<TRTrans3D> probePoses;
    _kinematics->getLinkPoses(linkPoses);
    _kinematics->getProbePoses(probePoses);

    // Probe position
    for (int p = 0; p < getNProbes(); p++) {
        if (_probeTransform[p] == nullptr)
            continue;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                mat[i*4 + j] = probePoses[p](i, j);  // vtkMatrix4x4 needs mat in row-major order
        _probeTransform[p]->SetMatrix(mat);
        _probeTransformFilter[p]->Update();
        _probeCollisionBoxTransformFilter[p]->Update();
    }

    // Robot link positions
    // k == 0 (link 0) is always at the global zero position and isn't included in linkPoses
    for (int k = 1; k < getNLinks(); k++) {
        if (_robotLinkTransform[k] == nullptr)
            continue;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                mat[i*4 + j] = linkPoses[k-1](i, j);  // vtkMatrix4x4 needs mat in row-major order
        _robotLinkTransform[k]->SetMatrix(mat);
        _robotLinkTransformFilter[k]->Update();
        _robotLinkCollisionBoxTransformFilter[k]->Update();
    }

    if (_detectCollisions)
        updateCollisionStatus();

    emit robotPoseUpdated();
}
