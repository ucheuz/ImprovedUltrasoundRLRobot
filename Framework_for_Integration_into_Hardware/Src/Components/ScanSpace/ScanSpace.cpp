#include <iostream>
#include <utility>

#include "ScanSpace.h"

#include "Point3D.h"  // From TransMatrix library

#include <vtkXMLPolyDataReader.h>
#include <vtkCellData.h>
#include <vtkMatrix4x4.h>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
ScanSpace::ScanSpace(QObject *parent) : QObject(parent)
{
    initialiseSurfaceData();
}

ScanSpace::~ScanSpace()
{
}

//----------------------------------------------------------------------
// Load the surface data
//----------------------------------------------------------------------
void ScanSpace::loadSurfaceMesh(const QString& filename)
{
    vtkSmartPointer<ErrorObserver> errorObserver = vtkSmartPointer<ErrorObserver>::New();

    // Read pre-processed surface from vtp file.
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(filename.toLocal8Bit().data());
    //reader->SetFileName("./Resources/surface_seg_orientation.vtp");
    //reader->SetFileName("./Resources/kawal3_surface2.vtp");
    //reader->SetFileName("./Resources/Ellipsoid_surface.vtp");
    reader->AddObserver(vtkCommand::ErrorEvent, errorObserver);
    reader->Update();

    if (errorObserver->getError()) {
        std::cerr << "Error in ScanSpace::loadSurfaceMesh:" << std::endl;
        std::cerr << errorObserver->getErrorMessage() << std::endl;
        _surfStatus = LOAD_FAILED;
        emit surfaceMeshStatus("Load failed");
        return;
    }

    _surfData->DeepCopy(reader->GetOutput());

    _surfTransformFilter->Update();
    _normalFilter->Update();
    _cellLocator->BuildLocator();

    _surfStatus = LOADED;
    emit surfaceMeshStatus("Loaded");

    emit surfaceMeshChanged();
}

//----------------------------------------------------------------------
// Set the position of the surface
//----------------------------------------------------------------------
void ScanSpace::setSurfacePosition(Point3D position)
{
    vtkSmartPointer<vtkMatrix4x4> mat = vtkSmartPointer<vtkMatrix4x4>::New();
    mat->Identity();
    mat->SetElement(0, 3, position.X());
    mat->SetElement(1, 3, position.Y());
    mat->SetElement(2, 3, position.Z());
    _surfTransform->SetMatrix(mat);

    if (isSurfaceLoaded()) {
        // Locator needs rebuilding if the surface has moved
        _surfTransformFilter->Update();
        _normalFilter->Update();
        _cellLocator->BuildLocator();
    }

    emit surfaceMeshChanged();
}

//----------------------------------------------------------------------
// Check for a valid surface mesh
//----------------------------------------------------------------------
bool ScanSpace::isSurfaceLoaded() const
{
    return _surfStatus == LOADED;
}

//----------------------------------------------------------------------
// Get output mesh
//----------------------------------------------------------------------
vtkAlgorithmOutput* ScanSpace::getSurfacePort()
{
    //if (isSurfaceLoaded())
        return _surfTransformFilter->GetOutputPort();
    /*else {
        std::cerr << "Warning in ScanSpace::getSurfacePort: no surface loaded." << std::endl;
        return nullptr;
    }*/
}

//----------------------------------------------------------------------
// Given a 3D position (point), find the nearest point on the surface
// and also the surface normal and distance to the surface at that
// point.
//----------------------------------------------------------------------
Point3D ScanSpace::getNearestPointOnSurface(const Point3D& point, Point3D& normal, double& dist) const
{
    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::getNearestPointOnSurface: no surface loaded." << std::endl;
        normal.set(0.0, 0.0, 1.0);
        dist = 0.0;
        return Point3D(point);
    }

    // Find current nearest point on the surface using VTKCellLocator
    Point3D closestPoint;
    double pt[3], closestPt[3], dist2;
    vtkIdType cellId;
    int subId;
    pt[0] = point.X();  pt[1] = point.Y();  pt[2] = point.Z();
    _cellLocator->FindClosestPoint(pt, closestPt, cellId, subId, dist2);
    closestPoint.set(closestPt[0], closestPt[1], closestPt[2]);

    // Distance to surface
    dist = sqrt(dist2);
    if (point.Z() < closestPt[2])
        dist = -dist;

    // Surface normal at this point
    double norm[3];
    _normalFilter->GetOutput()->GetCellData()->GetNormals()->GetTuple(cellId, norm);
    normal.set(norm[0], norm[1], norm[2]);

    return closestPoint;
}

//----------------------------------------------------------------------
// Given the current pose, work out the nearest pose that puts the probe
// in contact with and normal to the surface.
//----------------------------------------------------------------------
void ScanSpace::getNearestSurfaceAlignedPose(const TRTrans3D& currentPose, TRTrans3D& alignedPose) const
{
    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::getNearestSurfaceAlignedPose: no surface loaded." << std::endl;
        alignedPose.set(currentPose);
        return;
    }

    // Find current nearest point and normal on the surface
    Point3D currentProbePoint, currentSurfPoint, normal;
    double origDist;
    currentPose.getTranslation(currentProbePoint);
    currentSurfPoint = getNearestPointOnSurface(currentProbePoint, normal, origDist);

    // Calculate new X direction perpendicular to the surface normal
    Point3D xDir, yDir, zDir;
    currentPose.getBasisVectors(xDir, yDir, zDir);
    xDir = xDir - normal * xDir.dot(normal);
    xDir.normalise();

    // New Y direction is aligned with the normal
    yDir = -normal;
    if (yDir.dot(Point3D(0,0,1)) > 0)
        yDir = -yDir;
    yDir.normalise();

    // Set the new orientation
    alignedPose.setIdentity();
    alignedPose.setBasisVectors(xDir, yDir);
    alignedPose.setTranslation(currentSurfPoint);
}

//----------------------------------------------------------------------
// Given the current pose, work out a new pose following the surface in
// a given direction for a given distance.
//----------------------------------------------------------------------
void ScanSpace::calcSurfaceStepPose(const TRTrans3D& currentPose, const Point3D& direction, double distance, TRTrans3D& stepPose) const
{
    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::calcSurfaceStepPose: no surface loaded." << std::endl;
        stepPose.set(currentPose);
        return;
    }

    // Find current nearest point and normal on the surface
    Point3D currentProbePoint, currentSurfPoint, normal;
    double origDist;
    currentPose.getTranslation(currentProbePoint);
    currentSurfPoint = getNearestPointOnSurface(currentProbePoint, normal, origDist);

    // Work out the tangential vector from the current position in the given direction
    Point3D dir = currentPose.transformDirection(direction);
    dir = dir - normal * dir.dot(normal);
    dir.normalise();

    // Translate by given distance
    Point3D nextPoint = currentSurfPoint + dir * distance;

    // Find the nearest point on the surface to the new point and get its normal
    Point3D nextClosestPoint, nextNormal;
    double tmpDist;
    nextClosestPoint = getNearestPointOnSurface(nextPoint, nextNormal, tmpDist);

    // Set the new orientation so that the normal is the same relative to the original probe pose
    stepPose.setIdentity();
    if (normal == nextNormal) {
        stepPose.set(currentPose);  // Orientation unchanged
    }
    else {
        TRTrans3D R;
        Point3D rotateAxis(normal.cross(nextNormal));
        double rotateAngle = asin(rotateAxis.norm());  // Radians
        rotateAxis.normalise();
        R.setRotation(cos(rotateAngle/2.0), rotateAxis * sin(rotateAngle/2.0));
        R.setTranslation(0, 0, 0);
        stepPose = R * currentPose;
    }
    // Only orientation is valid so far. Now set translation
    stepPose.setTranslation(nextClosestPoint + nextNormal * origDist);
}

//----------------------------------------------------------------------
// Given the current poses of several probes, work out a new set of
// poses following the surface in a given direction for a given
// distance.
//----------------------------------------------------------------------
void ScanSpace::calcGroupSurfaceStepPoses(const std::vector<TRTrans3D>& currentPoses, const Point3D& direction, double distance, std::vector<TRTrans3D>& stepPoses) const
{
    if (currentPoses.size() < 2) {
        std::cerr << "Error in ScanSpace::calcGroupSurfaceStepPoses: need at least 2 probes to calculate group movements." << std::endl;
        return;
    }

    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::calcGroupSurfaceStepPoses: no surface loaded." << std::endl;
        stepPoses = currentPoses;
        return;
    }

    stepPoses.resize(currentPoses.size());

    // Find group pose for the probes
    TRTrans3D currentGroupPose;
    std::vector<TRTrans3D> T_probeToGroup;
    calcGroupPose(currentPoses, currentGroupPose, T_probeToGroup);

    // Surface step pose for the group
    TRTrans3D groupStepPose;
    calcSurfaceStepPose(currentGroupPose, direction, distance, groupStepPose);

    // New pose of each probe
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++) {
        // Surface normal for each probe in their original positions
        Point3D origPoint, origNormal;
        double origDist;
        currentPoses[p].getTranslation(origPoint);
        getNearestPointOnSurface(origPoint, origNormal, origDist);

        // Probe tilt from normal in original position
        TRTrans3D alignedPose, tilt;
        getNearestSurfaceAlignedPose(currentPoses[p], alignedPose);
        tilt = (~alignedPose) * currentPoses[p];

        // New position of probe
        TRTrans3D tmpStepPose = groupStepPose * T_probeToGroup[p] * (~tilt);
        Point3D nextPoint;
        tmpStepPose.getTranslation(nextPoint);

        // Realign to surface at new position
        getNearestSurfaceAlignedPose(tmpStepPose, stepPoses[p]);

        // Find the nearest point on the surface to the new point and get its normal
        Point3D nextClosestPoint, nextNormal;
        double tmpDist;
        nextClosestPoint = getNearestPointOnSurface(nextPoint, nextNormal, tmpDist);

        // Reapply tilt
        stepPoses[p] *= tilt;

        // Only orientation is valid so far. Now set translation
        stepPoses[p].setTranslation(nextClosestPoint + nextNormal * origDist);
    }
}

//----------------------------------------------------------------------
// Given the current poses of several probes, work out a new set of
// poses following the surface after a group axial rotation.
//----------------------------------------------------------------------
void ScanSpace::calcGroupSurfaceRotationPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& rotation, std::vector<TRTrans3D>& rotatePoses) const
{
    if (currentPoses.size() < 2) {
        std::cerr << "Error in ScanSpace::calcGroupSurfaceRotationPoses: need at least 2 probes to calculate group movements." << std::endl;
        return;
    }

    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::calcGroupSurfaceRotationPoses: no surface loaded." << std::endl;
        rotatePoses = currentPoses;
        return;
    }

    rotatePoses.resize(currentPoses.size());

    // Find group pose for the probes
    TRTrans3D currentGroupPose;
    std::vector<TRTrans3D> T_probeToGroup;
    calcGroupPose(currentPoses, currentGroupPose, T_probeToGroup);

    // Rotated pose for the group
    TRTrans3D groupRotatePose = currentGroupPose * rotation;

    // New pose of each probe
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++) {
        // Surface distance of probe in its original position
        Point3D origPoint, origNormal;
        double origDist;
        currentPoses[p].getTranslation(origPoint);
        getNearestPointOnSurface(origPoint, origNormal, origDist);

        // Probe tilt from normal in original position
        TRTrans3D alignedPose, tilt;
        getNearestSurfaceAlignedPose(currentPoses[p], alignedPose);
        tilt = (~alignedPose) * currentPoses[p];

        // New position of probe
        TRTrans3D tmpRotatePose = groupRotatePose * T_probeToGroup[p] * (~tilt);
        Point3D nextPoint;
        tmpRotatePose.getTranslation(nextPoint);

        // Realign to surface at new position
        getNearestSurfaceAlignedPose(tmpRotatePose, rotatePoses[p]);

        // Find the nearest point on the surface to the new point and get its normal
        Point3D nextClosestPoint, nextNormal;
        double tmpDist;
        nextClosestPoint = getNearestPointOnSurface(nextPoint, nextNormal, tmpDist);

        // Reapply tilt
        rotatePoses[p] *= tilt;

        // Only orientation is valid so far. Now set translation
        rotatePoses[p].setTranslation(nextClosestPoint + nextNormal * origDist);
    }
}

//----------------------------------------------------------------------
// Given the current poses of several probes, work out a new set of
// poses after a group tilt.
//----------------------------------------------------------------------
void ScanSpace::calcGroupSurfaceTiltPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& tiltAdjust, std::vector<TRTrans3D>& tiltPoses) const
{
    if (currentPoses.size() < 2) {
        std::cerr << "Error in ScanSpace::calcGroupSurfaceTiltPoses: need at least 2 probes to calculate group movements." << std::endl;
        return;
    }

    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::calcGroupSurfaceTiltPoses: no surface loaded." << std::endl;
        tiltPoses = currentPoses;
        return;
    }

    tiltPoses.resize(currentPoses.size());

    // Find group pose for the probes
    TRTrans3D currentGroupPose;
    std::vector<TRTrans3D> T_probeToGroup;
    calcGroupPose(currentPoses, currentGroupPose, T_probeToGroup);

    // Rotated pose for the group
    TRTrans3D groupTiltPose = currentGroupPose * tiltAdjust;

    // New pose of each probe
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++) {
        // Surface distance of probe in its original position
        Point3D origPoint, origNormal;
        double origDist;
        currentPoses[p].getTranslation(origPoint);
        getNearestPointOnSurface(origPoint, origNormal, origDist);

        // New position of probe
        tiltPoses[p] = groupTiltPose * T_probeToGroup[p];
        Point3D nextPoint;
        tiltPoses[p].getTranslation(nextPoint);

        // Find the nearest point on the surface to the new point and get its normal
        Point3D nextClosestPoint, nextNormal;
        double tmpDist;
        nextClosestPoint = getNearestPointOnSurface(nextPoint, nextNormal, tmpDist);

        // Only orientation is valid so far. Now set translation
        tiltPoses[p].setTranslation(nextClosestPoint + nextNormal * origDist);
    }
}

//----------------------------------------------------------------------
// Given the current poses of several probes, work out a new set of
// poses after a group pose adjustment.
//----------------------------------------------------------------------
void ScanSpace::calcGroupPoseAdjustPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& poseAdjust, bool isLocal, std::vector<TRTrans3D>& adjustedPoses) const
{
    if (currentPoses.size() < 2) {
        std::cerr << "Error in ScanSpace::calcGroupPoseAdjustPoses: need at least 2 probes to calculate group movements." << std::endl;
        return;
    }

    adjustedPoses.resize(currentPoses.size());

    // Find group pose for the probes
    TRTrans3D currentGroupPose;
    std::vector<TRTrans3D> T_probeToGroup;
    calcGroupPose(currentPoses, currentGroupPose, T_probeToGroup);

    TRTrans3D groupAdjustedPose(currentGroupPose);

    // Rotated pose for the group
    if (isLocal) {
        groupAdjustedPose *= poseAdjust;  // Apply adjustment in local coordinate system of group
    }
    else {
        Point3D position, newPosition;
        currentGroupPose.getTranslation(position);

        groupAdjustedPose.setTranslation(0.0, 0.0, 0.0);  // Translate group to origin, so rotation will be applied about group origin
        groupAdjustedPose = poseAdjust * groupAdjustedPose;  // Apply adjustment in global coordinate system
        groupAdjustedPose.getTranslation(newPosition);
        groupAdjustedPose.setTranslation(newPosition + position);  // Translate back
    }

    // New pose of each probe
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++)
        adjustedPoses[p] = groupAdjustedPose * T_probeToGroup[p];
}

//----------------------------------------------------------------------
// Work out a sequence of positions to follow the surface for a given
// distance in the elevational direction.
//----------------------------------------------------------------------
void ScanSpace::followSurfaceElevational(const TRTrans3D& startingPose, double distance, std::vector<TRTrans3D>& sequence)
{
    if (!isSurfaceLoaded()) {
        std::cerr << "Warning in ScanSpace::followSurfaceElevational: no surface loaded." << std::endl;
        sequence.clear();
        return;
    }

    // Divide the movement into subsections
    double maxStep = 10.0;  // mm
    int nSteps = ceil(fabs(distance) / maxStep);

    sequence.clear();
    TRTrans3D currentPose(startingPose);
    std::cerr << "Starting pose" << std::endl;
    currentPose.printMatrix();

    for (int s = 0; s < nSteps; s++) {
        std::cerr << "Step " << s+1 << std::endl;

        // Find current nearest point and normal on the surface
        Point3D currentProbePoint, currentSurfPoint, normal;
        double dist;
        currentPose.getTranslation(currentProbePoint);
        currentSurfPoint = getNearestPointOnSurface(currentProbePoint, normal, dist);
        std::cerr << dist << std::endl;

        // Work out the elevational tangential direction from the current position
        Point3D elev(currentPose.getMatrix()(0,2), currentPose.getMatrix()(1,2), currentPose.getMatrix()(2,2));
        elev = elev - normal * elev.dot(normal);
        elev.normalise();
        //elev.print();

        // Translate by one step
        Point3D nextPoint;
        double moveDist = (s == nSteps-1) ? (fabs(distance) - (nSteps-1) * maxStep) : maxStep;
        if (distance < 0)
            moveDist = -moveDist;
        nextPoint = currentSurfPoint + elev * moveDist;
        //nextPoint.print();

        // Find the nearest point on the surface to the new point and get its normal
        Point3D nextClosestPoint, normal2;
        nextClosestPoint = getNearestPointOnSurface(nextPoint, normal2, dist);

        // Create the new pose matrix
        TRTrans3D nextPose;
        Point3D axDir(normal2);  // *** Currently assuming the probe is not tilted
        axDir.normalise();  // Maybe not necessary
        Point3D elevDir = elev - axDir * elev.dot(axDir);
        elevDir.normalise();
        Point3D latDir = axDir.cross(elevDir);
        nextPose.setBasisVectors(latDir, axDir);
        nextPose.setTranslation(nextClosestPoint + normal * dist);
        //TransMatrix3D T;
        //T.setIdentity();
        //T(0, 0) = latDir.X();  T(0, 1) = axDir.X();  T(0, 2) = elevDir.X();  T(0, 3) = nextClosestPoint.X() + normal.X() * dist;
        //T(1, 0) = latDir.Y();  T(1, 1) = axDir.Y();  T(1, 2) = elevDir.Y();  T(1, 3) = nextClosestPoint.Y() + normal.Y() * dist;
        //T(2, 0) = latDir.Z();  T(2, 1) = axDir.Z();  T(2, 2) = elevDir.Z();  T(2, 3) = nextClosestPoint.Z() + normal.Z() * dist;
        //nextPose.set(T);

        sequence.push_back(nextPose);
        currentPose.set(nextPose);
    }
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Initialise VTK surface data
//----------------------------------------------------------------------
void ScanSpace::initialiseSurfaceData()
{
    _surfData = vtkSmartPointer<vtkPolyData>::New();

    _surfTransform = vtkSmartPointer<vtkTransform>::New();

    _surfTransformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    _surfTransformFilter->SetTransform(_surfTransform);
    _surfTransformFilter->SetInputData(_surfData);

    _normalFilter = vtkSmartPointer<vtkPolyDataNormals>::New();
    _normalFilter->SetInputData(_surfTransformFilter->GetOutput());
    _normalFilter->ComputePointNormalsOff();
    _normalFilter->ComputeCellNormalsOn();

    _cellLocator = vtkSmartPointer<vtkCellLocator>::New();
    _cellLocator->SetDataSet(_normalFilter->GetOutput());

    _surfStatus = NOT_LOADED;
}

//----------------------------------------------------------------------
// Work out a pose representing a group of probes
//----------------------------------------------------------------------
void ScanSpace::calcGroupPose(const std::vector<TRTrans3D>& currentPoses, TRTrans3D& currentGroupPose, std::vector<TRTrans3D>& T_probeToGroup) const
{
    T_probeToGroup.resize(currentPoses.size());

    // Find group pose for the probes
    // Group position is average position of probes
    Point3D translationSum;
    std::vector<Point3D> positions(currentPoses.size());
    translationSum.setZero();
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++) {
        currentPoses[p].getTranslation(positions[p]);
        translationSum += positions[p];
    }
    translationSum /= currentPoses.size();
    currentGroupPose.setTranslation(translationSum);

    // Group X direction is from probe 1 to 2 position
    Point3D xDir;
    xDir = positions[1] - positions[0];
    xDir.normalise();

    // Group Y direction is average Y direction of probes, orthogonal to xDir
    Point3D yDir, xd, yd, zd;
    yDir.setZero();
    for (const TRTrans3D& currentPose : currentPoses) {
        currentPose.getBasisVectors(xd, yd, zd);
        yDir += yd;
    }
    yDir -= xDir * yDir.dot(xDir);
    yDir.normalise();
    currentGroupPose.setBasisVectors(xDir, yDir);

    // Positions of each probe relative to the group pose
    T_probeToGroup.resize(currentPoses.size());
    for (int p = 0; std::cmp_less(p, currentPoses.size()); p++)
        T_probeToGroup[p] = (~currentGroupPose) * currentPoses[p];
}

//======================================================================
// ErrorObserver class for PolyData reader
//======================================================================
ErrorObserver::ErrorObserver()
{
    clear();
}

ErrorObserver* ErrorObserver::New()
{
    return new ErrorObserver;
}

void ErrorObserver::Execute(vtkObject *vtkNotUsed(caller), unsigned long event, void *calldata)
{
    switch(event) {
    case vtkCommand::ErrorEvent:
        _errorMessage = static_cast<char *>(calldata);
        _error = true;
        break;
    }
}

void ErrorObserver::clear()
{
    _error = false;
    _errorMessage = "";
}

bool ErrorObserver::getError() const
{
    return _error;
}

std::string ErrorObserver::getErrorMessage() const
{
    return _errorMessage;
}
