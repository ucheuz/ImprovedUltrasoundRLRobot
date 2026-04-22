#ifndef SCANSPACE_H
#define SCANSPACE_H

#include "TRTrans3D.h"  // From TransMatrix library

#include <QObject>
#include <QString>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkCellLocator.h>

//======================================================================
// ScanSpace class stores the scanned surface and provides functions for
// calculating positions relative to the surface.
//======================================================================
class ScanSpace : public QObject
{
    Q_OBJECT

public:
    explicit ScanSpace(QObject *parent = nullptr);
    ~ScanSpace();

    enum SurfaceMeshStatus_t {NOT_LOADED=0, LOAD_FAILED, LOADED};

    void loadSurfaceMesh(const QString& filename);
    void setSurfacePosition(Point3D position);

    bool isSurfaceLoaded() const;

    vtkAlgorithmOutput* getSurfacePort();

    Point3D getNearestPointOnSurface(const Point3D& point, Point3D& normal, double& dist) const;
    void getNearestSurfaceAlignedPose(const TRTrans3D& currentPose, TRTrans3D& alignedPose) const;
    void calcSurfaceStepPose(const TRTrans3D& currentPose, const Point3D& direction, double distance, TRTrans3D& stepPose) const;
    void calcGroupSurfaceStepPoses(const std::vector<TRTrans3D>& currentPoses, const Point3D& direction, double distance, std::vector<TRTrans3D>& stepPoses) const;
    void calcGroupSurfaceTiltPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& tiltAdjust, std::vector<TRTrans3D>& tiltPoses) const;
    void calcGroupSurfaceRotationPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& rotation, std::vector<TRTrans3D>& rotatePoses) const;
    void calcGroupPoseAdjustPoses(const std::vector<TRTrans3D>& currentPoses, const TRTrans3D& poseAdjust, bool isLocal, std::vector<TRTrans3D>& adjustedPoses) const;
    void followSurfaceElevational(const TRTrans3D& startingPose, double distance, std::vector<TRTrans3D>& sequence);  // *** Experimental function

signals:
    void surfaceMeshChanged();
    void surfaceMeshStatus(QString status);

private:
    vtkSmartPointer<vtkPolyData> _surfData;
    vtkSmartPointer<vtkTransform> _surfTransform;
    vtkSmartPointer<vtkTransformFilter> _surfTransformFilter;
    vtkSmartPointer<vtkPolyDataNormals> _normalFilter;

    vtkSmartPointer<vtkCellLocator> _cellLocator;

    SurfaceMeshStatus_t _surfStatus;

    void initialiseSurfaceData();
    void calcGroupPose(const std::vector<TRTrans3D>& currentPoses, TRTrans3D& currentGroupPose, std::vector<TRTrans3D>& T_probeToGroup) const;
};

#include <vtkCommand.h>

class ErrorObserver : public vtkCommand
{
public:
    ErrorObserver();
    static ErrorObserver* New();

    virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long event, void *calldata);

    void clear();

    bool getError() const;
    std::string getErrorMessage() const;

private:
    bool        _error;
    std::string _errorMessage;
};

#endif // SCANSPACE_H
