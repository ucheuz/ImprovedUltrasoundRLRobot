#ifndef LINKDATA_H
#define LINKDATA_H

#include <vector>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>
#include <vtkTriangleFilter.h>

//======================================================================
// LinkData class stores the robot link and probe stl files. Its purpose
// is to allow a single copy of the data that other classes can access.
//======================================================================
struct LinkData
{
public:
    LinkData();
    ~LinkData();

    void initialise(int nLinks, const std::string robotId);

    // Access functions
    vtkPolyData* getProbeData() const;
    vtkPolyData* getLinkData(int idx) const;
    int getNLinks() const;

    vtkAlgorithmOutput* getProbeCollisionBoxPort() const;
    vtkAlgorithmOutput* getRobotLinkCollisionBoxPort(int idx) const;

private:
    std::vector<vtkSmartPointer<vtkPolyData> > _robotLinkData;
    vtkSmartPointer<vtkPolyData> _probeData;
    std::vector<vtkSmartPointer<vtkCubeSource> > _robotLinkCollisionBoxCubeSource;
    vtkSmartPointer<vtkCubeSource> _probeCollisionBoxCubeSource;
    std::vector<vtkSmartPointer<vtkCleanPolyData> > _robotLinkCollisionBoxCleanFilter;
    vtkSmartPointer<vtkCleanPolyData> _probeCollisionBoxCleanFilter;
    std::vector<vtkSmartPointer<vtkTriangleFilter> > _robotLinkCollisionBoxTriFilter;
    vtkSmartPointer<vtkTriangleFilter> _probeCollisionBoxTriFilter;
    int _nLinks;
};

#endif // LINKDATA_H
