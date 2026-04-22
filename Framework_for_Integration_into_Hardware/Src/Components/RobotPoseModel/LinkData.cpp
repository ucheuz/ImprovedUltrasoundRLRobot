#include <iostream>
#include <sstream>

#include "LinkData.h"

#include <QString>
#include <QFileInfo>

#include <vtkXMLPolyDataReader.h>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
LinkData::LinkData()
{
    _nLinks = 0;

    _probeData = nullptr;
    _robotLinkData.resize(0);
    _probeCollisionBoxCubeSource = nullptr;
    _robotLinkCollisionBoxCubeSource.resize(0);
    _probeCollisionBoxCleanFilter = nullptr;
    _robotLinkCollisionBoxCleanFilter.resize(0);
    _probeCollisionBoxTriFilter = nullptr;
    _robotLinkCollisionBoxTriFilter.resize(0);
}

LinkData::~LinkData()
{
}

//----------------------------------------------------------------------
// Load the stl files for the probe and robot links
//----------------------------------------------------------------------
void LinkData::initialise(int nLinks, const std::string robotId)
{
    // Probe
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName("./Resources/RobotVTP/probe.vtp");
    reader->Update();

    // Probe mesh data
    _probeData = vtkSmartPointer<vtkPolyData>::New();
    _probeData->DeepCopy(reader->GetOutput());

    _probeCollisionBoxCubeSource = vtkSmartPointer<vtkCubeSource>::New();
    _probeCollisionBoxCubeSource->SetBounds(_probeData->GetBounds());

    _probeCollisionBoxCleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
    _probeCollisionBoxCleanFilter->SetInputConnection(_probeCollisionBoxCubeSource->GetOutputPort());

    _probeCollisionBoxTriFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    _probeCollisionBoxTriFilter->SetInputConnection(_probeCollisionBoxCleanFilter->GetOutputPort());
    _probeCollisionBoxTriFilter->Update();

    // Robot link mesh data
    _robotLinkData.resize(nLinks, nullptr);
    _robotLinkCollisionBoxCubeSource.resize(nLinks, nullptr);
    _robotLinkCollisionBoxCleanFilter.resize(nLinks, nullptr);
    _robotLinkCollisionBoxTriFilter.resize(nLinks, nullptr);

    for (int k = 0; k < nLinks; k++) {
        // Check whether file exists
        std::ostringstream oss;
        oss << "./Resources/RobotVTP/" << robotId << "/Link" << std::setfill('0') << std::setw(2) << k << ".vtp";
        QString filename(oss.str().c_str());
        QFileInfo checkFile(filename);
        if (!checkFile.exists() || !checkFile.isFile())
            continue;

        reader->SetFileName(oss.str().c_str());
        reader->Update();

        _robotLinkData[k] = vtkSmartPointer<vtkPolyData>::New();
        _robotLinkData[k]->DeepCopy(reader->GetOutput());

        _robotLinkCollisionBoxCubeSource[k] = vtkSmartPointer<vtkCubeSource>::New();
        _robotLinkCollisionBoxCubeSource[k]->SetBounds(_robotLinkData[k]->GetBounds());

        _robotLinkCollisionBoxCleanFilter[k] = vtkSmartPointer<vtkCleanPolyData>::New();
        _robotLinkCollisionBoxCleanFilter[k]->SetInputConnection(_robotLinkCollisionBoxCubeSource[k]->GetOutputPort());

        _robotLinkCollisionBoxTriFilter[k] = vtkSmartPointer<vtkTriangleFilter>::New();
        _robotLinkCollisionBoxTriFilter[k]->SetInputConnection(_robotLinkCollisionBoxCleanFilter[k]->GetOutputPort());
        _robotLinkCollisionBoxTriFilter[k]->Update();
    }

    _nLinks = nLinks;
}

//----------------------------------------------------------------------
// Access functions
//----------------------------------------------------------------------
vtkPolyData* LinkData::getProbeData() const
{
    return _probeData;
}

vtkPolyData* LinkData::getLinkData(int idx) const
{
    try {
        return _robotLinkData.at(idx);
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in LinkData::getLinkData: index is out of range (0-" << _robotLinkData.size() << ")" << std::endl;
        return nullptr;
    }
}

int LinkData::getNLinks() const
{
    return _nLinks;
}

vtkAlgorithmOutput* LinkData::getProbeCollisionBoxPort() const
{
    return _probeCollisionBoxTriFilter->GetOutputPort();
}

vtkAlgorithmOutput* LinkData::getRobotLinkCollisionBoxPort(int idx) const
{
    try {
        if (_robotLinkCollisionBoxTriFilter.at(idx) == nullptr)
            return nullptr;
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error in LinkData::getRobotLinkCollisionBoxPort: index is out of range (0-" << _robotLinkCollisionBoxTriFilter.size() << ")" << std::endl;
        return nullptr;
    }

    return _robotLinkCollisionBoxTriFilter[idx]->GetOutputPort();
}
