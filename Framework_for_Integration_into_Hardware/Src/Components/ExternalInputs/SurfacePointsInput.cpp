#include <iostream>

#include "SurfacePointsInput.h"

#include "Point3D.h"  // From TransMatrix library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
SurfacePointsInput::SurfacePointsInput(QObject* parent) : QObject(parent)
{
    _surfPointsDataThread = nullptr;
    _surfPointsData = nullptr;

    _controlledProbe = 0;  // *** Needs to be adjustable for full control

    _status = DISCONNECTED;

    qRegisterMetaType<std::vector<double> >("std::vector<double>");
    qRegisterMetaType<TRTrans3D>("TRTrans3D");
}

SurfacePointsInput::~SurfacePointsInput()
{
    disconnectSurfPointsData();
}

//----------------------------------------------------------------------
// Connect surface points data
//----------------------------------------------------------------------
void SurfacePointsInput::connectSurfPointsData()
{
    disconnectSurfPointsData();

    _surfPointsDataThread = new QThread();
    _surfPointsData = new SurfacePointsData();

    bool success = _surfPointsData->initialise(_surfPointsDataIpAddress, _receivePort);

    if (!success) {
        std::cerr << "Failed to create UDP socket for SurfacePointsData" << std::endl;
        delete _surfPointsData;
        delete _surfPointsDataThread;
    }
    else {
        _surfPointsData->moveToThread(_surfPointsDataThread);

        // Thread signals
        connect(_surfPointsDataThread, &QThread::started, _surfPointsData, &SurfacePointsData::run);
        connect(this, &SurfacePointsInput::stopSurfPointsDataProcess, _surfPointsData, &SurfacePointsData::stop);
        connect(_surfPointsData, &SurfacePointsData::finished, _surfPointsDataThread, &QThread::quit);
        connect(_surfPointsData, &SurfacePointsData::finished, _surfPointsData, &SurfacePointsData::deleteLater);
        connect(_surfPointsDataThread, &QThread::finished, _surfPointsDataThread, &QThread::deleteLater);

        _surfPointsDataThread->start();

        connect(_surfPointsData, &SurfacePointsData::poseRequestReceived, this, &SurfacePointsInput::updateTargetPose);
    }

    if (_surfPointsData != nullptr) {
        _status = CONNECTED;
        emit connectionStatus("Port open");
    }
    else {
        _status = CONNECT_FAILED;
        emit connectionStatus("Connect failed");
    }

    //emit connectionChanged();
}

//----------------------------------------------------------------------
// Get surface points data source connection status
//----------------------------------------------------------------------
bool SurfacePointsInput::isConnected() const
{
    return _surfPointsData != nullptr;
}

void SurfacePointsInput::setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses)
{
    _neutralPoses = neutralPoses;
}

void SurfacePointsInput::setIpAddress(const QHostAddress& address)
{
    _surfPointsDataIpAddress = address;
    connectSurfPointsData();
}

void SurfacePointsInput::setPort(unsigned short port)
{
    _receivePort = port;
    connectSurfPointsData();
}

//----------------------------------------------------------------------
// Access pose values
// Return value indicates whether the pose is valid
//----------------------------------------------------------------------
bool SurfacePointsInput::getCurrentPoseData(std::vector<double>& poseData) const
{
    if (_surfPointsData == nullptr)
        return false;
    else {
        bool valid = _surfPointsData->getPoseData(poseData);
        return valid;
    }
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the surface points data source  and clear variables ready
// to reconnect
//----------------------------------------------------------------------
void SurfacePointsInput::disconnectSurfPointsData()
{
    // Signal surface points data source to disconnect
    emit stopSurfPointsDataProcess();

    // Wait for surface points data source to disconnect fully so it's safe to reconnect if needed
    while (_surfPointsData != nullptr)
        qApp->processEvents();

    disconnect(_surfPointsData, nullptr, this, nullptr);

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    //emit connectionChanged();
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// QTimer slot to send a target pose request
//----------------------------------------------------------------------
void SurfacePointsInput::updateTargetPose()
{
    if (_surfPointsData == nullptr)
        return;

    TRTrans3D pose;

    std::vector<double> poseData;
    bool valid = getCurrentPoseData(poseData);
    if (!valid)
        return;

    Point3D position(poseData[0], poseData[1], poseData[2]);
    Point3D normal(poseData[3], poseData[4], poseData[5]);

    // Define orientation to align with normal vector (i.e. align with surface)

    // Calculate new X direction perpendicular to the surface normal
    Point3D xDir, yDir, zDir;
    _neutralPoses[_controlledProbe].getBasisVectors(xDir, yDir, zDir);
    xDir = xDir - normal * xDir.dot(normal);
    xDir.normalise();

    Point3D neutralPosition;
    _neutralPoses[_controlledProbe].getTranslation(neutralPosition);

    // New Y direction is aligned with the normal, pointing towards surface (-normal)
    yDir = -normal;
    if (yDir.dot(Point3D(0,0,1)) > 0)
        yDir = -yDir;
    yDir.normalise();

    // Set the new orientation
    pose.setIdentity();
    pose.setBasisVectors(xDir, yDir);
    pose.setTranslation(neutralPosition+position);

    emit singleProbePoseRequest(_controlledProbe, pose);
}
