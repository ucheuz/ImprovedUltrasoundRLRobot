#include <iostream>
#include <cmath>

#include "IGTLInput.h"

#define EPS_D std::numeric_limits<double>::epsilon()

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
IGTLInput::IGTLInput(QObject *parent) : QObject(parent)
{
#ifndef WIN32
    _trackingDataServer = igtl::iFindSessionManager::New();
    _trackingDataServer->SetMode(igtl::iFindSessionManager::MODE_SERVER);
    _trackingDataServer->SetDebug(0);
#endif

    _status = DISCONNECTED;
    _receivePort = 11000;  // Default

    _centrePose.setIdentity();

    for (int i = 0; i < _maxProbes*_nDofs; i++)
        _poseData[i] = 0.0;

    qRegisterMetaType<std::vector<TRTrans3D> >("std::vector<TRTrans3D>");
}

IGTLInput::~IGTLInput()
{
    disconnectIGTL();
}

//----------------------------------------------------------------------
// Connect IGTL server
//----------------------------------------------------------------------
void IGTLInput::connectIGTL()
{
#ifndef WIN32
    disconnectIGTL();

    // IGTL server, for communicating position information to and from the imaging software
    _trackingDataServer->SetPort(_receivePort);
    int r = _trackingDataServer->Start();
    bool success = (r > 0);

    if (!success) {
        std::cerr << "Error in IGTLInput::connectIGTL: failed to open port connection (r = " << r << ")" << std::endl;
        disconnect(_trackingDataServer, nullptr, this, nullptr);
    }
    else {
        //updateRobotPose();  // Update server with current pose
        std::cout << "connecting signals" << std::endl;
        connect(_trackingDataServer, &igtl::iFindSessionManager::TrackingDataReceived,
                this, &IGTLInput::positionRequestReceived,
                Qt::DirectConnection);
    }

    if (success) {
        _status = CONNECTED;
        emit connectionStatus("Port open");
    }
    else {
        _status = CONNECT_FAILED;
        emit connectionStatus("Connect failed");
    }

    //emit connectionChanged();
#endif  // !WIN32
}

//----------------------------------------------------------------------
// Get IGTL connection status
//----------------------------------------------------------------------
bool IGTLInput::isConnected() const
{
    return _status == CONNECTED;
}

//----------------------------------------------------------------------
// Set netral position of the robot. All position requests are relative
// to this
//----------------------------------------------------------------------
void IGTLInput::setCentrePose(const TRTrans3D& centrePose)
{
    _centrePose.set(centrePose);
}

//----------------------------------------------------------------------
// Set the port number to use
//----------------------------------------------------------------------
void IGTLInput::setPort(unsigned int port)
{
    _receivePort = port;
}

//----------------------------------------------------------------------
// Access pose values
// Return value indicates whether the pose is valid
//----------------------------------------------------------------------
bool IGTLInput::getCurrentPoseData(int idx, std::vector<double>& poseData) const
{
#ifndef WIN32
    poseData.resize(_nDofs, 0.0);

    if (!_trackingDataServer->IsRunning())
        return false;

    else {
        if (idx < 0 || idx >= _maxProbes) {
            std::cerr << "Error in IGTLInput::getCurrentPoseData: index out of range." << std::endl;
            return false;
        }

#endif  // !WIN32
        for (int i = 0; i < _nDofs; i++)
            poseData[i] = _poseData[idx*_nDofs + i];

        // Invalid pose if quaternion is zero
        return !(std::fabs(poseData[3]) < EPS_D && std::fabs(poseData[4]) < EPS_D &&
                 std::fabs(poseData[5]) < EPS_D && std::fabs(poseData[6]) < EPS_D);
#ifndef WIN32
	}
#endif  // !WIN32
}

//----------------------------------------------------------------------
// Update current data in the server
//----------------------------------------------------------------------
void IGTLInput::setCurrentPoses(const std::vector<TRTrans3D>& probePoses)
{
#ifndef WIN32
    if (!_trackingDataServer->IsConnected())
        return;

    if (std::cmp_greater(probePoses.size(), _maxProbes)) {
        std::cerr << "Error in IGTLInput::setCurrentPose: too many probes." << std::endl;
        return;
    }

    for (int p = 0; std::cmp_less(p, probePoses.size()); p++) {
        TRTrans3D T = ~_centrePose * probePoses[p];  // Relative to origin used by IGTL client
        QList<float> matrix;
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                matrix.append(T(i,j));

        _trackingDataServer->SetElementTrackingData(p, matrix);
    }
#endif  // !WIN32
}

// Not implemented in IGTLController
/*void IGTLInput::setCurrentForce(const std::vector<double>& forceData)
{
}*/

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Disconnnect the IGTL and clear variables ready to reconnect
//----------------------------------------------------------------------
void IGTLInput::disconnectIGTL()
{
#ifndef WIN32
    if (!_trackingDataServer->IsRunning())
        return;
    disconnect(_trackingDataServer, nullptr, this, nullptr);
    _trackingDataServer->Stop();
#endif  // !WIN32

    _status = DISCONNECTED;
    emit connectionStatus("Disconnected");
    //emit connectionChanged();
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Handle new position requests
//----------------------------------------------------------------------
#ifndef WIN32
void IGTLInput::positionRequestReceived(igtl::TrackingDataMessage::Pointer position)
{
    std::cerr << "Position request received: " << position->GetNumberOfTrackingDataElements() << " elements." << std::endl;

    if (position->GetNumberOfTrackingDataElements() > _maxProbes) {
        std::cerr << "Error in IGTLInput::positionRequestReceived: number of posiions is more than the maximum number of probes." << std::endl;
        return;
    }

    igtl::TrackingDataElement::Pointer data;
    std::vector<TRTrans3D> requestedPoses(position->GetNumberOfTrackingDataElements());

    for (int el = 0; el < position->GetNumberOfTrackingDataElements(); el++) {
        position->GetTrackingDataElement(el, data);
        igtl::Matrix4x4 matrix;
        data->GetMatrix(matrix);

        TransMatrix3D m;
        for (int i=0; i<4; i++)
            for (int j=0; j<4; j++)
                m(i,j) = static_cast<double>(matrix[i][j]);
        TRTrans3D pose(m);
        //pose.printMatrix();

        if (pose.isInvalidMatrixError()) {
            for (int i = 0; i < _nDofs; i++)
                _poseData[el*_nDofs + i] = 0.0;
            return;
        }

        // Local copy
        double p[3], q[4];
        pose.getTranslation(p[0], p[1], p[2]);
        pose.getRotation(q[0], q[1], q[2], q[3]);
        _poseData[el*_nDofs + 0] = p[0];
        _poseData[el*_nDofs + 1] = p[1];
        _poseData[el*_nDofs + 2] = p[2];
        _poseData[el*_nDofs + 3] = q[0];
        _poseData[el*_nDofs + 4] = q[1];
        _poseData[el*_nDofs + 5] = q[2];
        _poseData[el*_nDofs + 6] = q[3];

        requestedPoses[el].set(_centrePose * pose);
        //requestedPoses[el].printMatrix();
    }

    emit multiProbePoseRequest(requestedPoses);
}
#endif  // !WIN32
