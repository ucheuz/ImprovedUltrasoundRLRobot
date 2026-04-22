#include <iostream>

#include "RLInput.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
RLInput::RLInput(int nExpectedJoints, QObject* parent) : QObject(parent)
{
    _server = new QTcpServer(this);
    _clientSocket = nullptr;
    _status = DISCONNECTED;
    _nExpectedJoints = nExpectedJoints;

    connect(_server, &QTcpServer::newConnection, this, &RLInput::onNewConnection);
}

RLInput::~RLInput()
{
    stopListening();
}

//----------------------------------------------------------------------
// Start TCP server listening on the given port
//----------------------------------------------------------------------
void RLInput::startListening(quint16 port)
{
    if (_server->isListening())
        stopListening();

    if (_server->listen(QHostAddress::Any, port)) {
        _status = LISTENING;
        std::cerr << "RLInput: Listening for RL model on port " << port << std::endl;
        emit connectionStatus(QString("Listening on port %1").arg(port));
    }
    else {
        _status = CONNECT_FAILED;
        std::cerr << "RLInput: Failed to start server on port " << port
                  << " - " << _server->errorString().toStdString() << std::endl;
        emit connectionStatus("Listen failed");
    }
}

//----------------------------------------------------------------------
// Stop the server and disconnect client
//----------------------------------------------------------------------
void RLInput::stopListening()
{
    if (_clientSocket != nullptr) {
        disconnect(_clientSocket, nullptr, this, nullptr);
        _clientSocket->disconnectFromHost();
        _clientSocket = nullptr;
    }

    if (_server->isListening())
        _server->close();

    _status = DISCONNECTED;
    _receiveBuffer.clear();
    emit connectionStatus("Disconnected");
}

//----------------------------------------------------------------------
// Connection status
//----------------------------------------------------------------------
bool RLInput::isConnected() const
{
    return _clientSocket != nullptr && _clientSocket->state() == QAbstractSocket::ConnectedState;
}

//----------------------------------------------------------------------
// Send current joint positions back to the RL model as feedback
// Format: {"type":"joint_positions","data":[0.0, 90.0, ...]}
//----------------------------------------------------------------------
void RLInput::sendCurrentJointPositions(const std::vector<double>& jointPositions)
{
    QJsonObject obj;
    obj["type"] = "joint_positions";
    QJsonArray arr;
    for (double val : jointPositions)
        arr.append(val);
    obj["data"] = arr;

    QJsonDocument doc(obj);
    sendJson(doc.toJson(QJsonDocument::Compact) + "\n");
}

//----------------------------------------------------------------------
// Send current forces back to the RL model as feedback
// Format: {"type":"forces","data":[0.1, 0.2, ...]}
//----------------------------------------------------------------------
void RLInput::sendCurrentForces(const std::vector<double>& forces)
{
    QJsonObject obj;
    obj["type"] = "forces";
    QJsonArray arr;
    for (double val : forces)
        arr.append(val);
    obj["data"] = arr;

    QJsonDocument doc(obj);
    sendJson(doc.toJson(QJsonDocument::Compact) + "\n");
}

//======================================================================
// Private slots
//======================================================================
//----------------------------------------------------------------------
// Handle a new incoming TCP connection
//----------------------------------------------------------------------
void RLInput::onNewConnection()
{
    // Accept only one client at a time
    QTcpSocket* newSocket = _server->nextPendingConnection();
    if (newSocket == nullptr)
        return;

    if (_clientSocket != nullptr) {
        // Already have a client; reject the new one
        std::cerr << "RLInput: Rejecting additional connection (already have a client)" << std::endl;
        newSocket->disconnectFromHost();
        newSocket->deleteLater();
        return;
    }

    _clientSocket = newSocket;
    _receiveBuffer.clear();
    _status = CONNECTED;

    connect(_clientSocket, &QTcpSocket::readyRead, this, &RLInput::onDataReceived);
    connect(_clientSocket, &QTcpSocket::disconnected, this, &RLInput::onClientDisconnected);

    std::cerr << "RLInput: RL model connected from "
              << _clientSocket->peerAddress().toString().toStdString()
              << ":" << _clientSocket->peerPort() << std::endl;
    emit connectionStatus("RL model connected");
}

//----------------------------------------------------------------------
// Handle incoming data from the RL model client
// Messages are newline-delimited JSON.
//----------------------------------------------------------------------
void RLInput::onDataReceived()
{
    if (_clientSocket == nullptr)
        return;

    _receiveBuffer.append(_clientSocket->readAll());

    // Process all complete messages (newline-delimited)
    while (true) {
        int newlineIdx = _receiveBuffer.indexOf('\n');
        if (newlineIdx < 0)
            break;

        QByteArray message = _receiveBuffer.left(newlineIdx).trimmed();
        _receiveBuffer.remove(0, newlineIdx + 1);

        if (message.isEmpty())
            continue;

        std::vector<double> jointAngles;
        if (parseMessage(message, jointAngles)) {
            emit jointAnglesReceived(jointAngles);
        }
        else {
            std::cerr << "RLInput: Failed to parse message: " << message.toStdString() << std::endl;
        }
    }
}

//----------------------------------------------------------------------
// Handle client disconnection
//----------------------------------------------------------------------
void RLInput::onClientDisconnected()
{
    std::cerr << "RLInput: RL model disconnected" << std::endl;

    if (_clientSocket != nullptr) {
        disconnect(_clientSocket, nullptr, this, nullptr);
        _clientSocket->deleteLater();
        _clientSocket = nullptr;
    }

    _receiveBuffer.clear();

    _status = _server->isListening() ? LISTENING : DISCONNECTED;
    emit connectionStatus(_status == LISTENING ? "Listening (client disconnected)" : "Disconnected");
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Parse a JSON message and extract joint angles
// Expected format: {"joint_angles": [0.0, 90.0, 13.33, ...]}
// Returns true if parsing succeeded and array has the expected size.
//----------------------------------------------------------------------
bool RLInput::parseMessage(const QByteArray& message, std::vector<double>& jointAngles)
{
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(message, &parseError);

    if (parseError.error != QJsonParseError::NoError) {
        std::cerr << "RLInput: JSON parse error: " << parseError.errorString().toStdString() << std::endl;
        return false;
    }

    if (!doc.isObject())
        return false;

    QJsonObject obj = doc.object();

    if (!obj.contains("joint_angles") || !obj["joint_angles"].isArray())
        return false;

    QJsonArray arr = obj["joint_angles"].toArray();

    if (arr.size() != _nExpectedJoints) {
        std::cerr << "RLInput: Expected " << _nExpectedJoints
                  << " joint angles, received " << arr.size() << std::endl;
        return false;
    }

    jointAngles.resize(arr.size());
    for (int i = 0; i < arr.size(); i++) {
        if (!arr[i].isDouble())
            return false;
        jointAngles[i] = arr[i].toDouble();
    }

    return true;
}

//----------------------------------------------------------------------
// Send a JSON string to the connected client
//----------------------------------------------------------------------
void RLInput::sendJson(const QByteArray& json)
{
    if (_clientSocket == nullptr || _clientSocket->state() != QAbstractSocket::ConnectedState)
        return;

    _clientSocket->write(json);
    _clientSocket->flush();
}
