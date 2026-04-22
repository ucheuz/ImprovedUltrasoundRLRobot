#ifndef RLINPUT_H
#define RLINPUT_H

#include <vector>

#include <QObject>
#include <QString>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

//======================================================================
// RLInput class receives joint angle commands from an external
// Reinforcement Learning model via a TCP socket. The RL model sends
// a JSON array of joint angles (in degrees, kinematics order), and
// this class parses them and emits a signal carrying the raw joint
// angle vector. This signal bypasses inverse kinematics entirely.
//
// Protocol:  The RL client connects to the TCP server and sends
//            newline-terminated JSON messages of the form:
//            {"joint_angles": [0.0, 90.0, 13.33, ...]}  (22 values for V3.4)
//
// The class also provides a feedback channel: current joint positions
// and sensor data can be sent back to the RL client as JSON.
//======================================================================
class RLInput : public QObject
{
    Q_OBJECT

public:
    explicit RLInput(int nExpectedJoints, QObject* parent = nullptr);
    ~RLInput();

    enum ConnectionStatus_t {DISCONNECTED=0, LISTENING, CONNECTED, CONNECT_FAILED};

    /// Start listening for incoming RL model connections
    void startListening(quint16 port = 12345);

    /// Stop the server and disconnect any client
    void stopListening();

    bool isConnected() const;

    // --- Feedback to the RL model ---
    void sendCurrentJointPositions(const std::vector<double>& jointPositions);
    void sendCurrentForces(const std::vector<double>& forces);

signals:
    /// Emitted when a valid set of joint angles is received from the RL model.
    /// The vector is in kinematics order (22 elements for robot V3.4).
    void jointAnglesReceived(const std::vector<double>& jointAngles);

    void connectionStatus(QString status);

private slots:
    void onNewConnection();
    void onDataReceived();
    void onClientDisconnected();

private:
    QTcpServer* _server;
    QTcpSocket* _clientSocket;

    ConnectionStatus_t _status;
    int _nExpectedJoints;

    QByteArray _receiveBuffer;

    bool parseMessage(const QByteArray& message, std::vector<double>& jointAngles);
    void sendJson(const QByteArray& json);
};

#endif // RLINPUT_H
