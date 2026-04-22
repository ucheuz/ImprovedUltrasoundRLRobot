#include <iostream>
#include "ExternalInputs.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
ExternalInputs::ExternalInputs(QObject *parent) : QObject(parent)
{
    _currentInput = NONE;

    _joystickInput = new JoystickInput(this);
#ifdef USE_INPUT_IGTL
    _igtlInput = new IGTLInput(this);
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    _spaceMouseInput = new SpaceMouseInput(this);
#endif  // USE_INPUT_SPACEMOUSE
    _surfPointsInput = new SurfacePointsInput(this);
    _controlPanelInput = new ControlPanelInput(this);
    
    //The nExpectedJoints should match the robot version.
    //For v3.4 this is 22, but a more flexible approach could be to pass the value
    //from RobotControl
    _rlInput = new RLInput(22, this);
}

ExternalInputs::~ExternalInputs()
{
    delete _joystickInput;
#ifdef USE_INPUT_IGTL
    delete _igtlInput;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    delete _spaceMouseInput;
#endif  // USE_INPUT_SPACEMOUSE
    delete _surfPointsInput;
    delete _controlPanelInput;
    delete _rlInput;
}

//----------------------------------------------------------------------
// Attempt to connect to all possible inputs
//----------------------------------------------------------------------
void ExternalInputs::connectInputs()
{
    _joystickInput->connectJoystick();
#ifdef USE_INPUT_IGTL
    _igtlInput->connectIGTL();
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    _spaceMouseInput->connectSpaceMouse();
#endif  // USE_INPUT_SPACEMOUSE
    _surfPointsInput->connectSurfPointsData();
    _controlPanelInput->connectControlPanel();
    //_rlInput->startListening(12345);  // Default port 12345
    _rlInput->startListening(_rlPort);

    emit connectionStatus(QString::number(nConnected()) + " input(s)");
}

//----------------------------------------------------------------------
// Get Arduino connection status
//----------------------------------------------------------------------
int ExternalInputs::nConnected() const
{
    int nInputs = 0;
    if (_joystickInput->isConnected())
        nInputs++;
#ifdef USE_INPUT_IGTL
    if (_igtlInput->isConnected())
        nInputs++;
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    if (_spaceMouseInput->isConnected())
        nInputs++;
#endif  // USE_INPUT_SPACEMOUSE
    if (_surfPointsInput->isConnected())
        nInputs++;
    if (_controlPanelInput->isConnected())
        nInputs++;
    if (_rlInput->isConnected())
        nInputs++;

    return nInputs;
}

//----------------------------------------------------------------------
// Select which input to listen to
//----------------------------------------------------------------------
void ExternalInputs::selectInput(Input_t input)
{
    _currentInput = input;
    disconnect(_joystickInput, nullptr, this, nullptr);
#ifdef USE_INPUT_IGTL
    disconnect(_igtlInput, nullptr, this, nullptr);
#endif  // USE_INPUT_IGTL
#ifdef USE_INPUT_SPACEMOUSE
    disconnect(_spaceMouseInput, nullptr, this, nullptr);
#endif  // USE_INPUT_SPACEMOUSE
    disconnect(_surfPointsInput, nullptr, this, nullptr);
    disconnect(_controlPanelInput, nullptr, this, nullptr);
    disconnect(_rlInput, nullptr, this, nullptr);

    // Forward pose requests from the appropriate input
    switch (_currentInput) {
    case JOYSTICK:
        std::cerr << "Listening to joystick" << std::endl;
        connect(_joystickInput, &JoystickInput::singleProbePoseRequest, this, &ExternalInputs::singleProbePoseRequest);
        connect(_joystickInput, &JoystickInput::multiProbePoseRequest, this, &ExternalInputs::multiProbePoseRequest);
        break;

    case IGTL:
#ifdef USE_INPUT_IGTL
        std::cerr << "Listening to IGTL" << std::endl;
        connect(_igtlInput, &IGTLInput::singleProbePoseRequest, this, &ExternalInputs::singleProbePoseRequest);
        connect(_igtlInput, &IGTLInput::multiProbePoseRequest, this, &ExternalInputs::multiProbePoseRequest);
#else
        std::cerr << "Ignoring all external inputs" << std::endl;
#endif
        break;

    case SPACE_MOUSE:
#ifdef USE_INPUT_SPACEMOUSE
        std::cerr << "Listening to SpaceMouse" << std::endl;
        connect(_spaceMouseInput, &SpaceMouseInput::poseAdjustRequest, this, &ExternalInputs::poseAdjustRequest);
#else
        std::cerr << "Ignoring all external inputs" << std::endl;
#endif  // USE_INPUT_SPACEMOUSE
        break;

    case SURF_POINT:
        std::cerr << "Listening to surface points input" << std::endl;
        connect(_surfPointsInput, &SurfacePointsInput::singleProbePoseRequest, this, &ExternalInputs::singleProbePoseRequest);
        connect(_surfPointsInput, &SurfacePointsInput::multiProbePoseRequest, this, &ExternalInputs::multiProbePoseRequest);
        break;

    case CONTROL_PANEL:
        std::cerr << "Listening to ControlPanel" << std::endl;
        connect(_controlPanelInput, &ControlPanelInput::poseAdjustRequest, this, &ExternalInputs::poseAdjustRequest);
        connect(_controlPanelInput, &ControlPanelInput::setActiveProbe, this, &ExternalInputs::setActiveProbe);
        connect(_controlPanelInput, &ControlPanelInput::cancel, this, &ExternalInputs::cancel);
        break;

    case RL_INPUT:
        std::cerr << "Listening to RL model input" << std::endl;
        connect(_rlInput, &RLInput::jointAnglesReceived, this, &ExternalInputs::jointAnglesFromRL);
        break;

    case NONE:
    case UNKNOWN:
    default:
        std::cerr << "Ignoring all external inputs" << std::endl;
        break;
    }
}

//----------------------------------------------------------------------
// Set the zero pose of each probe needed by some inputs
//----------------------------------------------------------------------
void ExternalInputs::setNeutralPoses(const std::vector<TRTrans3D>& neutralPoses)
{
    _joystickInput->setNeutralPoses(neutralPoses);
#ifdef USE_INPUT_IGTL
    //_igtlInput->setNeutralPoses(neutralPoses);
#endif  // USE_INPUT_IGTL
    _surfPointsInput->setNeutralPoses(neutralPoses);
}

//----------------------------------------------------------------------
// Set the centre (origin) pose used by some inputs
//----------------------------------------------------------------------
void ExternalInputs::setCentrePose(const TRTrans3D& centrePose)
{
    //_joystickInput->setCentrePose(centrePose);
#ifdef USE_INPUT_IGTL
    _igtlInput->setCentrePose(centrePose);
#endif  // USE_INPUT_IGTL
}

//----------------------------------------------------------------------
// Send current information to all inputs that want it
//----------------------------------------------------------------------
void ExternalInputs::setCurrentPoses(const std::vector<TRTrans3D>& probePoses)
{
#ifdef USE_INPUT_IGTL
    _igtlInput->setCurrentPoses(probePoses);
#endif  // USE_INPUT_IGTL
}

void ExternalInputs::setCurrentForce(const std::vector<double>& forceData)
{
    _controlPanelInput->setCurrentForces(forceData);
    _joystickInput->setCurrentForce(forceData);
#ifdef USE_INPUT_IGTL
    //_igtlInput->setCurrentForce(forceData);  // Not implemented in IGTLController
#endif  // USE_INPUT_IGTL
    if (_rlInput != nullptr && _rlInput->isConnected())
        _rlInput->sendCurrentForces(forceData);
}

void ExternalInputs::setCurrentDistances(const std::vector<double>& distData)
{
    // Currently no external sources listen to distance data, so this function does nothing with the data.
}

void ExternalInputs::setCurrentJointPositions(const std::vector<double>& jointPositions)
{
    if (_rlInput != nullptr && _rlInput->isConnected())
        _rlInput->sendCurrentJointPositions(jointPositions);
}

//----------------------------------------------------------------------
// Input access for GUI
//----------------------------------------------------------------------
JoystickInput* ExternalInputs::getJoystickInputPtr() const
{
    return _joystickInput;
}

#ifdef USE_INPUT_IGTL
IGTLInput* ExternalInputs::getIGTLInputPtr() const
{
    return _igtlInput;
}
#endif  // USE_INPUT_IGTL

#ifdef USE_INPUT_SPACEMOUSE
SpaceMouseInput* ExternalInputs::getSpaceMouseInputPtr() const
{
    return _spaceMouseInput;
}
#endif  // USE_INPUT_SPACEMOUSE

SurfacePointsInput* ExternalInputs::getSurfPointsInputPtr() const
{
    return _surfPointsInput;
}

ControlPanelInput* ExternalInputs::getControlPanelInputPtr() const
{
    return _controlPanelInput;
}

RLInput* ExternalInputs::getRLInputPtr() const
{
    return _rlInput;
}
