#include <iostream>

#include "IMotorControl.h"

#include <QDomDocument>
#include <QDir>
#include "xmlUtils.h"  // XmlUtils library

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructor and destructor
//----------------------------------------------------------------------
IMotorControl::IMotorControl()
{
    _status = DISCONNECTED;

    _moving = false;
    _speedScale = 1.0;

    _cancelSequence = false;
}

IMotorControl::~IMotorControl()
{
}

//----------------------------------------------------------------------
// Get connection status
//----------------------------------------------------------------------
bool IMotorControl::isConnected() const
{
    return _status == CONNECTED;
}

//----------------------------------------------------------------------
// Get number of motors
//----------------------------------------------------------------------
int IMotorControl::getNMotors() const
{
    return _nMotors;
}

//----------------------------------------------------------------------
// Set the angles that will be the neutral positions for each motor
//----------------------------------------------------------------------
void IMotorControl::setNeutralPositions(std::vector<double>& neutralPositions)
{
    _neutralPos.resize(_nMotors, 0.0);

    for (unsigned int i = 0; i < _nMotors; i++)
        _neutralPos[i] = neutralPositions[i];
}

//----------------------------------------------------------------------
// Change speed scale value
//----------------------------------------------------------------------
void IMotorControl::setSpeedScale(double value)
{
    if (value > 1.0)
        value = 1.0;
    if (value < 0.1)
        value = 0.1;
    _speedScale = value;
}

//----------------------------------------------------------------------
// Check whether the motor has a homing sensor
//----------------------------------------------------------------------
bool IMotorControl::isHomeable(unsigned int idx) const
{
    if (idx >= _nMotors) {
        std::cerr << "Error in IMotorControl::isHomeable: index is out of range." << std::endl;
        return false;
    }

    return _homingSensor[idx];
}

//----------------------------------------------------------------------
// Check whether motors are running
//----------------------------------------------------------------------
bool IMotorControl::isIdle() const
{
    return !_moving;
}

//======================================================================
// Public slots
//======================================================================
//----------------------------------------------------------------------
// Motor finished slot
//----------------------------------------------------------------------
void IMotorControl::goIdle()
{
    _moving = false;
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Initialise variables from xml file for the specified robot
//----------------------------------------------------------------------
void IMotorControl::initialise(const QString& robotId)
{
    QDomDocument doc("robotInfo");
    bool xmlParseSuccess = true;
    xmlParseSuccess &= XmlUtils::extractFileToDoc(QDir::currentPath() + "/Resources/robotInfo.xml", doc);

    QDomElement robotEl, motorsEl;
    xmlParseSuccess &= XmlUtils::getElementFromDoc(doc, "Robot", robotEl, "version", robotId);
    xmlParseSuccess &= XmlUtils::getElementFromElement(robotEl, "Motors", motorsEl);

    xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "motorArduinoId", _arduinoIds);
    xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "nMotors", _nMotors);

    if (_nMotors > 0) {
        // Steps per degree for each motor
        bool xmlParseSuccess2 = XmlUtils::readValFromElement(motorsEl, "stepsPerDegree", _stepsPerDegree);
        xmlParseSuccess2 &= _stepsPerDegree.size() == _nMotors;
        if (!xmlParseSuccess2) {
            // Look for individual parameters of the motor
            std::vector<double> stepsPerRev, microStepping, internalGearRatio, externalGearRatio;
            xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "stepsPerRevolution", stepsPerRev);
            xmlParseSuccess &= stepsPerRev.size() == _nMotors;
            xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "microstepping", microStepping);
            xmlParseSuccess &= microStepping.size() == _nMotors;
            xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "internalGearRatio", internalGearRatio);
            xmlParseSuccess &= internalGearRatio.size() == _nMotors;
            xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "externalGearRatio", externalGearRatio);
            xmlParseSuccess &= externalGearRatio.size() == _nMotors;
            _stepsPerDegree.resize(_nMotors, 1.0);
            if (xmlParseSuccess)
                for (int i = 0; i < _nMotors; i++)
                    _stepsPerDegree[i] = (stepsPerRev[i] * microStepping[i] * internalGearRatio[i] / 360.0) * externalGearRatio[i];
        }

        // Homing position for each motor
        xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "homingPosition", _homePos);
        xmlParseSuccess &= _homePos.size() == _nMotors;
        xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "homingSensor", _homingSensor);
        xmlParseSuccess &= _homingSensor.size() == _nMotors;
        xmlParseSuccess &= XmlUtils::readValFromElement(motorsEl, "homingSequence", _homingOrder);
        xmlParseSuccess &= _homingOrder.size() == _nMotors;
    }

    if (!xmlParseSuccess) {
        _status = INVALID_SETUP;
        std::cerr << "Error in IMotorControl: failed to read valid motor specification from xml file." << std::endl;
    }
}
