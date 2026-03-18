////////////////////////////////////////////////////////////////////////////////
/// @file   ArgonautRobotAutonomous3.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 3 for ArgonautRobot.
///
/// Copyright (c) 2026 Argonaut
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"                   // for DisplayMessage()
#include "ArgonautRobot.hpp"                // for robot class declaration
#include "ArgonautRobotAutonomous.hpp"      // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousRoutine3
///
/// Autonomous routine 3.
/// Rough auto for grabbing from the depot and dumping the fuel 
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousRoutine3()
{
    //drive backwards
    //lower intake 
    //turn on motors
    //drive back with those on for a few seconds
    //drive forward
    //angle towards hub
    //shoot
    
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 3 done.");
}
