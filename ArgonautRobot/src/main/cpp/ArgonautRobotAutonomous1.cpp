////////////////////////////////////////////////////////////////////////////////
/// @file   ArgonautRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for ArgonautRobot.
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
/// @method ArgonautRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousRoutine1()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
