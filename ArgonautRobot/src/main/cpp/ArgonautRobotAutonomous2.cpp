////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for CmsdRobot.
///
/// Copyright (c) 2024 CMSD
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "RobotUtils.hpp"               // for DisplayMessage()
#include "CmsdRobot.hpp"                // for robot class declaration
#include "CmsdRobotAutonomous.hpp"      // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousRoutine2()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
