////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdRobotAutonomous1.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 1 for CmsdRobot.
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
/// @method CmsdRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousRoutine1()
{
    // We have to wait for the absolute encoders to stabilize
    while (!m_AbsoluteEncodersInitialized)
    {
        WaitForSensorConfig();
    }

    AutonomousSwerveDriveSequence(ROBOT_FORWARD, ROBOT_NO_ROTATE, 0.15, 0.0, 0.0, 3.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
