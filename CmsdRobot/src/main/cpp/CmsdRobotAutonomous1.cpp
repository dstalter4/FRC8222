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
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    // We have to wait for the absolute encoders to stabilize
    while (!m_AbsoluteEncodersInitialized)
    {
        WaitForSensorConfig();
    }

    // Drive towards the reef (remember that the robot is facing the opposite alliance wall)
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.0, 2.5_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
