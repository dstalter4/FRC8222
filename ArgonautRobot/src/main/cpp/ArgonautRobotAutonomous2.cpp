////////////////////////////////////////////////////////////////////////////////
/// @file   ArgonautRobotAutonomous2.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routine 2 for ArgonautRobot.
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
/// @method ArgonautRobot::AutonomousRoutine2
///
/// Autonomous routine 2.
/// Another rough auto for grabbing fuel from the human player station and dumping
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousRoutine2()
{
    // Drop intake 

    // drive forward
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.0, 0.2, 2.5_s, true);
    
    //wait for fuel to be dumped 
    AutonomousDelay(2.0_s);

    // spin up
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);
    AutonomousDelay(2.0_s);

    // shoot
    m_pHopperFeed->SetDutyCycle(HOPPER_FEED_MOTOR_SPEED);
    m_pShooterFeed->SetDutyCycle(SHOOTER_FEED_MOTOR_SPEED);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
