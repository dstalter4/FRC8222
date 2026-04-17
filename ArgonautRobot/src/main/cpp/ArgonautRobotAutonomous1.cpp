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
#include "RobotCamera.hpp"                  // for AlignToTargetSwerve()
#include "RobotUtils.hpp"                   // for DisplayMessage()
#include "ArgonautRobot.hpp"                // for robot class declaration
#include "ArgonautRobotAutonomous.hpp"      // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
/// Back up from against the hub and shoot the eight pieces of fuel
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousRoutine1()
{
    //drive BACKWARDS PLS
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.15, 0.0, 0.0, 2.5_s, true);
    //shooter ramp up
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);
    AutonomousDelay(2.0_s);
    //dump
    m_pHopperFeed->SetDutyCycle(HOPPER_FEED_MOTOR_SPEED);
    m_pShooterFeed->SetDutyCycle(SHOOTER_FEED_MOTOR_SPEED);
    AutonomousDelay(5.0_s);
    //everything off
    m_pShooterMotors->Set(0.0);
    // m_pHopperFeed->SetDutyCycle(0.0);
    m_pShooterFeed->SetDutyCycle(0.0);

    // This is currently just for debug info
    for (units::time::second_t t = 0.0_s; t < 3.0_s; t += 20.0_ms)
    {
        RobotCamera::AutonomousCamera::AlignToTargetSwerve(m_pPigeon->GetYaw().GetValueAsDouble());
        AutonomousDelay(20.0_ms);
    }

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 1 done.");
}
