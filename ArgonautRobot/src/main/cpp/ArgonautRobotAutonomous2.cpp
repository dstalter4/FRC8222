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
#include "RobotCamera.hpp"                  // for AlignToTargetSwerve()
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
    // The robot faces the driver station, so it is off by 180 degrees
    m_pPigeon->SetYaw(units::angle::degree_t(ANGLE_180_DEGREES));

    // Intake down
    m_pIntakePivot->SetPositionVoltage(INTAKE_DOWN_POSITION_DEGREES.value());

    //drive BACKWARDS PLS
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.25, 0.0, 0.0, 2.0_s, true);
    // Slow down into the driver station wall to minimize hopper impact
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 2.0_s, true);

    // Wait for the human players
    AutonomousDelay(2.0_s);

    //shooter ramp up
    m_pShooterMotors->Set(SHOOTER_MOTOR_SPEED);

    // Move to get in line with shooting
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.10, 0.14, 2.0_s, true);

    // Use the camera to finish centering
    for (units::time::second_t t = 0.0_s; t < 3.0_s; t += 20.0_ms)
    {
        RobotCamera::AutonomousCamera::AlignToTargetSwerve(m_pPigeon->GetYaw().GetValueAsDouble());
        AutonomousDelay(20.0_ms);
    }

    //dump
    m_pHopperFeed->SetDutyCycle(HOPPER_FEED_MOTOR_SPEED);
    m_pShooterFeed->SetDutyCycle(SHOOTER_FEED_MOTOR_SPEED);
    AutonomousDelay(3.0_s);
    
    // Intake back up
    m_pIntakePivot->SetPositionVoltage(INTAKE_UP_POSITION_DEGREES.value());
    AutonomousDelay(2.0_s);

    //everything off
    m_pShooterMotors->Set(0.0);
    m_pHopperFeed->SetDutyCycle(0.0);
    m_pShooterFeed->SetDutyCycle(0.0);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto routine 2 done.");
}
