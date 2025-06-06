////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdRobotAutonomous.hpp
/// @author David Stalter
///
/// @details
/// Contains the declarations for the autonomous portions of code ran in an FRC
/// robot.
///
/// Copyright (c) CMSD
////////////////////////////////////////////////////////////////////////////////

#ifndef CMSDROBOTAUTONOMOUS_HPP
#define CMSDROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"                // for inline autonomous function declarations

using namespace frc;

////////////////////////////////////////////////////////////////
/// @namespace CmsdRobotAutonomous
///
/// Namespace that contains robot autonomous variable and
/// function declarations.
///
////////////////////////////////////////////////////////////////
namespace CmsdRobotAutonomous
{
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // VARIABLES
    extern bool bAutonomousExecutionComplete;
    
    // CONSTS
    
    // Autonomous Mode Constants
    // @todo: Convert to class and make a friend in CmsdRobot
    
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines are currently controlled by
    // the SendableChooser.
    //static const bool       ROUTINE_1                           = true;
    //static const bool       ROUTINE_2                           = false;
    //static const bool       ROUTINE_3                           = false;
    //static const bool       TEST_ENABLED                        = false;

    // Autonomous drive speed constants
    // (none)
    
    // Autonomous angle constants
    static const int        FORTY_FIVE_DEGREES                  = 45;
    static const int        NINETY_DEGREES                      = 90;
    static const int        ONE_HUNDRED_EIGHTY_DEGREES          = 180;
    static const int        THREE_HUNDRED_SIXTY_DEGREES         = 360;
    
    // Autonomous delay constants
    static constexpr units::second_t SWERVE_OP_STEP_TIME_S      =  0.10_s;
    static constexpr units::second_t DELAY_SHORT_S              =  0.50_s;
    static constexpr units::second_t DELAY_MEDIUM_S             =  1.00_s;
    static constexpr units::second_t DELAY_LONG_S               =  2.00_s;
    
} // End namespace



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousDelay(units::second_t time)
{
    Wait(time);
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousSwerveDriveSequence
///
/// Drives during autonomous for a specified amount of time
/// using swerve drive modules.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousSwerveDriveSequence(RobotSwerveDirections & rSwerveDirections, double translationSpeed, double strafeSpeed, double rotateSpeed, units::second_t time, bool bFieldRelative)
{
    units::meter_t translation = 0.0_m;
    units::meter_t strafe = 0.0_m;

    switch (rSwerveDirections.GetTranslation())
    {
        case RobotTranslation::ROBOT_TRANSLATION_FORWARD:
        {
            translation = units::meter_t(translationSpeed);
            break;
        }
        case RobotTranslation::ROBOT_TRANSLATION_REVERSE:
        {
            translation = units::meter_t(-translationSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (rSwerveDirections.GetStrafe())
    {
        case RobotStrafe::ROBOT_STRAFE_LEFT:
        {
            strafe = units::meter_t(strafeSpeed);
            break;
        }
        case RobotStrafe::ROBOT_STRAFE_RIGHT:
        {
            strafe = units::meter_t(-strafeSpeed);
            break;
        }
        default:
        {
            break;
        }
    }

    switch (rSwerveDirections.GetRotation())
    {
        case RobotRotation::ROBOT_NO_ROTATION:
        {
            // Just in case the user decided to pass a speed anyway
            rotateSpeed = 0.0;
            break;
        }
        case RobotRotation::ROBOT_CLOCKWISE:
        {
            rotateSpeed *= -1.0;
            break;
        }
        case RobotRotation::ROBOT_COUNTER_CLOCKWISE:
        default:
        {
            break;
        }
    }

    Translation2d translation2d = {translation, strafe};
    units::second_t duration = 0.0_s;
    while (duration < time)
    {
        m_pSwerveDrive->SetModuleStates(translation2d, rotateSpeed, bFieldRelative, true);
        AutonomousDelay(CmsdRobotAutonomous::SWERVE_OP_STEP_TIME_S);
        duration += CmsdRobotAutonomous::SWERVE_OP_STEP_TIME_S;
    }

    // Stop motion
    m_pSwerveDrive->SetModuleStates({0_m, 0_m}, 0.0, true, true);

    // Clear the swerve directions to prevent the caller from
    // accidentally reusing them without explicitly setting them again.
    rSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousRotateByGyroSequence
///
/// Turns the robot by the gyro.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousRotateByGyroSequence(RobotRotation robotRotation, double rotateDegrees, double rotateSpeed, bool bFieldRelative)
{
    double startingGyroAngle = m_pPigeon->GetYaw().GetValueAsDouble();

    while (std::abs(m_pPigeon->GetYaw().GetValueAsDouble() - startingGyroAngle) <= rotateDegrees)
    {
        if (robotRotation == RobotRotation::ROBOT_CLOCKWISE)
        {
            m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, rotateSpeed, bFieldRelative, true);
        }
        else if (robotRotation == RobotRotation::ROBOT_COUNTER_CLOCKWISE)
        {
            m_pSwerveDrive->SetModuleStates({0.0_m, 0.0_m}, -rotateSpeed, bFieldRelative, true);
        }
        else
        {
        }
    }

    // Stop motion
    m_pSwerveDrive->SetModuleStates({0_m, 0_m}, 0.0, true, true);
}


#endif // CMSDROBOTAUTONOMOUS_HPP
