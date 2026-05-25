////////////////////////////////////////////////////////////////////////////////
/// @file   ArgonautRobotAutonomous.cpp
/// @author David Stalter
///
/// @details
/// Implementation of autonomous routines for ArgonautRobot.
///
/// Copyright (c) 2026 Argonaut
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <frc2/command/CommandScheduler.h>      // for scheduling commands

// C INCLUDES
// (none)

// C++ INCLUDES
#include "ArgonautRobot.hpp"                // for robot class declaration
#include "ArgonautRobotAutonomous.hpp"      // for autonomous declarations
#include "RobotUtils.hpp"                       // for DisplayMessage()

// NAMESPACE DATA
bool ArgonautRobotAutonomous::bAutonomousExecutionComplete;
std::optional<CommandPtr> AutonomousCommand;


////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousInit
///
/// The autonomous init method.  This method is called once each
/// time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousInit()
{
    RobotUtils::DisplayMessage("AutonomousInit called.");
    
    // Put everything in a stable state
    InitialStateSetup();
    
    // Indicate the autonomous routine has not executed yet
    ArgonautRobotAutonomous::bAutonomousExecutionComplete = false;

    if (ArgonautRobotAutonomous::USE_COMMAND_BASED_AUTONOMOUS)
    {
        RobotUtils::DisplayMessage("Autonomous init - command based.");

        // Scheduled commands must have non-local scope!
        // Otherwise the scheduler does not continue to see them.
        AutonomousCommand = AutonomousTestCommandDashboardRoutine();
        //AutonomousCommand = AutonomousTestCommandMotionRoutine();
        //AutonomousCommand = AutonomousTestTrajectoryRoutine();

        if (AutonomousCommand.has_value())
        {
            RobotUtils::DisplayMessage("Autonomous init - command scheduled.");
            CommandScheduler::GetInstance().Schedule(AutonomousCommand.value());
        }
        else
        {
            RobotUtils::DisplayMessage("Autonomous init - command NOT scheduled.");
        }
    }
    else
    {
        RobotUtils::DisplayMessage("Autonomous init - time based.");
    }
}



////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousPeriodic
///
/// The autonomous control method.  This method is called
/// periodically while the robot is in autonomous control.
/// Even though this method wil be called periodically, it
/// deliberately checks the driver station state controls.
/// This is to give finer control over the autonomous state
/// machine flow.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_AUTONOMOUS);

    if (ArgonautRobotAutonomous::USE_COMMAND_BASED_AUTONOMOUS)
    {
        AutonomousPeriodicCommand();
    }
    else
    {
        AutonomousPeriodicTimed();
    }
}



////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousPeriodicCommand
///
/// Main workflow for command based autonomous routines.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousPeriodicCommand()
{
    // Update the swerve odometry and run the command scheduler
    // @todo: The odometry updates may be causing drift
    m_pSwerveDrive->UpdateOdometry();
    CommandScheduler::GetInstance().Run();
}



////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousPeriodicTimed
///
/// Main workflow for time based autonomous routines.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousPeriodicTimed()
{    
    if (ArgonautRobotAutonomous::bAutonomousExecutionComplete)
    {
        return;
    }
    
    // @todo: Figure out how to kick the watchdog from here so it doesn't overrun.
    // @note: Since autonomous is configured as a one shot state machine, there will definitely be watchdog overrun.
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    // Get the selected autonomous routine from the smart dashboard
    std::string selectedAutoRoutineString = m_AutonomousChooser.GetSelected();
    
    // Auto routine 1
    //if ( ArgonautRobotAutonomous::ROUTINE_1 )
    if (selectedAutoRoutineString == AUTO_ROUTINE_1_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 1.");
        AutonomousRoutine1();
    }
    
    // Auto routine 2
    //else if ( ArgonautRobotAutonomous::ROUTINE_2 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_2_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 2.");
        AutonomousRoutine2();
    }
    
    // Auto routine 3
    //else if ( ArgonautRobotAutonomous::ROUTINE_3 )
    else if (selectedAutoRoutineString == AUTO_ROUTINE_3_STRING)
    {
        RobotUtils::DisplayMessage("Auto routine 3.");
        AutonomousRoutine3();
    }
    
    // No autonomous routine
    else if (selectedAutoRoutineString == AUTO_NO_ROUTINE_STRING)
    {
        RobotUtils::DisplayMessage("No autonomous routine.");
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    //else if ( ArgonautRobotAutonomous::TEST_ENABLED )
    else if (selectedAutoRoutineString == AUTO_TEST_ROUTINE_STRING)
    {
        RobotUtils::DisplayMessage("Auto test code.");
        AutonomousTestRoutine();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        RobotUtils::DisplayMessage("No auto selection made, going idle.");
    }
    
    // One shot through autonomous is over, indicate as such.
    ArgonautRobotAutonomous::bAutonomousExecutionComplete = true;
    
    /*
    // Idle until auto is terminated
    RobotUtils::DisplayMessage("Auto idle loop.");
    while ( m_pDriverStation->IsAutonomous() && m_pDriverStation->IsEnabled() )
    {
    }
    */
}



////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousCommon
///
/// Common autonomous behavior.  It moves away from the alliance
/// wall and to the fuel loading station.  The variance is
/// whether it shoots at the start or at the end.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousCommon()
{

    if (m_AllianceColor == DriverStation::Alliance::kRed)
    {
        AutonomousCommonRed();
    }
    else if (m_AllianceColor == DriverStation::Alliance::kBlue)
    {
        AutonomousCommonBlue();
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
/// @method ArgonautRobot::AutonomousCommonRed
///
/// Common autonomous behavior when on the red alliance.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousCommonRed()
{
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///                           Red/Blue Separation                            ///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
// @method ArgonautRobot::AutonomousCommonBlue
///
/// Common autonomous behavior when on the blue alliance.
///
////////////////////////////////////////////////////////////////
void ArgonautRobot::AutonomousCommonBlue()
{
}
