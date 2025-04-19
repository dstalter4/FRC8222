////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdRobotAutonomousTest.cpp
/// @author David Stalter
///
/// @details
/// Implementation of an autonomous test routines for CmsdRobot.
///
/// Copyright (c) 2024 CMSD
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Pose2d.h"                    // for type declaration
#include "frc/geometry/Rotation2d.h"                // for type declaration
#include "frc/geometry/Translation2d.h"             // for type declaration
#include "frc/trajectory/Trajectory.h"              // for working with trajectories
#include "frc/trajectory/TrajectoryConfig.h"        // for creating a trajectory config
#include "frc/trajectory/TrajectoryGenerator.h"     // for generating a trajectory

// C++ INCLUDES
#include "RobotUtils.hpp"                           // for DisplayMessage()
#include "CmsdRobot.hpp"                            // for robot class declaration
#include "CmsdRobotAutonomous.hpp"                  // for autonomous declarations


////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousTestRoutine
///
/// Autonomous test routine.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousTestRoutine()
{
    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test routine done.");
}


////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousTestSwerveRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousTestSwerveRoutine()
{
    // Simple demonstration of directional movements
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_NO_ROTATION);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.0, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_NO_TRANSLATION, RobotStrafe::ROBOT_NO_STRAFE, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.0, 0.10, 0.0, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_FORWARD, RobotStrafe::ROBOT_STRAFE_RIGHT, RobotRotation::ROBOT_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);
    m_AutoSwerveDirections.SetSwerveDirections(RobotTranslation::ROBOT_TRANSLATION_REVERSE, RobotStrafe::ROBOT_STRAFE_LEFT, RobotRotation::ROBOT_COUNTER_CLOCKWISE);
    AutonomousSwerveDriveSequence(m_AutoSwerveDirections, 0.10, 0.10, 0.10, 1.0_s, true);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test swerve routine done.");
}


////////////////////////////////////////////////////////////////
/// @method CmsdRobot::AutonomousTestTrajectoryRoutine
///
/// Autonomous swerve test routine.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousTestTrajectoryRoutine()
{
    // Swerve trajectory routine, but requires switching to command based robot.

    //TrajectoryConfig config =
    //new TrajectoryConfig(
    //        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //    .setKinematics(Constants.Swerve.swerveKinematics);
    TrajectoryConfig trajectoryConfig = {0.5_mps, 1.0_mps_sq};
    trajectoryConfig.SetKinematics(SwerveConfig::Kinematics);

    // An example trajectory to follow.  All units in meters.
    //Trajectory exampleTrajectory =
    //TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        //new Pose2d(3, 0, new Rotation2d(0)),
        //config);
    const Pose2d INITIAL_POSE = {0_m, 0_m, 0_deg};
    const Pose2d FINAL_POSE = {2_m, 0_m, 0_deg};
    const std::vector<Translation2d> WAY_POINTS = 
    {
        {0_m, 0_m}
    };
    Trajectory testTrajectory = TrajectoryGenerator::GenerateTrajectory(INITIAL_POSE, WAY_POINTS, FINAL_POSE, trajectoryConfig);
    (void)testTrajectory;

    //var thetaController =
    //new ProfiledPIDController(
    //    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    //thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //SwerveControllerCommand swerveControllerCommand =
    //new SwerveControllerCommand(
    //    exampleTrajectory,
    //    s_Swerve::getPose,
    //    Constants.Swerve.swerveKinematics,
    //    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //    thetaController,
    //    s_Swerve::setModuleStates,
    //    s_Swerve);

    // Returning from here will enter the idle state until autonomous is over
    RobotUtils::DisplayMessage("Auto test trajectory routine done.");
}
