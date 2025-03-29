////////////////////////////////////////////////////////////////////////////////
/// @file   SwerveDrive.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for a swerve drive robot base.
///
/// Copyright (c) 2025 CMSD
////////////////////////////////////////////////////////////////////////////////

#ifndef SWERVEDRIVE_HPP
#define SWERVEDRIVE_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/geometry/Translation2d.h"                 // for class declaration
#include "frc/kinematics/SwerveDriveOdometry.h"         // for class declaration
#include "frc/kinematics/SwerveModulePosition.h"        // for struct declaration
#include "frc/kinematics/SwerveModuleState.h"           // for struct declaration
#include "units/angle.h"                                // for angle user defined literals
#include "units/angular_velocity.h"                     // for angular velocity user defined literals
#include "units/length.h"                               // for distance user defined literals

// C++ INCLUDES
#include "SwerveConfig.hpp"                             // for swerve configuration and constants
#ifdef USE_NEO_SWERVE
  #include "NeoSwerveModule.hpp"                        // for interacting with a Neo swerve module
  typedef NeoSwerveModule SwerveModule;
#endif
#ifdef USE_TALONFX_SWERVE
  #include "TalonFxSwerveModule.hpp"                    // for interacting with a TalonFX swerve module
  typedef TalonFxSwerveModule SwerveModule;
#endif
#include "ctre/phoenix6/Pigeon2.hpp"                    // for PigeonIMU

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class SwerveDrive
///
/// Declarations for a swerve drive object.
///
////////////////////////////////////////////////////////////////
class SwerveDrive
{
    typedef SwerveModule::SwerveModuleConfig SwerveModuleConfig;

public:
    // Constructor
    SwerveDrive(Pigeon2 * pPigeon);

    // Updates each swerve module based on the inputs
    void SetModuleStates(Translation2d translation, double rotation, bool bFieldRelative, bool bIsOpenLoop);

    // Puts useful values on the dashboard
    void UpdateSmartDashboard();

    // Sets the gyro yaw back to zero degrees
    inline void ZeroGyroYaw()
    {
        m_pPigeon->SetYaw(0.0_deg);
    }

    // Points all the modules to zero degrees, which should be straight forward
    inline void HomeModules()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].HomeModule();
        }
    }

    // Lock the wheels in an X pattern to prevent movement
    inline void LockWheels()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].LockWheel();
        }
    }

    // Recalibrate the modules based on the absolute encoder
    inline void RecalibrateModules()
    {
        for (uint32_t i = 0U; i < SwerveConfig::NUM_SWERVE_DRIVE_MODULES; i++)
        {
            m_SwerveModules[i].RecalibrateModules();
        }
    }

private:
    Pigeon2 * m_pPigeon;
    SwerveModule m_SwerveModules[SwerveConfig::NUM_SWERVE_DRIVE_MODULES];

    // From https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
    // 0 degrees / radians represents the robot angle when the robot is facing directly toward your opponent’s
    // alliance station. As your robot turns to the left, your gyroscope angle should increase. By default, WPILib
    // gyros exhibit the opposite behavior, so you should negate the gyro angle.
    SwerveDriveOdometry<SwerveConfig::NUM_SWERVE_DRIVE_MODULES> m_Odometry;

    static constexpr const SwerveModulePosition INITIAL_SWERVE_MODULE_POSITION = {0_m, 0_deg};

    // Note: If using the RobotTestCode routines (for Neo swerve), these objects have to be disabled (or use different CAN IDs).

    // Config information on each swerve module.
    // Fields are: Name, Position, Drive TalonFX CAN ID, Angle TalonFX CAN ID, CANCoder ID, Angle Offset
    // 2025 Raw Measurements (bevels right)
    // FL: 0.428467_tr (154.24812_deg)
    // FR: 0.153320_tr (55.1952_deg)
    // BL: 0.014404_tr (5.18544_deg)
    // BR: 0.653320_tr (235.1952_deg)
    static constexpr const SwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = {"Front left", SwerveModule::FRONT_LEFT, 11, 12, 1, 154.24812_deg};
    static constexpr const SwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = {"Front right", SwerveModule::FRONT_RIGHT, 13, 14, 2, 55.1952_deg};
    static constexpr const SwerveModuleConfig BACK_LEFT_MODULE_CONFIG = {"Back left", SwerveModule::BACK_LEFT, 15, 16, 3, 5.18544_deg};
    static constexpr const SwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = {"Back right", SwerveModule::BACK_RIGHT, 17, 18, 4, 235.1952_deg};

    SwerveDrive(const SwerveDrive &) = delete;
    SwerveDrive & operator=(const SwerveDrive &) = delete;
};

#endif // SWERVEDRIVE_HPP
