////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdRobot.cpp
/// @author David Stalter
///
/// @details
/// Implementation of the CmsdRobot class.  This file contains the functions
/// for full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// Copyright (c) 2025 CMSD
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>                      // for nullptr
#include <cstring>                      // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"                // for class declaration (and other headers)
//#include "RobotCamera.hpp"              // for interacting with cameras
#include "RobotUtils.hpp"               // for Trim(), Limit() and DisplayMessage()

// STATIC MEMBER VARIABLES
CmsdRobot * CmsdRobot::m_pThis;


////////////////////////////////////////////////////////////////
/// @method CmsdRobot::CmsdRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
CmsdRobot::CmsdRobot() :
    m_AutonomousChooser                 (),
    m_AutoSwerveDirections              (),
    m_pDriveController                  (new DriveControllerType(DRIVE_CONTROLLER_MODEL, DRIVE_JOYSTICK_PORT)),
    m_pAuxController                    (new AuxControllerType(AUX_CONTROLLER_MODEL, AUX_JOYSTICK_PORT)),
    m_pPigeon                           (new Pigeon2(PIGEON_CAN_ID, "canivore-8222")),
    m_pSwerveDrive                      (new SwerveDrive(m_pPigeon)),
    m_pCandle                           (new CANdle(CANDLE_CAN_ID, "canivore-8222")),
    m_RainbowAnimation                  ({1, 0.5, 308}),
    m_pDebugOutput                      (new DigitalOutput(DEBUG_OUTPUT_DIO_CHANNEL)),
    m_pCompressor                       (new Compressor(PneumaticsModuleType::CTREPCM)),
    m_pMatchModeTimer                   (new Timer()),
    m_pRobotProgramTimer                (new Timer()),
    m_pSafetyTimer                      (new Timer()),
    //m_CameraThread                      (RobotCamera::LimelightThread),
    m_RobotMode                         (ROBOT_MODE_NOT_SET),
    m_AllianceColor                     (DriverStation::GetAlliance()),
    m_bRioPinsStable                    (false),
    m_HeartBeat                         (0U)
{
    RobotUtils::DisplayMessage("Robot constructor.");
    
    // LiveWindow is not used
    LiveWindow::SetEnabled(false);
    
    // Set the autonomous options
    m_AutonomousChooser.SetDefaultOption(AUTO_ROUTINE_1_STRING, AUTO_ROUTINE_1_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_2_STRING, AUTO_ROUTINE_2_STRING);
    m_AutonomousChooser.AddOption(AUTO_ROUTINE_3_STRING, AUTO_ROUTINE_3_STRING);
    m_AutonomousChooser.AddOption(AUTO_TEST_ROUTINE_STRING, AUTO_TEST_ROUTINE_STRING);
    SmartDashboard::PutData("Autonomous Modes", &m_AutonomousChooser);

    RobotUtils::DisplayFormattedMessage("The drive forward axis is: %d\n", Cmsd::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.RIGHT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive reverse axis is: %d\n", Cmsd::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_TRIGGER);
    RobotUtils::DisplayFormattedMessage("The drive left/right axis is: %d\n", Cmsd::Controller::Config::GetControllerMapping(DRIVE_CONTROLLER_MODEL)->AXIS_MAPPINGS.LEFT_X_AXIS);

    ConfigureMotorControllers();

    CANdleConfiguration candleConfig;
    candleConfig.stripType = LEDStripType::RGB;
    m_pCandle->ConfigAllSettings(candleConfig);
    m_pCandle->Animate(m_RainbowAnimation);

    // Spawn the vision thread
    //RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    //RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);
    //m_CameraThread.detach();

    // Start the free running timer
    m_pRobotProgramTimer->Reset();
    m_pRobotProgramTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::ResetMemberData
///
/// This method resets relevant member data variables.  Since
/// the robot object is only constructed once, it may be
/// necessary/helpful to return to a state similar to when the
/// constructor first ran (e.g. when enabling/disabling robot
/// states).  Only variables that need to be reset are modified
/// here.  This also works around the issue where non-member
/// static data cannot be easily reinitialized (since clearing
/// the .bss and running static constructors will only happen
/// once on program start up).
///
////////////////////////////////////////////////////////////////
void CmsdRobot::ResetMemberData()
{
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::RobotInit
///
/// This method is run when initializing the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::RobotInit()
{
    RobotUtils::DisplayMessage("RobotInit called.");
    SetStaticThisInstance();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::RobotPeriodic
///
/// This method is run in all robot states.  It is called each
/// time a new packet is received from the driver station.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::RobotPeriodic()
{
    static bool bRobotPeriodicStarted = false;
    if (!bRobotPeriodicStarted)
    {
        RobotUtils::DisplayMessage("RobotPeriodic called.");
        bRobotPeriodicStarted = true;
    }

    CheckIfRioPinsAreStable();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::CheckIfRioPinsAreStable
///
/// Wait for any sensors on the robot that route to the RIO
/// to stabilize for accurate readings.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::CheckIfRioPinsAreStable()
{
    // This is the logic to wait to take PWM based sensor readings until the RIO is ready.
    // The behavior of the RIO is that it measures how many microseconds the signal is high
    // every second.  This requires waiting to get stable readings.
    // See https://github.com/wpilibsuite/allwpilib/issues/5284 for some related info.
    static units::time::second_t enabledTimeStamp = 0.0_s;
    units::time::second_t currentTimeStamp = m_pRobotProgramTimer->Get();
    if (DriverStation::IsEnabled() && (!m_bRioPinsStable))
    {
        // If the robot was just enabled (in any mode)
        if (enabledTimeStamp == 0.0_s)
        {
            // Set the start time stamp
            enabledTimeStamp = currentTimeStamp;
        }

        // Now check if enough time has passed for the RIO pins to have stabilized
        static constexpr const units::time::second_t RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY = 2.0_s;
        if ((currentTimeStamp - enabledTimeStamp) > RIO_DUTY_CYCLE_ENCODER_STARTUP_DELAY)
        {
            // Example encoder configuration algorithm

            //double encoderValue = m_pEncoder->Get();
            //units::angle::degree_t encoderValueDegrees(encoderValue * ANGLE_360_DEGREES);

            // This is the delta between the current mechanism position and the desired starting position (or zero point)
            //units::angle::degree_t startingOffsetDegrees = encoderValueDegrees - STARTING_POSITION_ENCODER_VALUE;
            //std::printf("startingOffsetDegrees (start): %f\n", startingOffsetDegrees.value());

            // If the starting offset is negative, we crossed over the absolute encoder boundary
            // We give a tolerance of five degrees in case the mechanism is near where we want to start
            // @todo: Does this need to check for very small readings below zero?
            // @todo: Boundary conditions here will be difficult
            //if (startingOffsetDegrees < ENCODER_BOUNDARY_TOLERANCE_DEGREES)
            //{
                // the 0/1 boundary is 360, so subtract the starting position to see how many degrees were up to that point
                // Add in the absolute value of the overage, which was negative
                //startingOffsetDegrees = (units::angle::degree_t(ANGLE_360_DEGREES) - STARTING_POSITION_ENCODER_VALUE) + encoderValueDegrees;
            //}

            // At this point we have the angle we want relative to zero
            //(void)m_pMotor->m_pTalonFx->GetConfigurator().SetPosition(startingOffsetDegrees);
            //std::printf("encoderValue: %f\n", encoderValue);
            //std::printf("encoderValueDegrees: %f\n", encoderValueDegrees.value());
            //std::printf("startingOffsetDegrees (final): %f\n", startingOffsetDegrees.value());

            m_bRioPinsStable = true;
        }
    }
    else
    {
        // Set the enabled time stamp back to zero until the robot is enabled again
        enabledTimeStamp = 0.0_s;

        // m_bRioPinsStable exists for the life of the program.  Once we have a stable
        // reading acquired, we don't need to do it again until the robot program restarts.
    }
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::ConfigureMotorControllers
///
/// Sets motor controller specific configuration information.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::ConfigureMotorControllers()
{
    // These are the defaults for the configuration (see TalonFX.h)
    //ctre::phoenix::sensors::AbsoluteSensorRange absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    //double integratedSensorOffsetDegrees = 0;
    //ctre::phoenix::sensors::SensorInitializationStrategy initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero;

    // The default constructor for TalonFXConfiguration will call the parent
    // BaseTalonConfiguration constructor with FeedbackDevice::IntegratedSensor.

    /*
    // Example configuration
    TalonFXConfiguration talonConfig;
    talonConfig.slot0.kP = 0.08;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.3;
    talonConfig.slot0.kF = 0.0;
    talonConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
    talonConfig.integratedSensorOffsetDegrees = 0.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.slot0.closedLoopPeakOutput = 0.10;

    TalonFX * pTalon = new TalonFX(0xFF);
    pTalon->ConfigFactoryDefault();
    pTalon->ConfigAllSettings(talonConfig);
    pTalon->SetSelectedSensorPosition(0);
    const StatorCurrentLimitConfiguration INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG = {true, 5.0, 50.0, 5.0};
    pTalon->ConfigStatorCurrentLimit(INTAKE_MOTOR_STATOR_CURRENT_LIMIT_CONFIG);
    */

    // Some notes about applying motor configurations:
    // - The classes/structs in YtaTalon.hpp have motor configuration objects in them.
    // - Declaring stack local or class scope configuration objects are *separate and
    //   distinct* from the configuration objects in the YtaTalon.hpp classes/structs.
    // - If a stack local or class scope configuration is applied, it will overwrite
    //   the configuration stored in the device.
    // - Calling the methods provided by YtaTalon.hpp *never* update the configuration
    //   objects in the classes/structs.  To update those objects, retrieve the objects
    //   via things like m_MotorConfiguration (for individual motors) or
    //   GetMotorConfiguration() (for motor groups).
    // - The classes/structs in YtaTalon.hpp provide ApplyConfiguration() routines.
    //   These can be used to directly apply a stack local or class scope configuration,
    //   or to apply an updated configuration when the configuration objects were directly
    //   modified.  Keep the notes above in mind when calling them.
    // - The ApplyConfiguration() method for motor groups will default to applying the
    //   configuration to all motors unless a specific CAN ID is given.  The configuration
    //   applied to each motor in the group is the *saved configuration* for that specific
    //   motor.  It may be different for each motor, depending on the robot code.
    // - Configurations can be applied to the whole configuration object type, or to
    //   sub-types only (e.g. TalonFXConfiguration vs. CurrentLimitsConfigs), as
    //   ApplyConfiguration() is overloaded.  The template version only applies stack
    //   local or class scope configs, so remember the notes above.  Right now the
    //   template version is disabled, so don't call it.
    // - Thank CTRE for all this.  Instead of letting the config be a member of the motor
    //   object class with simple getter/setters, it's separate and overly complex.

    // Example configurations

    // Configure a motor group (only needs to be applied to the lead motor of the group)
    // Brake mode was set when the motor group was constructed
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Feedback.WithSensorToMechanismRatio(12.0 / 1.0);
    //(void)m_pMotors->GetMotorConfiguration(MOTORS_CAN_START_ID)->Slot0.WithKP(18.0).WithKI(0.0).WithKD(0.1);
    //m_pMotors->ApplyConfiguration(MOTORS_CAN_START_ID);
    //(void)m_pMotors->GetMotorObject(MOTORS_CAN_START_ID)->GetConfigurator().SetPosition(0.0_tr);

    // Configure a single motor
    //(void)m_pMotor->m_MotorConfiguration.MotorOutput.WithNeutralMode(NeutralModeValue::Brake);
    //(void)m_pMotor->m_MotorConfiguration.Feedback.WithSensorToMechanismRatio(135.0 / 1.0);
    //(void)m_pMotor->m_MotorConfiguration.Slot0.WithKP(50.0).WithKI(0.0).WithKD(2.0);
    //(void)m_pMotor->m_pTalonFx->GetConfigurator().SetPosition(0.0_tr);
    //m_pMotor->ApplyConfiguration();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::InitialStateSetup()
{
    // First reset any member data
    ResetMemberData();

    // Stop/clear any timers, just in case
    // @todo: Make this a dedicated function.
    m_pMatchModeTimer->Stop();
    m_pMatchModeTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Just in case constructor was called before these were set (likely the case)
    m_AllianceColor = DriverStation::GetAlliance();

    // Disable the rainbow animation
    m_pCandle->ClearAnimation(0);

    //Set the LEDs to the alliance color
    SetLedsToAllianceColor();

    // Indicate the camera thread can continue
    //RobotCamera::ReleaseThread();

    // Clear the debug output pin
    m_pDebugOutput->Set(false);

    // Reset the heartbeat
    m_HeartBeat = 0U;

    // Point the swerve modules straight.  With SparkMax, this (also) addresses
    // an issue where setting position during constructors doesn't take effect.
    m_pSwerveDrive->HomeModules();

    // With CTRE swerve electronics, sometimes the CANcoder appears to not be
    // ready when constructors measure the absolute position.  The issue isn't
    // entirely understood, but recalibrating here seems to provide stability.
    // Note: This won't work if Neo swerve is used.
    m_pSwerveDrive->RecalibrateModules();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::TeleopInit
///
/// The teleop init method.  This method is called once each
/// time the robot enters teleop control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::TeleopInit()
{
    RobotUtils::DisplayMessage("TeleopInit called.");
    
    // Autonomous should have left things in a known state, but
    // just in case clear everything.
    InitialStateSetup();

    // Tele-op won't do detailed processing of the images unless instructed to
    //RobotCamera::SetFullProcessing(false);
    //RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    //RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

    // Start the mode timer for teleop
    m_pMatchModeTimer->Start();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::TeleopPeriodic
///
/// The teleop control method.  This method is called
/// periodically while the robot is in teleop control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::TeleopPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_TELEOP);

    HeartBeat();

    if (Cmsd::Drive::Config::USE_SWERVE_DRIVE)
    {
        SwerveDriveSequence();
    }

    //PneumaticSequence();
    
    //CameraSequence();

    UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::UpdateSmartDashboard
///
/// Updates values in the smart dashboard.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::UpdateSmartDashboard()
{
    // @todo: Check if RobotPeriodic() is called every 20ms and use static counter.
    // Give the drive team some state information
    SmartDashboard::PutBoolean("RIO pins stable", m_bRioPinsStable);
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::PneumaticSequence
///
/// This method contains the main workflow for updating the
/// state of the pnemuatics on the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::PneumaticSequence()
{
    // @todo: Monitor other compressor API data?
    SmartDashboard::PutBoolean("Compressor status", m_pCompressor->IsEnabled());
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::CameraSequence()
{
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::SwerveDriveSequence
///
/// This method contains the main workflow for swerve drive
/// control.  It will gather input from the drive joystick and
/// then filter those values to ensure they are past a certain
/// threshold (deadband) and generate the information to pass
/// on to the swerve drive system.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SwerveDriveSequence()
{
    // Check for a switch between field relative and robot centric
    static bool bFieldRelative = true;
    if (m_pDriveController->DetectButtonChange(FIELD_RELATIVE_TOGGLE_BUTTON))
    {
        bFieldRelative = !bFieldRelative;
    }

    if (m_pDriveController->DetectButtonChange(REZERO_SWERVE_BUTTON))
    {
        m_pSwerveDrive->ZeroGyroYaw();
        m_pSwerveDrive->RecalibrateModules();
        m_pSwerveDrive->HomeModules();
    }

    // The GetDriveX() and GetDriveYInput() functions refer to ***controller joystick***
    // x and y axes.  Multiply by -1.0 here to keep the joystick input retrieval code common.
    double translationAxis = RobotUtils::Trim(m_pDriveController->GetDriveYInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double strafeAxis = RobotUtils::Trim(m_pDriveController->GetDriveXInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    double rotationAxis = RobotUtils::Trim(m_pDriveController->GetDriveRotateInput() * -1.0, JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // Override normal control if a fine positioning request is made
    switch (m_pDriveController->GetPovAsDirection())
    {
        case Cmsd::Controller::PovDirections::POV_UP:
        {
            translationAxis = SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case Cmsd::Controller::PovDirections::POV_DOWN:
        {
            translationAxis = -SWERVE_DRIVE_SLOW_SPEED;
            strafeAxis = 0.0;
            rotationAxis = 0.0;
            break;
        }
        case Cmsd::Controller::PovDirections::POV_LEFT:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (Cmsd::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (Cmsd::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        case Cmsd::Controller::PovDirections::POV_RIGHT:
        {
            // Left/right POV control can either toggle strafe or rotation
            translationAxis = 0.0;
            strafeAxis = (Cmsd::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (0.0) : (-SWERVE_DRIVE_SLOW_SPEED);
            rotationAxis = (Cmsd::Drive::Config::SWERVE_SLOW_USE_ROTATION_AXIS) ? (-SWERVE_ROTATE_SLOW_SPEED) : (0.0);
            break;
        }
        default:
        {
            break;
        }
    }

    SmartDashboard::PutNumber("Strafe", strafeAxis);
    SmartDashboard::PutNumber("Translation", translationAxis);
    SmartDashboard::PutNumber("Rotation", rotationAxis);
    SmartDashboard::PutBoolean("Field Relative", bFieldRelative);

    // Notice that this is sending translation to X and strafe to Y, despite
    // the inputs coming from the opposite of what may be intuitive (strafe as X,
    // translation as Y).  See the comment in Translation2d.h about the robot
    // placed at origin facing the X-axis.  Forward movement increases X and left
    // movement increases Y.
    Translation2d translation = {units::meter_t(translationAxis), units::meter_t(strafeAxis)};

    // Update the swerve module states
    m_pSwerveDrive->SetModuleStates(translation, rotationAxis, bFieldRelative, true);

    // Display some useful information
    m_pSwerveDrive->UpdateSmartDashboard();
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::DisabledInit
///
/// The disabled init method.  This method is called once each
/// time the robot enters disabled mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::DisabledInit()
{
    RobotUtils::DisplayMessage("DisabledInit called.");

    // @todo: Shut off the limelight LEDs?
    //RobotCamera::SetLimelightMode(RobotCamera::LimelightMode::DRIVER_CAMERA);
    //RobotCamera::SetLimelightLedMode(RobotCamera::LimelightLedMode::PIPELINE);

    // Turn the rainbow animation back on
    m_pCandle->Animate(m_RainbowAnimation);
}



////////////////////////////////////////////////////////////////
/// @method CmsdRobot::DisabledPeriodic
///
/// The disabled control method.  This method is called
/// periodically while the robot is disabled.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::DisabledPeriodic()
{
    // Log a mode change if one occurred
    CheckAndUpdateRobotMode(ROBOT_MODE_DISABLED);
}



////////////////////////////////////////////////////////////////
/// @method main
///
/// Execution start for the robt.
///
////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<CmsdRobot>();
}
#endif
