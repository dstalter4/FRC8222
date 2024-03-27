////////////////////////////////////////////////////////////////////////////////
/// @file   CmsdCustomController.hpp
/// @author David Stalter
///
/// @details
/// A class designed to interface to several controller types (Logitech Gamepad,
/// Xbox GameSir, PS4, etc.) with custom responses.
///
///
/// Copyright (c) 2024 CMSD
////////////////////////////////////////////////////////////////////////////////

#ifndef CMSDCUSTOMCONTROLLER_HPP
#define CMSDCUSTOMCONTROLLER_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "frc/GenericHID.h"             // for base class declaration

// C++ INCLUDES
#include "ControllerConfiguration.hpp"  // for Cmsd::Controller::Config::Styles

using namespace frc;


////////////////////////////////////////////////////////////////
/// @class CmsdCustomController
///
/// Class that provides methods for interacting with a generic
/// controller.  It is derived from GenericHID and can support
/// several different types of controllers.
///
////////////////////////////////////////////////////////////////
class CmsdCustomController : public GenericHID
{
public:
    
    // Constructor/destructor
    explicit CmsdCustomController(Cmsd::Controller::Config::Models controllerModel, int port);
    virtual ~CmsdCustomController() = default;
    
    double GetDriveX() const;
    double GetDriveY() const;
    double GetDriveRotate() const;
    double GetThrottle() const;
    
private:
    
    ////////////////////////////////////////////////////////////////
    /// @method CmsdController::NormalizeTriggers
    ///
    /// Function to normalize the trigger inputs to expected output
    /// ranges.  The controller logic wants the left trigger to be
    /// a value between [-1:0] and the right trigger to be a value
    /// between [0:+1].  This function will invoke code specific to
    /// a controller to adjust the triggers since each controller
    /// will be unique in how the axes values are reported.
    ///
    ////////////////////////////////////////////////////////////////
    inline void NormalizeTriggers(double & rLeftTrigger, double & rRightTrigger) const
    {
        switch (CONTROLLER_MODEL)
        {
            case Cmsd::Controller::Config::Models::CUSTOM_LOGITECH:
            case Cmsd::Controller::Config::Models::CUSTOM_XBOX:
            {
                // Logitech and Xbox joystick axes inputs are:
                // LT: 0->+1, RT: 0->+1
                // x-axis: -1    +1

                // LT: in 0->+1, out 0->+1
                // RT: in 0->+1, out -1->0
                rRightTrigger *= -1.0;
                break;
            }
            case Cmsd::Controller::Config::Models::CUSTOM_PLAY_STATION:
            {
                // PlayStation joystick axes inputs are:
                // L2: -1->+1, R2: +1->-1
                // x-axis: -1    +1

                // L2: in -1->+1, out 0->+1
                // R2: in -1->+1, out -1->0
                rLeftTrigger = (rLeftTrigger + 1.0) / 2.0;
                rRightTrigger = (rRightTrigger + 1.0) / -2.0;
                break;
            }
            default:
            {
                break;
            }
        }
    }
    
    const Cmsd::Controller::Config::Models CONTROLLER_MODEL;
    const Cmsd::Controller::Config::Mappings * const CONTROLLER_MAPPINGS;
    double m_ThrottleValue;
    
    static constexpr double X_AXIS_DRIVE_SENSITIVITY_SCALING = 1.00;
    static constexpr double Y_AXIS_DRIVE_SENSITIVITY_SCALING = 1.00;
    
    // Prevent copying/assignment
    CmsdCustomController(const CmsdCustomController&) = delete;
    CmsdCustomController& operator=(const CmsdCustomController&) = delete;
};

#endif // CMSDCUSTOMCONTROLLER_HPP
