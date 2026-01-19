////////////////////////////////////////////////////////////////////////////////
/// @file   ControllerTemplateSpecializations.cpp
/// @author David Stalter
///
/// @details
/// Implements the specializations for the CmsdController and
/// CmsdDriveController template classes.
///
/// Copyright (c) 2024 CMSD
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdController.hpp"                       // for class declarations

// STATIC MEMBER DATA
// (none)


////////////////////////////////////////////////////////////////
/// CmsdCustomController template specializations for both
/// CmsdController<> and CmsdDriveController<>.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method CmsdController<CmsdCustomController>::CmsdController
///
/// Constructor for a template instantiated with a custom CMSD
/// controller type.
///
////////////////////////////////////////////////////////////////
template <>
CmsdController<CmsdCustomController>::CmsdController(Cmsd::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new CmsdCustomController(controllerModel, controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges()
{
}

////////////////////////////////////////////////////////////////
/// @method CmsdController<CmsdCustomController>::GetThrottleControl
///
/// Retrieves a throttle value for the controller.  Specialized
/// because some built-in types have an available fixed
/// position axis that can provide throttle (such as a z-axis).
/// This function just needs to pass through for a custom CMSD
/// controller.
///
////////////////////////////////////////////////////////////////
template <>
double CmsdController<CmsdCustomController>::GetThrottleControl()
{
    return m_pController->GetThrottle();
}

////////////////////////////////////////////////////////////////
/// @method CmsdDriveController<CmsdCustomController>::GetDriveXInput
///
/// Retrieves the x-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double CmsdDriveController<CmsdCustomController>::GetDriveXInput()
{
    return m_pController->GetDriveX();
}

////////////////////////////////////////////////////////////////
/// @method CmsdDriveController<CmsdCustomController>::GetDriveYInput
///
/// Retrieves the y-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double CmsdDriveController<CmsdCustomController>::GetDriveYInput()
{
    return m_pController->GetDriveY();
}

////////////////////////////////////////////////////////////////
/// @method CmsdDriveController<CmsdCustomController>::GetDriveRotateInput
///
/// Retrieves the drive rotate axis value from controller inputs.
///
////////////////////////////////////////////////////////////////
template <>
double CmsdDriveController<CmsdCustomController>::GetDriveRotateInput()
{
    return m_pController->GetDriveRotate();
}
