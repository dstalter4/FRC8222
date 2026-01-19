////////////////////////////////////////////////////////////////////////////////
/// @file   ControllerTemplateSpecializations.cpp
/// @author David Stalter
///
/// @details
/// Implements the specializations for the ArgonautController and
/// ArgonautDriveController template classes.
///
/// Copyright (c) 2026 Argonaut
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "ArgonautController.hpp"                   // for class declarations

// STATIC MEMBER DATA
// (none)


////////////////////////////////////////////////////////////////
/// ArgonautCustomController template specializations for both
/// ArgonautController<> and ArgonautDriveController<>.
///
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
/// @method ArgonautController<ArgonautCustomController>::ArgonautController
///
/// Constructor for a template instantiated with a custom Argonaut
/// controller type.
///
////////////////////////////////////////////////////////////////
template <>
ArgonautController<ArgonautCustomController>::ArgonautController(Argonaut::Controller::Config::Models controllerModel, int controllerPort) :
    m_pController(new ArgonautCustomController(controllerModel, controllerPort)),
    m_ControllerModel(controllerModel),
    m_ButtonStateChanges()
{
}

////////////////////////////////////////////////////////////////
/// @method ArgonautController<ArgonautCustomController>::GetThrottleControl
///
/// Retrieves a throttle value for the controller.  Specialized
/// because some built-in types have an available fixed
/// position axis that can provide throttle (such as a z-axis).
/// This function just needs to pass through for a custom Argonaut
/// controller.
///
////////////////////////////////////////////////////////////////
template <>
double ArgonautController<ArgonautCustomController>::GetThrottleControl()
{
    return m_pController->GetThrottle();
}

////////////////////////////////////////////////////////////////
/// @method ArgonautDriveController<ArgonautCustomController>::GetDriveXInput
///
/// Retrieves the x-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double ArgonautDriveController<ArgonautCustomController>::GetDriveXInput()
{
    return m_pController->GetDriveX();
}

////////////////////////////////////////////////////////////////
/// @method ArgonautDriveController<ArgonautCustomController>::GetDriveYInput
///
/// Retrieves the y-axis drive value from controller inputs.
/// Specialized to provide video game style drive controls
/// instead of using a single axis.
///
////////////////////////////////////////////////////////////////
template <>
double ArgonautDriveController<ArgonautCustomController>::GetDriveYInput()
{
    return m_pController->GetDriveY();
}

////////////////////////////////////////////////////////////////
/// @method ArgonautDriveController<ArgonautCustomController>::GetDriveRotateInput
///
/// Retrieves the drive rotate axis value from controller inputs.
///
////////////////////////////////////////////////////////////////
template <>
double ArgonautDriveController<ArgonautCustomController>::GetDriveRotateInput()
{
    return m_pController->GetDriveRotate();
}
