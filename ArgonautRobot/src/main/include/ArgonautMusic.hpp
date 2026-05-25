////////////////////////////////////////////////////////////////////////////////
/// @file   ArgonautMusic.hpp
/// @author David Stalter
///
/// @details
/// Implements functionality for playing music on a robot.
///
/// Copyright (c) 2026 Argonaut
////////////////////////////////////////////////////////////////////////////////

#ifndef ARGONAUTMUSIC_HPP
#define ARGONAUTMUSIC_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "ctre/phoenix6/TalonFX.hpp"            // for TalonFX type

using namespace ctre::phoenix6::controls;
using namespace ctre::phoenix6::hardware;


namespace Argonaut::Music::Config
{
    static constexpr const bool PLAYING_TONES_ENABLED = false;
}


////////////////////////////////////////////////////////////////
/// @class ArgonautMusicController
///
/// Declarations for managing music/tones on a robot.
///
////////////////////////////////////////////////////////////////
class ArgonautMusicController
{
public:
    static bool PlayTones(TalonFX * pTalonFx);
};

#endif // ARGONAUTMUSIC_HPP
