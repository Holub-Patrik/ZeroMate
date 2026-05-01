// ---------------------------------------------------------------------------------------------------------------------
/// \file execution_listener.hpp
/// \date 12. 04. 2026
/// \author Patrik Holub
///
/// \brief This file defines an interface for objects that want to control the execution loop of the emulator.
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

namespace zero_mate
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class IExecution_Listener
    /// \brief This class represents an interface for objects that want to control the execution loop of the emulator.
    // -----------------------------------------------------------------------------------------------------------------
    class IExecution_Listener
    {
    public:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Destroys (deletes) the object from memory.
        // -------------------------------------------------------------------------------------------------------------
        virtual ~IExecution_Listener() = default;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns whether or not the emulator should stop execution.
        /// \return true, if the emulator should stop. false, otherwise.
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] virtual bool Should_Stop() = 0;
    };

} // namespace zero_mate
