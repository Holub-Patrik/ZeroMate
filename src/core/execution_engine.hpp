// ---------------------------------------------------------------------------------------------------------------------
/// \file execution_engine.hpp
/// \date 12. 04. 2026
/// \author Gemini CLI
///
/// \brief This file defines the execution engine of the emulator.
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <memory>
#include <vector>
#include <atomic>
#include <thread>
/// \endcond

// Project file imports

#include "arm1176jzf_s/core.hpp"
#include "zero_mate/execution_listener.hpp"

namespace zero_mate::core
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class CExecution_Engine
    /// \brief This class represents the execution engine of the emulator.
    ///
    /// It manages the execution loop, notifying listeners and checking for stop requests.
    // -----------------------------------------------------------------------------------------------------------------
    class CExecution_Engine
    {
    public:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Creates an instance of the class.
        /// \param cpu Reference to the CPU (stepping through the code)
        // -------------------------------------------------------------------------------------------------------------
        explicit CExecution_Engine(std::shared_ptr<arm1176jzf_s::CCPU_Core> cpu);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Deletes the object from memory.
        // -------------------------------------------------------------------------------------------------------------
        ~CExecution_Engine();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Registers an execution listener.
        /// \param listener Execution listener to be registered
        // -------------------------------------------------------------------------------------------------------------
        void Register_Listener(IExecution_Listener* listener);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Requests the CPU execution to start (separate thread).
        // -------------------------------------------------------------------------------------------------------------
        void Start();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Requests the CPU execution to stop.
        // -------------------------------------------------------------------------------------------------------------
        void Stop();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Performs a single step.
        // -------------------------------------------------------------------------------------------------------------
        void Step();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns true if the CPU is currently running.
        /// \return True if the CPU is currently running
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] bool Is_Running() const noexcept;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns a reference to the running flag (C++20 wait/notify support).
        /// \return Reference to the running flag
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] const std::atomic<bool>& Get_Running_Flag() const noexcept;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns true if a breakpoint has been hit.
        /// \return True if a breakpoint has been hit
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] bool Has_Hit_Breakpoint() const noexcept;

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Runs CPU execution (separate thread).
        // -------------------------------------------------------------------------------------------------------------
        void Run();

    private:
        std::shared_ptr<arm1176jzf_s::CCPU_Core> m_cpu; ///< CPU
        std::vector<IExecution_Listener*> m_listeners;  ///< Collection of execution listeners
        std::atomic<bool> m_running;                   ///< Is the CPU running?
        std::atomic<bool> m_stop_requested;            ///< Has a stop been requested?
        std::atomic<bool> m_breakpoint_hit;            ///< Has a breakpoint been hit?
    };

} // namespace zero_mate::core
