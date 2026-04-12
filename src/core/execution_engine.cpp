// ---------------------------------------------------------------------------------------------------------------------
/// \file execution_engine.cpp
/// \date 12. 04. 2026
/// \author Gemini CLI
///
/// \brief This file implements the execution engine of the emulator.
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <thread>
#include <algorithm>
/// \endcond

// Project file imports

#include "soc.hpp"
#include "execution_engine.hpp"

namespace zero_mate::core
{
    CExecution_Engine::CExecution_Engine(std::shared_ptr<arm1176jzf_s::CCPU_Core> cpu)
    : m_cpu{ cpu }
    , m_running{ false }
    , m_stop_requested{ false }
    , m_breakpoint_hit{ false }
    {
    }

    CExecution_Engine::~CExecution_Engine()
    {
        Stop();
        while (m_running)
        {
            // Wait for the thread to finish
        }
    }

    void CExecution_Engine::Register_Listener(IExecution_Listener* listener)
    {
        m_listeners.push_back(listener);
    }

    void CExecution_Engine::Start()
    {
        if (m_running)
        {
            return;
        }

        m_stop_requested = false;
        m_breakpoint_hit = false;

        std::thread execution_thread(&CExecution_Engine::Run, this);
        execution_thread.detach();
    }

    void CExecution_Engine::Stop()
    {
        m_stop_requested = true;
    }

    void CExecution_Engine::Step()
    {
        if (m_running)
        {
            return;
        }

        m_cpu->Step(true);
    }

    bool CExecution_Engine::Is_Running() const noexcept
    {
        return m_running;
    }

    const std::atomic<bool>& CExecution_Engine::Get_Running_Flag() const noexcept
    {
        return m_running;
    }

    bool CExecution_Engine::Has_Hit_Breakpoint() const noexcept
    {
        return m_breakpoint_hit;
    }

    void CExecution_Engine::Run()
    {
        m_running = true;
        m_running.notify_all();

        // Perform one step without checking for breakpoints to unblock.
        m_cpu->Step(true);

        while (!m_stop_requested)
        {
            // Perform a single step and check for breakpoints.
            if (!m_cpu->Step())
            {
                m_breakpoint_hit = true;
                break;
            }

            // Check if any of the listeners want to stop the execution.
            bool should_stop = std::any_of(m_listeners.begin(), m_listeners.end(), [](auto* listener) {
                return listener->Should_Stop();
            });

            if (should_stop)
            {
                zero_mate::soc::g_logging_system.Debug("Execution engine: Halted by a listener");
                break;
            }
        }

        m_running = false;
        m_running.notify_all();
    }

} // namespace zero_mate::core
