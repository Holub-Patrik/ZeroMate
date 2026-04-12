// ---------------------------------------------------------------------------------------------------------------------
/// \file control_window.cpp
/// \date 10. 07. 2023
/// \author Jakub Silhavy (jakub.silhavy.cz@gmail.com)
///
/// \brief This file implements a window that allows the user to control CPU execution.
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <thread>
/// \endcond

// 3rd party libraries

#include "imgui/imgui.h"
#include "IconFontCppHeaders/IconsFontAwesome5.h"

// Project file imports

#include "control_window.hpp"
#include "zero_mate/utils/singleton.hpp"
#include "../../core/soc.hpp"

namespace zero_mate::gui
{
    // Anonymous namespace to make its content visible only to this translation unit.
    namespace
    {
        [[maybe_unused]] void Render_ImGUI_Demo()
        {
            static bool s_show_demo_window{ false };

            // Check box to close the demo window.
            ImGui::Checkbox("Show demo window", &s_show_demo_window);

            // Show the window.
            if (s_show_demo_window)
            {
                ImGui::ShowDemoWindow();
            }
        }
    }

    CControl_Window::CControl_Window(std::shared_ptr<arm1176jzf_s::CCPU_Core> cpu,
                                     bool& scroll_to_curr_line,
                                     const bool& elf_file_has_been_loaded,
                                     const std::string& kernel_filename)
    : m_cpu{ cpu }
    , m_scroll_to_curr_line{ scroll_to_curr_line }
    , m_elf_file_has_been_loaded{ elf_file_has_been_loaded }
    , m_logging_system{ *utils::CSingleton<utils::CLogging_System>::Get_Instance() }
    , m_kernel_filename{ kernel_filename }
    {
    }

    CControl_Window::~CControl_Window()
    {
        // Is the CPU execution thread still running?
        if (Is_Running())
        {
            // Terminate the execution thread.
            soc::g_execution_engine->Stop();

            // Wait for the CPU (execution thread) to stop.
            soc::g_execution_engine->Get_Running_Flag().wait(true);
        }
    }

    void CControl_Window::Render()
    {
        // Render the window.
        if (ImGui::Begin("Control"))
        {
            Render_Control_Buttons();
            Render_CPU_State();
            Render_Currently_Loaded_Kernel();

            // Just for debugging purposes.
            // Render_ImGUI_Demo();
        }

        ImGui::End();
    }

    void CControl_Window::Render_Currently_Loaded_Kernel()
    {
        ImGui::Text("Loaded kernel: %s", m_kernel_filename.c_str());
    }

    void CControl_Window::Render_Step_Button()
    {
        if (ImGui::Button(ICON_FA_STEP_FORWARD " Step") && Is_Stopped())
        {
            // Make sure a kernel has been loaded.
            if (!m_elf_file_has_been_loaded)
            {
                Print_No_ELF_File_Loaded_Error_Msg();
            }
            else
            {
                // Perform a single step regardless of any set breakpoints.
                soc::g_execution_engine->Step();

                // Trigger the GUI to scroll to the current line of execution.
                m_scroll_to_curr_line = true;
            }
        }
    }

    void CControl_Window::Render_Stop_Button()
    {
        if (ImGui::Button(ICON_FA_STOP " Stop") && Is_Running())
        {
            Request_Stop();
        }
    }

    void CControl_Window::Render_Run_Button()
    {
        if (ImGui::Button(ICON_FA_PLAY_CIRCLE " Run") && Is_Stopped())
        {
            if (!m_elf_file_has_been_loaded)
            {
                Print_No_ELF_File_Loaded_Error_Msg();
            }
            else
            {
                Request_Start();
            }
        }
    }

    void CControl_Window::Request_Stop() noexcept
    {
        soc::g_execution_engine->Stop();
    }

    void CControl_Window::Request_Start() noexcept
    {
        if (!Is_Running() && m_elf_file_has_been_loaded)
        {
            soc::g_execution_engine->Start();
        }
    }

    bool CControl_Window::Is_Running() const noexcept
    {
        return soc::g_execution_engine->Is_Running();
    }

    bool CControl_Window::Is_Stopping() const noexcept
    {
        return false; // Not really supported anymore in this simple way
    }

    bool CControl_Window::Is_Stopped() const noexcept
    {
        return !Is_Running();
    }

    void CControl_Window::Render_Control_Buttons()
    {
        // Step button
        Render_Step_Button();
        ImGui::SameLine();

        // Run button
        Render_Run_Button();
        ImGui::SameLine();

        // Stop button
        Render_Stop_Button();
        ImGui::Separator();
    }

    void CControl_Window::Render_CPU_State() const
    {
        // Render a "state" label.
        ImGui::Text("State:");
        ImGui::SameLine();

        if (soc::g_execution_engine->Has_Hit_Breakpoint())
        {
            // Breakpoint
            ImGui::PushStyleColor(ImGuiCol_Text, color::Light_Blue);
            ImGui::Text("Breakpoint");
        }
        else
        {
            if (Is_Running())
            {
                // Running
                ImGui::PushStyleColor(ImGuiCol_Text, color::Green);
                ImGui::Text("Running");
            }
            else
            {
                // Stopped
                ImGui::PushStyleColor(ImGuiCol_Text, color::Red);
                ImGui::Text("Stopped");
            }
        }

        // Do not forget to pop the pushed style (color).
        ImGui::PopStyleColor();
    }

    inline void CControl_Window::Print_No_ELF_File_Loaded_Error_Msg() const
    {
        m_logging_system.Error("No .ELF file has been loaded");
    }

} // namespace zero_mate::gui
