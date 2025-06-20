#include "imgui.h"
#include "implot.h"
#include <vector>
#include <cmath>
#include <random>
#include "Kalman_RLS.hpp"
#include <algorithm>


#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

// -----------------------------------------------------------------------------
// Global variables for plotting and control
// -----------------------------------------------------------------------------
std::vector<float> x, noisy, filtered, ground_truth;
float R = 0.4f; // Shared measurement noise variance (tunable via UI)

// -----------------------------------------------------------------------------
// Simulate a noisy measurement around the true value
// -----------------------------------------------------------------------------
float simulate_measurement(float true_value, float C, std::default_random_engine& gen, float R) {
    std::normal_distribution<float> dist(0.0f, std::sqrt(R));
    return C * true_value + dist(gen);
}


// -----------------------------------------------------------------------------
// Update simulated measurement and Kalman filter
// -----------------------------------------------------------------------------
void update_data() {
    static float true_value = 1.5f;             // Constant value to estimate
    static float C = 10.0f;                     // Sensor gain
    static Kalman_RLS kf(C, R, 0.0f, 0.1f);     // Initial filter

    static std::default_random_engine gen;
    static float last_R = -1.0f;

    // Update R in filter only if it has changed
    if (std::abs(R - last_R) > 1e-6f) {
        kf.setR(R);
        last_R = R;
    }

    float z = simulate_measurement(true_value, C, gen, R);
    kf.update(z);
    float estimate = kf.getEstimate();

    int step = x.size();
    x.push_back((float)step);
    noisy.push_back(z/C); // Normalize noisy measurement
    filtered.push_back(estimate);
    ground_truth.push_back(true_value);
}

// -----------------------------------------------------------------------------
// ImGui setup and shutdown
// -----------------------------------------------------------------------------
void setup_imgui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
}

void shutdown_imgui() {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

// -----------------------------------------------------------------------------
// Main application entry
// -----------------------------------------------------------------------------
int main() {
    if (!glfwInit())
        return -1;

    GLFWwindow* window = glfwCreateWindow(800, 600, "Kalman Filter - ImPlot", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    setup_imgui(window);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // UI controls
        ImGui::Begin("Kalman Parameters");
        ImGui::SliderFloat("R (measurement noise)", &R, 0.01f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::End();

        // Simulate 100 steps max
        static size_t i {0};
        if (i < 100) {
            update_data();
            i++;
        }

        // Plotting window
        ImGui::Begin("Kalman Filter Visualization");
        if (ImPlot::BeginPlot("Estimate vs Measurement")) {
            ImPlot::PlotLine("Measurement", x.data(), noisy.data(), noisy.size());
            ImPlot::PlotLine("Estimate", x.data(), filtered.data(), filtered.size());
            ImPlot::PlotLine("True Value", x.data(), ground_truth.data(), ground_truth.size());
            ImPlot::EndPlot();
        }
        ImGui::End();


        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    shutdown_imgui();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
