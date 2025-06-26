#include "kalman_filter.hpp"
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <iostream>

// ImGui and ImPlot
#include "external/imgui/imgui.h"
#include "external/imgui/backends/imgui_impl_glfw.h"
#include "external/imgui/backends/imgui_impl_opengl3.h"
#include "external/implot/implot.h"
#include <GLFW/glfw3.h>
#include <GL/gl.h>

// Helper for error callback
static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

int main() {
    // Simulation parameters (same as kalman_sim.cpp)
    double g = 9.81, l = 1.0, m = 1.0, zeta = 0.1;
    double omega_n = std::sqrt(g / l);
    Eigen::MatrixXd A(2,2);
    A << -2 * zeta * omega_n, -omega_n * omega_n,
          1,                 0;
    Eigen::MatrixXd B(2,1); B << 1.0 / (m * l * l), 0;
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd L(2,1); L << 1, 1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1,1) * 0.1;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2) * 0.2;
    double T = 0.01;
    KalmanFilter kf(A, B, C, Eigen::MatrixXd(), L, Q, R);
    kf.compute_discrete_matrices(T);
    Eigen::VectorXd x0(2); x0 << 0.0, 0.1;
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(2,2) * 0.1;
    double T_total = 10.0;
    int N_steps = static_cast<int>(T_total / T);
    Eigen::MatrixXd u_seq(1, N_steps); u_seq.setOnes();
    for (int i = static_cast<int>(2.0 / T); i < static_cast<int>(5.0 / T); ++i) u_seq(0, i) = 0.0;
    Eigen::MatrixXd X_true(2, N_steps + 1); X_true.col(0) = x0;
    for (int k = 0; k < N_steps; ++k) X_true.col(k + 1) = kf.Phi * X_true.col(k) + kf.Gamma * u_seq.col(k);
    std::mt19937 gen(42);
    std::normal_distribution<double> noise_theta(0.0, std::sqrt(R(1,1)));
    std::normal_distribution<double> noise_theta_dot(0.0, std::sqrt(R(0,0)));
    Eigen::MatrixXd z(2, N_steps + 1);
    for (int k = 0; k < N_steps + 1; ++k) {
        z(0, k) = X_true(0, k) + noise_theta_dot(gen);
        z(1, k) = X_true(1, k) + noise_theta(gen);
    }
    std::normal_distribution<double> noise_u(0.0, std::sqrt(Q(0,0)));
    Eigen::MatrixXd u_seq_noisy = u_seq;
    for (int k = 0; k < N_steps; ++k) u_seq_noisy(0, k) += noise_u(gen);
    // GLFW + OpenGL + ImGui/ImPlot setup
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;
    const char* glsl_version = "#version 130";
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    int win_w = mode->width, win_h = mode->height;
    GLFWwindow* window = glfwCreateWindow(win_w, win_h, "Kalman Filter Simulation (ImGui/ImPlot)", monitor, NULL);
    if (!window) return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    // Kalman filter state
    kf.setState(x0, P0);
    std::vector<float> time, true_theta, meas_theta, est_theta;
    time.reserve(N_steps + 1);
    true_theta.reserve(N_steps + 1);
    meas_theta.reserve(N_steps + 1);
    est_theta.reserve(N_steps + 1);
    time.push_back(0.0f);
    true_theta.push_back(X_true(1, 0));
    meas_theta.push_back(z(1, 0));
    est_theta.push_back(x0(1));
    int k = 0;
    bool running = true;
    bool auto_play = false;
    float step_interval = 0.01f; // seconds per step
    float last_step_time = 0.0f;
    while (!glfwWindowShouldClose(window) && running) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2((float)win_w, (float)win_h));
        ImGui::Begin("Kalman Filter Real-Time Simulation", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);
        if (ImGui::Button("Step##step") && k < N_steps) {
            kf.predict(u_seq_noisy.col(k));
            kf.update(z.col(k));
            ++k;
            time.push_back(k * T);
            true_theta.push_back(X_true(1, k));
            meas_theta.push_back(z(1, k));
            est_theta.push_back(kf.getState()(1));
        }
        ImGui::SameLine();
        if (ImGui::Button(auto_play ? "Pause##play" : "Play##play")) auto_play = !auto_play;
        ImGui::SameLine();
        if (ImGui::Button("Run to End##run")) {
            while (k < N_steps) {
                kf.predict(u_seq_noisy.col(k));
                kf.update(z.col(k));
                ++k;
                time.push_back(k * T);
                true_theta.push_back(X_true(1, k));
                meas_theta.push_back(z(1, k));
                est_theta.push_back(kf.getState()(1));
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset##reset")) {
            k = 0;
            kf.setState(x0, P0);
            time.clear(); true_theta.clear(); meas_theta.clear(); est_theta.clear();
            time.push_back(0.0f);
            true_theta.push_back(X_true(1, 0));
            meas_theta.push_back(z(1, 0));
            est_theta.push_back(x0(1));
            auto_play = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Quit##quit")) running = false;
        // Do not call ImGui::SameLine() after the last button
        ImGui::Text("Step: %d / %d", k, N_steps);
        ImGui::SliderFloat("Speed (steps/sec)##speed", &step_interval, 0.001f, 0.2f, "%.3f");
        if (auto_play && k < N_steps) {
            float now = ImGui::GetTime();
            if (now - last_step_time > step_interval) {
                kf.predict(u_seq_noisy.col(k));
                kf.update(z.col(k));
                ++k;
                time.push_back(k * T);
                true_theta.push_back(X_true(1, k));
                meas_theta.push_back(z(1, k));
                est_theta.push_back(kf.getState()(1));
                last_step_time = now;
            }
        }
        if (ImPlot::BeginPlot("Theta (Angle)", ImVec2(-1, win_h * 0.8f))) {
            ImPlot::SetupAxes("Time [s]", "Theta", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -1.4, 1.4, ImPlotCond_Always);
            // True Theta: solid, thick, blue
            ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 3.0f);
            ImPlot::SetNextLineStyle(ImVec4(0,0.5f,1,1), 3.0f);
            ImPlot::PlotLine("True Theta", time.data(), true_theta.data(), (int)time.size());
            ImPlot::PopStyleVar();
            // Measured Theta: thick, gray
            ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
            ImPlot::SetNextLineStyle(ImVec4(0.5f,0.5f,0.5f,1), 2.0f);
            ImPlot::PlotLine("Measured Theta", time.data(), meas_theta.data(), (int)time.size());
            ImPlot::PopStyleVar();
            // Estimated Theta: thick, orange
            ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
            ImPlot::SetNextLineStyle(ImVec4(1,0.5f,0,1), 2.0f);
            ImPlot::PlotLine("Estimated Theta", time.data(), est_theta.data(), (int)time.size());
            ImPlot::PopStyleVar();
            ImPlot::EndPlot();
        }
        ImGui::End();
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}