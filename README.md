# 📦 Kalman Filter Project (C++ & Python)

![kalman_estimation](images/kalman_RLS_estimation.png)

This repository demonstrates the use of **Kalman filters** in both simulated and real-time contexts, using:

- ✅ **C++** for real-time, interactive applications (with ImGui + ImPlot)
- ✅ **Python** for didactic and batch-style simulations

---

## 🧠 Features

- 📉 Recursive Least Squares (RLS) Kalman filter implementation
- 📊 Real-time visualization using **ImGui + ImPlot**
- 🎛️ Interactive measurement noise tuning (`R`) via GUI slider
- 🧪 Modular, testable C++ design

---

## 🔧 Dependencies

You will need the following to build the C++ portion of the project.

### ✅ System Tools
- CMake ≥ 3.14
- C++17-compliant compiler (e.g. MinGW, MSVC)
- Ninja (recommended)

### ✅ Libraries via [vcpkg](https://github.com/microsoft/vcpkg)

Make sure `vcpkg` is installed and its root is in your environment variable `VCPKG_ROOT`.

Install the required libraries:

```bash
vcpkg install eigen3:x64-mingw-static
vcpkg install glfw3:x64-mingw-static
```

> If you use MSVC instead of MinGW, replace `x64-mingw-static` with `x64-windows`.

---

## 📁 External Dependencies (imgui & implot)

This project uses [Dear ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot) for visualization.

These are not included in the repository — you must manually clone them:

```bash
cd cpp
mkdir external
cd external
git clone https://github.com/ocornut/imgui
git clone https://github.com/epezent/implot
```

After this, you should have:

```
cpp/
└── external/
    ├── imgui/
    └── implot/
```

---

## 🛠️ Build Instructions

From the project root (`Kalman_Filter/`), run:

```bash
mkdir build
cd build

cmake .. `
  -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-mingw-static

cmake --build .
```

## 🚀 Running the Application

```bash
./main_sim_RLS.exe
```

- Runs a live simulation of a Kalman filter (RLS variant)
- Opens a GUI window with real-time plots
- Lets you interactively tune the measurement noise `R`
- Visualizes noisy measurements, filter estimates, and the true value

---

## 💡 Notes

- Make sure the correct compiler and triplet (`x64-mingw-static` or `x64-windows`) match your system.
- The `VCPKG_ROOT` environment variable should point to the root directory of your [vcpkg](https://github.com/microsoft/vcpkg) installation

- If you're on Windows, launch from a terminal where `vcpkg` and your compiler are available in `PATH`.

 
