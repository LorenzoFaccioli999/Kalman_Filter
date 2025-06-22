
# 📦 Kalman Filter Project (C++ & Python)

![kalman_estimation](images/kalman_RLS_estimation.png)

This repository demonstrates the use of **Kalman filters** in both simulated and real-time contexts, using:

- ✅ **C++** for real-time, interactive applications (with ImGui + ImPlot)
- ✅ **Python** for didactic purposes and simulations

---

## 🧠 Features

- 📉 Recursive Least Squares (RLS) Kalman filter implementation
- 📐 Discrete-Time Kalman Filter (DTKF) implementation
- 📊 Real-time visualization using **ImGui + ImPlot**
- 🎛️ Interactive measurement noise tuning (`R`) via GUI slider
- 🧪 Modular, testable C++ design

---

## 🔧 Dependencies

### ✅ C++ Dependencies

#### System Tools
- CMake ≥ 3.14
- C++17-compliant compiler (e.g. MinGW, MSVC)
- Ninja (recommended)

#### Libraries via [vcpkg](https://github.com/microsoft/vcpkg)

Make sure `vcpkg` is installed and its root is in your environment variable `VCPKG_ROOT`.

Install the required libraries:

```bash
vcpkg install eigen3:x64-mingw-static
vcpkg install glfw3:x64-mingw-static
```

> If you're using MSVC instead of MinGW, replace `x64-mingw-static` with `x64-windows`.

#### External Dependencies (imgui & implot)

This project uses [Dear ImGui](https://github.com/ocornut/imgui) and [ImPlot](https://github.com/epezent/implot) for visualization.

Clone them manually into your C++ project folder:

```bash
cd cpp
mkdir external
cd external
git clone https://github.com/ocornut/imgui
git clone https://github.com/epezent/implot
```

You should end up with:

```
cpp/
└── external/
    ├── imgui/
    └── implot/
```

---

### ✅ Python Dependencies

To run the Python simulations:

```bash
# Create and activate a virtual environment
python -m venv venv
source venv/bin/activate      # On Linux/macOS
venv\Scripts\activate.bat   # On Windows

# Install required dependencies
pip install -r requirements.txt
```

This will allow you to run all the Jupyter notebooks and simulations included in the Python portion.

---

## 🛠️ Build Instructions

### 🛠️ C++ 

From the project root (`Kalman_Filter/`), run:

```bash
mkdir build
cd build

cmake .. ^
  -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake ^
  -DVCPKG_TARGET_TRIPLET=x64-mingw-static

cmake --build .
```


---

## 🚀 Running the Application

### C++ Executable

```bash
./main_sim_RLS.exe
```

- Runs a live simulation of a Kalman filter (RLS variant)
- Opens a GUI window with real-time plots
- Lets you interactively tune the measurement noise `R`
- Visualizes noisy measurements, filter estimates, and the true value

---

## 💡 Notes

- Ensure that your compiler and vcpkg triplet (`x64-mingw-static` or `x64-windows`) match your system.
- `VCPKG_ROOT` should point to the root of your vcpkg installation.
- On Windows, run the C++ app from a terminal with `vcpkg` and your compiler in the `PATH`.
