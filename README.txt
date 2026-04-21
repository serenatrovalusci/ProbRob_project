# Front-Tractor Odometry Calibration

This project implements a full calibration pipeline for a **front-steering, front-traction tricycle robot** using encoder readings and an external tracker (ground truth) as reference. The goal is to recover seven unknown parameters that jointly describe the robot's kinematics and the pose of the tracking sensor with respect to the base frame.

The code is written in **Octave / MATLAB** and is entirely contained in `calibration.m`. It works end-to-end: it loads the raw log, preprocesses the encoder data, builds a first-guess trajectory, runs a non-linear least-squares optimization, and finally saves the diagnostic plots in the `plots/` directory.

## Parameters Being Estimated

The state vector `x` contains 7 unknowns:

| Index | Symbol        | Meaning                                              |
|-------|---------------|------------------------------------------------------|
| 1     | `k_steer`     | Steering encoder gain (ticks → radians scaling)      |
| 2     | `k_traction`  | Traction encoder gain (ticks → meters scaling)       |
| 3     | `b`           | Wheelbase / axis length                              |
| 4     | `steer_offset`| Mechanical zero offset of the steering encoder      |
| 5     | `x_sb`        | Sensor-to-base translation along x                   |
| 6     | `y_sb`        | Sensor-to-base translation along y                   |
| 7     | `theta_sb`    | Sensor-to-base rotation                              |

The first four parameters describe the **kinematic model** of the vehicle, the last three describe the **extrinsic calibration** between the tracked sensor and the robot base.

## Pipeline Overview

The solution is organized in five logical stages that mirror the section headers in the main function.

### 1. Data Loading and Encoder Preprocessing

The dataset is parsed with a simple line-by-line reader (`load_dataset`) that extracts time, steer ticks, traction ticks, the vendor-provided model pose, and the ground-truth tracker pose.

Two preprocessing steps are critical before anything else can work:

- **Tracker re-referencing**: the tracker is expressed in an arbitrary world frame. I re-express every pose with respect to the initial one by pre-multiplying by `inv(v2t(tracker(1,:)))`, so the reference trajectory starts exactly at `[0,0,0]`. This makes it directly comparable to the odometry, which always starts at the origin.

- **Traction encoder overflow handling**: the traction encoder is a 32-bit unsigned counter that wraps around. I compute incremental deltas `Δticks = t(i+1) − t(i)` and fix wrap-arounds by adding/subtracting `2^32` whenever the jump exceeds `±2^31`. This restores a clean, monotonic signal.

- **Steer encoder sign conversion**: the steering encoder is a 13-bit absolute counter in the range `[0, 8191]`. I remap values above `4096` to the negative half `[−4096, −1]` so that the signal behaves as a signed angle around zero.

### 2. Model Pose Estimate with Nominal Parameters

Using the initial guess provided by the project statement (`k_steer=0.1`, `k_traction=0.0106141`, `b=1.4`, etc.), I integrate the bicycle-like kinematic model:

```
Δl   = (k_traction / 5000) · Δticks_traction
Δφ   = (k_steer · 2π / 8192) · ticks_steer + steer_offset

x_{i}     = x_{i−1}     + Δl · cos(θ_{i−1}) · cos(Δφ)
y_{i}     = y_{i−1}     + Δl · sin(θ_{i−1}) · cos(Δφ)
θ_{i}     = θ_{i−1}     + Δl · sin(Δφ) / b
```

This gives a first estimate of the base trajectory (`estimate_model_pose`) and is plotted against the dataset's `model_pose` as a sanity check (Figure 1).

### 3. Initial State and Sensor Pose Estimate

Starting non-linear optimization from a bad initial guess can easily get stuck in local minima or produce diverging steps, so I added a dedicated routine (`initial_state`) that rescales the two most sensitive gains before handing the problem to Gauss-Newton:

- `k_traction` is scaled so that the **total arc length** traced by the odometry matches the total arc length of the tracker trajectory.
- `k_steer` is then scaled so that the **total absolute heading variation** predicted by the odometry matches the one measured by the tracker (using `atan2(sin, cos)` to normalize angles).

This is a data-driven heuristic: it gives the optimizer a warm start where the predicted trajectory already has roughly the right length and curvature, leaving only the fine geometric corrections (wheelbase, offset, sensor pose) to be recovered by the solver.

The full sensor pose predictor is `estimate_sensor_pose`, which integrates the same kinematic model as before but then composes each base pose with the sensor-to-base transform `T_bs = v2t([x_sb, y_sb, theta_sb])`. The final trajectory is re-anchored to start at `[0,0,0]`, so that the optimizer does not have to compensate for a constant translational offset of roughly 1.4 m due to the sensor extrinsic.

Figure 2 shows the tracker, the trajectory predicted with the nominal parameters, and the one predicted with the initial-state parameters — the improvement is already substantial.

### 4. Non-Linear Least-Squares Calibration

The core of the project is a **Gauss-Newton optimizer** (`perform_calibration`) that minimizes the squared error between the predicted sensor trajectory and the tracker.

For each measurement I build:

- the error `e = h(x) − z`, with the angular component normalized via `atan2(sin, cos)`,
- the Jacobian `J = ∂h/∂x` (numerical, see below),
- the information contribution `H += J'·J`, `b += J'·e`.

A **robust kernel** (Huber-like) is applied whenever `||e||² > kernel_threshold`: the error is rescaled by `sqrt(threshold / chi_i)` so that outliers cannot dominate the normal equations. This stabilizes the optimization in the presence of noisy tracker samples.

At the end of each iteration the system `H·Δx = −b` is solved and the update is applied as `x ← x + Δx`. The loop runs for 30 iterations, printing the total `chi` at every step so that convergence can be monitored from the console.

**Jacobians via central finite differences.** Deriving analytical Jacobians for the composed base-plus-sensor model is tedious and error-prone, so I compute them numerically in `precompute_jacobians`:

```
J(:, j) ≈ ( h(x + ε·e_j) − h(x − ε·e_j) ) / (2·ε),   ε = 1e−3
```

Computing them once per outer iteration (and reusing them inside the inner accumulation loop) keeps the runtime reasonable. To further reduce cost, the inner loop over measurements subsamples every 50th sample (`for i = 1:50:nmeas`), which is more than enough to constrain 7 parameters while speeding things up considerably.

### 5. Results and Error Analysis

After calibration, the full-resolution predicted trajectory is regenerated with the final parameters and compared against the tracker:

- **Trajectory plot** (Figure 4) overlays the calibrated odometry and the ground truth.
- **Temporal error plot** (Figure 5) shows the position error magnitude `sqrt(ex² + ey²)` and the heading error in degrees, both as a function of the sample index.
- **Global metrics**: RMSE on `(x, y)` (cm) and RMSE on `θ` (deg) are printed to the console.

All figures are saved to `plots/` as low-resolution PNGs (`-r72`) so they can be inspected without re-running the script.

#### Final Output

Running the pipeline on `dataset.txt` produces the following results:

```
--- Final Calibration Parameters ---
k_steer: 0.5875 | k_traction: 0.011514 | b: 1.696
offset:  -0.0729 | x_sb: 1.796 | y_sb: 0.027 | theta_sb: 0.0004

--- Error Statistics ---
RMSE xy = 9.13 cm   RMSE theta = 2.54 deg
```

A few observations on the recovered parameters:

- `k_steer` moves from the nominal `0.1` to `0.5875`, confirming that the warm-start heuristic on total heading variation was essential to unlock the correct scale.
- `k_traction` is refined from `0.0106141` to `0.011514` (about +8.5%), which closes the arc-length mismatch between odometry and tracker.
- The wheelbase `b` settles at `1.696 m`, slightly larger than the nominal `1.4 m`.
- The sensor extrinsic `[x_sb, y_sb, theta_sb] ≈ [1.796, 0.027, 0.0004]` indicates that the tracker is mounted roughly `1.8 m` forward of the base origin, almost perfectly centered laterally and with negligible rotation — physically plausible for a sensor placed on the front of the vehicle.
- The residual error (`9.13 cm` on position, `2.54°` on heading) is consistent with the noise level of the tracker and the simplicity of the kinematic model.

## Key Techniques Used

- **Rigid transform utilities** (`v2t`, `t2v`) for moving between vector and homogeneous-matrix representations of SE(2) poses.
- **Frame re-anchoring** of both the tracker and the predicted sensor trajectory to `[0,0,0]`, which removes a constant bias that would otherwise be absorbed into the extrinsic parameters and slow down convergence.
- **Encoder unwrapping** for the 32-bit traction counter and **sign conversion** for the 13-bit steering counter.
- **Data-driven warm start** that rescales the two dominant gains before optimization.
- **Gauss-Newton** with a **Huber-style robust kernel** to downweight outliers.
- **Numerical Jacobians** (central differences) to avoid hand-derived expressions.
- **Measurement subsampling** in the inner loop for efficiency without loss of calibration quality.
- **Angular error normalization** via `atan2(sin, cos)` wherever heading differences are computed.

## How to Run

```octave
>> calibration
```

The script expects the file `dataset.txt` to be in the working directory. Upon completion it prints:

- the chi value at every Gauss-Newton iteration,
- the final 7-parameter vector,
- the RMSE on position and heading,

and writes the four diagnostic plots to the `plots/` subfolder.

## File Layout

```
.
├── calibration.m      # full pipeline (entry point + local functions)
├── dataset.txt        # input log: time, ticks, model pose, tracker pose
├── plots/             # auto-generated diagnostic figures
│   ├── 01_base_trajectory.png
│   ├── 02_pre_calibration.png
│   ├── 04_final_trajectory.png
│   └── 05_error_analysis.png
└── README.md
```