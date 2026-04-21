
**Kinematic Calibration for a Tricycle-Drive Robot**

The goal of this project is to estimate the intrinsic kinematic parameters and the extrinsic sensor-to-base transformation to minimize the error between the robot's odometry and the ground truth.

** Kinematic Model **
The robot utilizes a front-wheel traction and steering system. For this project, the kinematic state (x,y,θ) is defined on the kinematic center located at the middle of the rear wheel axis.

** Methodology **
Evolution of the Approach

Initially, I attempted to solve the optimization using relative poses of the base link. However, this approach proved to be overly chaotic due to the accumulation of noise and the sensitivity of incremental measurements.

I decided to transition to absolute poses. By minimizing the error in the global trajectory, the system handles sensor noise much better and ensures a more stable, less "chaotic" convergence during the non-linear optimization process. I abandoned the first method because it was too intricate considering the data available in the dataset.

** Minimization Algorithm **

The calibration uses a Non-Linear Least Squares approach (Gauss-Newton). Since the motion equations are non-linear, I used Numerical Jacobians to determine how the trajectory reacts to changes in each parameter.

To handle outliers like wheel slips or tracker glitches, I implemented a Robust Kernel. If the error for a specific sample exceeds a threshold, the system scales down its weight. This prevents "bad" data points from distorting the final results.

** Efficiency via Subsampling **

The dataset contains a high volume of redundant information due to the high sampling frequency of the sensors. I realized that considering every single point made the optimization unnecessarily slow and noisy.

By implementing a subsampling factor of 50, I ignored these repeated or near-identical samples. This allowed the optimizer to focus on the significant geometric changes of the trajectory, leading to a much faster and more stable convergence.

** The "Warm Start" Breakthrough **

During testing, I realized that the optimization process alone wouldn't converge. The "distance" between the default nominal parameters and the actual physical reality was too great for the non-linear solver to bridge, the optimizer would get stuck in local minima or diverge immediately.

To solve this, I implemented a "Warm Start" phase. This analytical step pre-calculates the gains before the optimization begins by comparing total distance and total rotation between the tracker and the ticks. This brought the initial guess close enough to the global minimum that the non-linear solver could then successfully fine-tune the parameters.



** Calibration Outputs **

** Results **
Final Calibrated Parameters

k_steer: 0.5197 rad/tick

k_traction: 0.008308 m/tick

base_line (b): 1.150 m

steer_offset: -0.1233 rad

Sensor Pose (wrt base link):

x_sb=1.115 m

y_sb=0.670 m

θ_sb=0.5419 rad

** Numerical Errors **

RMSE XY: 148.34 cm

RMSE Theta: 14.85 deg

** Project Structure & Evaluation **
calibration.m: Main logic and optimization script.

Output Plots (PNGs):

01_base_trajectory.png: Nominal base-link motion.

02_pre_calibration.png: Visualizing the Warm Start improvement.

04_final_trajectory.png: Final calibrated odometry vs. Ground Truth.

05_error_analysis.png: Temporal breakdown of residual errors.

** Usage **
Run the calibration() function in Octave or MATLAB. The script automatically processes dataset.txt, handles uint32 encoder overflows, performs 30 iterations of the solver, and saves the plots to the disk.