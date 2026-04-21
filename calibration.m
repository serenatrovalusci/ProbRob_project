function calibration()
  %==========================================================================
  % 1. LOAD DATA, ENCODER DATA PREPROCESSING
  %==========================================================================
  filename = 'dataset.txt';
  [times, t_steer, t_traction, model_pose, tracker] = load_dataset(filename);

  n = length(times);
  delta_traction = zeros(n, 1);

  % Traction encoder delta calculation with overflow/wraparound handling
  for i = 1:n-1
      diff = t_traction(i+1) - t_traction(i);
      if diff < -2^31
          diff = diff + 2^32;   % Underflow wraparound
      end
      if diff > 2^31
          diff = diff - 2^32;   % Overflow wraparound
      end
      delta_traction(i+1) = diff;
  end

  % Convert t_steer from unsigned [0, 8191] to signed [-4096, 4095]
  t_steer_signed = zeros(n, 1); 
  for i = 1:n
      if t_steer(i) > 4096
          t_steer_signed(i) = t_steer(i) - 8192;
      else
          t_steer_signed(i) = t_steer(i);
      endif
  endfor

  ticks_vec = [t_steer_signed, delta_traction];  % n x 2

  %==========================================================================
  % 2. MODEL POSE ESTIMATE (NOMINAL PARAMETERS)
  %==========================================================================
  % Initial guess: [Ksteer, Ktraction, axis_length, steer_offset, x_sb, y_sb, theta_sb]
  x = [0.1; 0.0106141; 1.4; 0; 1.4; 0; 0];  

  T_base_pose = estimate_model_pose(x, ticks_vec);

  figure(1);
  clf; hold on;
  plot(model_pose(:,1), model_pose(:,2), 'r-', 'LineWidth', 1.5);
  plot(T_base_pose(:,1), T_base_pose(:,2), 'b--', 'LineWidth', 1.5);
  legend('Model Pose Ground Truth', 'Model Pose Estimate (Nominal)');
  xlabel('x [m]'); ylabel('y [m]');
  title('Nominal vs Ground Truth Model Pose Trajectory (Base)');
  grid on;

  %==========================================================================
  % 3. WARM START & SENSOR POSE ESTIMATE
  %==========================================================================
  % Estimate initial parameters to ensure the optimization starts near reality
  x_init = warm_start(ticks_vec, tracker, x);  
  
  ticks_vec_transposed = ticks_vec'; 
  T_sensor_pose = estimate_sensor_pose(x, ticks_vec_transposed);
  T_sensor_pose_init = estimate_sensor_pose(x_init, ticks_vec_transposed);

  figure(2);
  clf; hold on;
  plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 1.5);
  plot(T_sensor_pose(:,1), T_sensor_pose(:,2), 'b-', 'LineWidth', 1.5);
  plot(T_sensor_pose_init(:,1), T_sensor_pose_init(:,2), 'g-', 'LineWidth', 1.5);
  legend('Tracker GT', 'Nominal Params', 'Warm Start');
  xlabel('x [m]'); ylabel('y [m]');
  title('Pre-Calibration Trajectory Comparison');
  grid on;

  %==========================================================================
  % 4. NON-LINEAR LEAST SQUARES CALIBRATION
  %==========================================================================
  Z = [];
  for i = 1:n
      Z = [Z, [ticks_vec_transposed(:, i); tracker(i, :)']];
  endfor

  fprintf('\nStarting Optimization...\n');
  
  for it = 1:30
    [x_new, chi] = perform_calibration(x_init, Z);
    x_init = x_new; % Update parameters for next iteration
    printf('  Iteration %d, chi = %.6f\n', it, chi);
  end

  fprintf('\n--- Final Calibration Parameters ---\n');
  printf('k_steer: %.4f | k_traction: %.6f | b: %.3f\n', x_new(1), x_new(2), x_new(3));
  printf('offset:  %.4f | x_sb: %.3f | y_sb: %.3f | theta_sb: %.4f\n', x_new(4), x_new(5), x_new(6), x_new(7));

  %==========================================================================
  % 5. ANALYSIS OF RESULTS & ERROR PLOTTING
  %==========================================================================
  T_calibrated = estimate_sensor_pose(x_new, ticks_vec_transposed);
  
  % --- Error Calculation and RMSE ---
  err = T_calibrated - tracker;
  err(:,3) = atan2(sin(err(:,3)), cos(err(:,3))); % Normalize angular error
  
  rmse_xy = sqrt(mean(err(:,1).^2 + err(:,2).^2));
  rmse_th = sqrt(mean(err(:,3).^2));
  
  fprintf('\n--- Error Statistics ---\n');
  printf('RMSE xy = %.2f cm   RMSE theta = %.2f deg\n', 100*rmse_xy, rad2deg(rmse_th));

  % --- Trajectory Plot ---
  figure(4); clf; hold on;
  plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 1.5);
  plot(T_calibrated(:,1), T_calibrated(:,2), 'b-', 'LineWidth', 1.5);
  legend('Tracker (Ground Truth)', 'Calibrated Odometry');
  xlabel('x [m]'); ylabel('y [m]');
  title('Trajectory Comparison Post-Calibration');
  grid on;

  % --- Error Analysis Plot ---
  figure(5); clf;
  subplot(2,1,1);
  plot(sqrt(err(:,1).^2 + err(:,2).^2), 'LineWidth', 1.2, 'Color', [0 0.4470 0.7410]);
  grid on;
  ylabel('Position Error [m]');
  title(sprintf('Temporal Error Analysis (RMSE XY: %.2f cm)', 100*rmse_xy));

  subplot(2,1,2);
  plot(rad2deg(err(:,3)), 'LineWidth', 1.2, 'Color', [0.8500 0.3250 0.0980]);
  grid on;
  xlabel('Sample Index'); ylabel('Theta Error [deg]');
  line([0 length(err)], [0 0], 'Color', 'k', 'LineStyle', '--');

  % --- SAVE PLOTS TO DISK (Low Resolution) ---
  fprintf('\nSaving plots to current directory...\n');
  print(1, '01_base_trajectory.png', '-dpng', '-r72');
  print(2, '02_pre_calibration.png', '-dpng', '-r72');
  print(4, '04_final_trajectory.png', '-dpng', '-r72');
  print(5, '05_error_analysis.png', '-dpng', '-r72');
  fprintf('Plots saved successfully.\n');

endfunction

%==============================================================================
% LOCAL FUNCTIONS
%==============================================================================

function [times, t_steer, t_traction, model_pose, tracker] = load_dataset(filename)
  fid = fopen(filename, 'r');
  times = []; t_steer = []; t_traction = []; model_pose = []; tracker = [];
  while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line) || isempty(line) || line(1) == '#', continue; endif
    vals = sscanf(line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f');
    if length(vals) == 9
      times = [times; vals(1)]; 
      t_steer = [t_steer; vals(2)]; 
      t_traction = [t_traction; vals(3)];
      model_pose = [model_pose; vals(4), vals(5), vals(6)]; 
      tracker = [tracker; vals(7), vals(8), vals(9)];
    endif
  endwhile
  fclose(fid);
endfunction

function h_odom = estimate_model_pose(x, ticks)  
  k_steer = x(1); ktraction = x(2); b = x(3); steer_offset = x(4);
  n = length(ticks);
  xp = zeros(n,1); yp = zeros(n,1); thetap = zeros(n,1);
  dl = (ktraction/5000) * ticks(:,2);
  dphi = (k_steer*2*pi/8192) * ticks(:,1) + steer_offset;
  h_odom = zeros(n, 3);
  h_odom(1, :) = [0, 0, 0];
  for i = 2:n
    xp(i) = xp(i-1) + dl(i) * cos(thetap(i-1))*cos(dphi(i));
    yp(i) = yp(i-1) + dl(i) * sin(thetap(i-1))*cos(dphi(i));
    thetap(i) = thetap(i-1) + dl(i)*sin(dphi(i))/b;
    h_odom(i, :) = [xp(i), yp(i), thetap(i)];
  endfor
endfunction

function h_odom = estimate_sensor_pose(x, ticks)
  k_steer = x(1); ktraction = x(2); b = x(3); steer_offset = x(4);
  x_sb = x(5); y_sb = x(6); theta_sb = x(7);
  n = size(ticks, 2);
  xp = zeros(n, 1); yp = zeros(n, 1); thetap = zeros(n, 1);
  dl = (ktraction/5000) * ticks(2,:)';
  dphi = (k_steer*2*pi/8192) * ticks(1,:)' + steer_offset;
  Tbs = [cos(theta_sb) -sin(theta_sb) x_sb; sin(theta_sb) cos(theta_sb) y_sb; 0 0 1];
  h_odom = zeros(n, 3);
  h_odom(1, :) = [xp(1), yp(1), thetap(1)];
  for i = 2:n
    xp(i) = xp(i-1) + dl(i) * cos(thetap(i-1))*cos(dphi(i));
    yp(i) = yp(i-1) + dl(i) * sin(thetap(i-1))*cos(dphi(i));
    thetap(i) = thetap(i-1) + dl(i)*sin(dphi(i))/b;
    Twb = v2t([xp(i), yp(i), thetap(i)]);
    h_odom(i, :) = t2v(Twb * Tbs)';
  endfor
endfunction

function [x_new, chi] = perform_calibration(x, Z)
  H = zeros(7, 7); b = zeros(7, 1);
  nmeas = size(Z, 2);
  kernel_threshold = 2;
  ticks = Z(1:2, :); meas_m = Z(3:5, :); meas_m(1:3, 1) = [0;0;0]; 
  pred_m = estimate_sensor_pose(x, ticks)';            
  J_all = precompute_jacobians(x, ticks);               
  chi = 0;
  for i = 1:50:nmeas
    e = pred_m(:, i) - meas_m(:, i);
    e(3) = atan2(sin(e(3)), cos(e(3))); 
    chi_i = e' * e; 
    if(chi_i > kernel_threshold)
        e *= sqrt(kernel_threshold / chi_i);
        chi_i = kernel_threshold;
    endif                    
    J = J_all(:, :, i);        
    H += J' * J; b += J' * e; chi += chi_i;
  endfor
  dx = -H\b; x_new = x + dx;
endfunction

function J_all = precompute_jacobians(x, ticks)
  nmeas = size(ticks, 2); J_all = zeros(3, 7, nmeas);
  epsilon = 1e-3; inv_eps2 = 0.5 / epsilon;
  for j = 1:7
    e_vec = zeros(7, 1); e_vec(j) = epsilon;
    J_plus  = estimate_sensor_pose(x + e_vec, ticks)';
    J_minus = estimate_sensor_pose(x - e_vec, ticks)';
    J_all(:, j, :) = inv_eps2 * (J_plus - J_minus);
  endfor
endfunction

function x = warm_start(ticks, tracker, x_nom)
  k_s_nom = x_nom(1); k_t_nom = x_nom(2); b_nom = x_nom(3);
  t_steer = ticks(:, 1);
  dtrac   = ticks(:, 2);
  steer_ang = (k_s_nom * 2 * pi / 8192) * t_steer;
  L_trk  = sum(sqrt(sum(diff(tracker(:, 1:2)).^2, 2)));
  L_odom = sum(abs(dtrac .* cos(steer_ang))) * k_t_nom / 5000;
  k_t    = k_t_nom * L_trk / max(L_odom, 1e-9);
  dth_trk = diff(tracker(:, 3)); tot_trk = sum(abs(atan2(sin(dth_trk), cos(dth_trk))));
  tot_odom = sum(abs(dtrac * k_t / 5000 .* sin(steer_ang) / b_nom));
  k_s      = k_s_nom * tot_trk / max(tot_odom, 1e-9);
  max_t = max(abs(t_steer));
  if k_s * 2 * pi / 8192 * max_t > pi / 3, k_s = (pi/3)*8192/(2*pi*max_t); end
  x = [k_s; k_t; b_nom; 0; 1.5; 0; 0];
endfunction

function A = v2t(v)
  c = cos(v(3)); s = sin(v(3));
  A = [c, -s, v(1); s, c, v(2); 0, 0, 1];
endfunction

function v = t2v(A)
  v = [A(1, 3); A(2, 3); atan2(A(2, 1), A(1, 1))];
endfunction