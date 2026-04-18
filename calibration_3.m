function calibration_3()

  %==========================================================
  % 1. LOAD DATA, ENCODER DATA PREPROCESSING 
  %=========================================================
  filename = 'dataset.txt';
  [times, t_steer, t_traction, model_pose, tracker] = load_dataset(filename);

  printf('times:      %d x %d\n', size(times, 1),      size(times, 2));
  printf('t_steer:    %d x %d\n', size(t_steer, 1),    size(t_steer, 2));
  printf('t_traction: %d x %d\n', size(t_traction, 1), size(t_traction, 2));
  printf('model_pose: %d x %d\n', size(model_pose, 1), size(model_pose, 2));
  printf('tracker:    %d x %d\n', size(tracker, 1),    size(tracker, 2));

  n = length(times);
  delta_traction = zeros(n, 1);

  for i = 1:n-1
      diff = t_traction(i+1) - t_traction(i);
      
      % Correzione overflow
      if diff < -2^31
          diff = diff + 2^32;   % wraparound
      end
      
      delta_traction(i+1) = diff;
  end

  t_steer_signed = zeros(n, 1);     % convert t_steer from unsigned to signed
  for i = 1:n
      if t_steer(i) > 4096
          t_steer_signed(i) = t_steer(i) - 8192;
      else 
          t_steer_signed(i) = t_steer(i);
      endif
  endfor

  ticks_vec = [t_steer_signed, delta_traction];  % n x 2


  %=========================================================================================
  % 2. ESTIMATE MODEL POSE (wrt world) WITH KINEMATIC MODEL and NOMINAL PARAMETERS.
  %=========================================================================================

  x = [0.1; 0.0106141; 1.4; 0];  % initial guess for the parameters : [ Ksteer Ktraction axis_length steer_offset ]

  T = zeros(n, 3);                  % estimated model pose (x, y, theta) for each measurement
  prev_pose = [0 0 0];              % assuming the robot starts at the origin with zero orientation
  for i = 1:n
    ticks = [t_steer_signed(i), delta_traction(i)];
    next_pose = estimate_model_pose(x, ticks, prev_pose);
    T(i, :) = next_pose;
    prev_pose = next_pose;
  endfor

  % Some debugging prints 
  %for i= 120:130
    %printf('row %d, x_p = %.6f, y_p = %.6f, theta_p = %.6f\n, x = %.6f, y = %.6f, theta = %.6f\n ', i, T(i, 1), T(i, 2), T(i, 3), model_pose(i, 1), model_pose(i, 2), model_pose(i, 3));
  %endfor
  %for i= 120:130
    %printf('row %d, t traction %d \n', i,t_traction(i));
  %endfor
  %for i= 120:130
    %printf('row %d, t steer %d \n', i, t_steer(i));
  %endfor
  %for i= 120:130
    %printf('row %d, delta traction %d \n', i, delta_traction(i));
  %endfor
  %for i= 1:n
    %e(i) = (T(i, 1) - model_pose(i, 1))^2 + (T(i, 2) - model_pose(i, 2))^2 + (T(i, 3) - model_pose(i, 3))^2;
    %if e(i) > 1e-5
      %printf('row %d, x_p = %.6f, y_p = %.6f, theta_p = %.6f\n, x = %.6f, y = %.6f, theta = %.6f\n ', i, T(i, 1), T(i, 2), T(i, 3), model_pose(i, 1), model_pose(i, 2), model_pose(i, 3));
      %break;
    %endif
  %endfor

  figure(1);   
  clf;
  hold on;
  plot(model_pose(:,1), model_pose(:,2), 'r-', 'LineWidth', 1);
  plot(T(:,1), T(:,2), 'b-', 'LineWidth', 1);
  legend('Model Pose', 'Estimated Model Pose');
  xlabel('x [m]');
  ylabel('y [m]');    
  title('Trajectory of Model Pose');
  grid on;

  %========================================================================================
  % 3. PLOT TRACKER POSE (wrt world) AND BIASED MODEL POSE ESTIMATE.
  %========================================================================================
  
  figure(2);
  clf;
  hold on;
  plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 1);
  plot(T(:,1), T(:,2), 'b-', 'LineWidth', 1);
  legend('Tracker', 'Biased Model Pose');
  xlabel('x [m]');
  ylabel('y [m]');
  title('Trajectory comparison');
  grid on;

  %==========================================================
  % 4. CALIBRATION WITH GAUSS-NEWTON
  %==========================================================
  delta_tracker = relative_pose(tracker);

  x = [0.1; 0.0106141; 1.4; 0];

  U = zeros(n, 3);
  for i = 1:n
    U(i, :) = h_odom(x,ticks_vec(i,:));
  endfor

  Traj_relative_biased = compute_odometry_trajectory(U);

  %build the measurement matrix Z for the increments, with subsampling

  Z = [];
  k = 1;
  for i = 1:n
    if abs(delta_traction(i))   % subsampling condition, to select only the measurements with significant movement
      Z(1:2, k) = ticks_vec(i, :)';      % o adattare se è vettore
      Z(3:5, k) = delta_tracker(i,:)';
      k = k + 1;
    endif  
  endfor

    % Perform one round of Gauss-Newton optimization
   
    for i = 1:20
        [x_new, chi] = oneRound(x, Z);
        printf('Round %d: chi = %.6f\n', i, chi);
        x = x_new;
      endfor
        
      printf('Updated parameters: k_steer = %.4f, k_traction = %.6f, b = %.2f, steer_offset = %.2f\n', x_new(1), x_new(2), x_new(3), x_new(4));

  U_cal = zeros(n, 3);
  for i = 1:n
    U_cal(i, :) = h_odom(x_new,ticks_vec(i,:));
  endfor

  Tbs = [1 0 1.5;
         0 1 0;
         0 0 1];  
  Traj_relative_calibrated = compute_odometry_trajectory(U_cal);


  figure(3);
  clf;
  hold on;
  plot(Traj_relative_calibrated(:,1), Traj_relative_calibrated(:,2), 'b-', 'LineWidth', 1);
  plot(Traj_relative_biased(:,1), Traj_relative_biased(:,2), 'g-', 'LineWidth', 1);
  plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 1);
  legend('Odometry (calibrated)', 'Odometry(biased)', 'Tracker');
  xlabel('x [m]');
  ylabel('y [m]');
  title('Trajectory comparison');
  grid on;


endfunction  

%=========================================================
%LOCAL FUNCTIONS
%=========================================================

% Load data

function [times, t_steer, t_traction, model_pose, tracker] = load_dataset(filename)
  fid   = fopen(filename, 'r');
  times      = [];
  t_steer    = [];
  t_traction = [];
  model_pose = [];
  tracker    = [];

  while ~feof(fid)                           % while not end of file
    line = fgetl(fid);

    % skip comments and empty lines
    if ~ischar(line) || isempty(line) || line(1) == '#'
      continue;
    endif

    % parse the line directly with sscanf
    % format: time: X ticks: X X model_pose: X X X tracker_pose: X X X
    vals = sscanf(line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f');

    if length(vals) == 9                    %append
      times      = [times;      vals(1)];
      t_steer    = [t_steer;    vals(2)];
      t_traction = [t_traction; vals(3)];
      model_pose = [model_pose; vals(4), vals(5), vals(6)];
      tracker    = [tracker;    vals(7), vals(8), vals(9)];
    endif
  endwhile

  fclose(fid);
  printf('Loaded %d measurements\n', length(times));
endfunction

% Estimate Model Pose

function next_pose= estimate_model_pose(x,ticks,prev_state)

  k_steer = x(1);
  ktraction = x(2);
  b = x(3);
  steer_offset = x(4);

  t_steer = ticks(1);
  delta_traction = ticks(2);
  
  x_prev = prev_state(1);
  y_prev = prev_state(2);
  theta_prev= prev_state(3);

  v = (ktraction/5000) * delta_traction;
  phi = (k_steer*2*pi/8192) * t_steer + steer_offset;


  x_next = x_prev + v * cos(theta_prev);
  y_next = y_prev + v * sin(theta_prev);
  theta_next = theta_prev + v*phi/b;

  next_pose = [x_next, y_next, theta_next];
 
endfunction


%Estimate Odometry increment 

function delta = h_odom(x, ticks)
  k_steer      = x(1);
  k_traction   = x(2);
  b            = x(3);
  steer_offset = x(4);

  t_steer          = ticks(1);
  delta_traction   = ticks(2);   % già il delta, calcolato fuori

  dl = (k_traction/5000) * delta_traction;
  dph = (k_steer*2*pi/8192) * t_steer + steer_offset;

  dth = dl * dph / b;
  dx  = dl * cos(dth);
  dy  =  sin(dth);

  delta= [dx; dy; dth];
  
endfunction

% Convert between vector and homogeneous transformation matrix

function A=v2t(v)
    c = cos(v(3));
    s = sin(v(3));

  A = [ c, -s, v(1);
        s,  c, v(2);
        0,  0, 1];
end

function v=t2v(A)
    v(1:2, 1) = A(1:2, 3);
    v(3, 1)   = atan2(A(2,1), A(1,1));
end

function delta_tracker = relative_pose (tracker)
  delta_tracker = zeros(size(tracker));
  for i = 2:size(tracker, 1)
    delta_tracker(i, :) = t2v(inv(v2t(tracker(i-1,:))) * v2t(tracker(i,:)));;
  endfor
endfunction


function [e, J] = errorAndJacobian(x, z)
  ticks = z(1:2);   % 2 ticks
  meas  = z(3:5);   % measurements
  pred  = h_odom(x, ticks);
  e     = pred - meas;
  %e(3) = atan2(sin(e(3)), cos(e(3)));  % normalize angle error to [-pi, pi]
  J     = zeros(3, 4);
  epsilon  = 1e-3;
  inv_eps2 = 0.5 / epsilon;
  for i = 1:4
    e_vec    = zeros(4, 1);
    e_vec(i) = epsilon;
    J(:, i)  = inv_eps2 * (h_odom(x + e_vec, ticks) - h_odom(x - e_vec, ticks));
  endfor
endfunction

function [x_new, chi] = oneRound(x, Z)
  H     = zeros(4, 4);
  b     = zeros(4, 1);
  nmeas = size(Z, 2);
  chi   = 0;
  lambda = 1e-3;  % damping factor for Levenberg-Marquardt
  for i = 1:nmeas
    [e, J] = errorAndJacobian(x, Z(:, i));
    H   += J' * J;
    b   += J' * e;
    chi += e' * e;   %'
  endfor
  dx    = -(H + lambda*eye(4)) \ b;
  x_new = x + dx;
endfunction

function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);   % pre-allocate the trajectory matrix 
	current_T=v2t(zeros(1,3)); % converts vector into a rotation matrix full of zeros (initial pose)
	for i=1:size(U,1),
		u=U(i,1:3)' ;  %'

		current_T = current_T * v2t(u);  % (world) current_T(i) = (world) current_T (i-1) * (i-1) relative pose (i)
		T(i,1:3)=t2v(current_T)';  %' converts the current pose into a vector and stores it in the trajectory matrix as a row vector
	end
end

