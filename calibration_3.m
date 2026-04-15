function calibration_3()

    %==========================================================
    % LOAD DATA
    %=========================================================
    filename = 'dataset.txt';
    [times, t_steer, t_traction, model_pose, tracker] = load_dataset(filename);

    n = length(times);
    delta_traction = zeros(n, 1);
    for i = 1:n-1
        diff = t_traction(i+1) - t_traction(i);
        if diff < 0 
          if diff < -2^31
            diff = diff + 2^32;
          else
            diff = 0
          endif
        endif
        delta_traction(i+1) = diff;      %this vector dimension is n-1, instead of modifying all the dimensions i choose to add one extra zero at the beginning
    endfor

    delta_traction = delta_traction/5000;
    steer_tick = t_steer / 8192;

    figure(1);   
    clf;
    hold on;
    plot(delta_traction, 'r-', 'LineWidth', 2);
    legend('Delta Traction');
    xlabel('x');
    ylabel('y [m]');    
    title('delta');
    grid on;
    figure;


    printf('times:      %d x %d\n', size(times, 1),      size(times, 2));
    printf('t_steer:    %d x %d\n', size(t_steer, 1),    size(t_steer, 2));
    printf('t_traction: %d x %d\n', size(t_traction, 1), size(t_traction, 2));
    printf('model_pose: %d x %d\n', size(model_pose, 1), size(model_pose, 2));
    printf('tracker:    %d x %d\n', size(tracker, 1),    size(tracker, 2));
    
    [max_val, max_idx] = max(delta_traction);
    printf('max value = %.0f at index %d\n', max_val, max_idx);

    [max_val, max_idx] = max(t_steer);
    printf('max value = %.0f at index %d\n', max_val, max_idx);

    %normalize delta_traction between 0 and 5000 ????
    %delta_traction = delta_traction / max(delta_traction) * 5000;

    %normalize t_steer between 0 and 8192 ???
    %t_steer = t_steer / max(t_steer) * 8192;

    ticks = [t_steer, delta_traction];  % n x 2

    %==========================================================
    % PLOT DATA: TRACKER wrt WORLD
    %==========================================================
   
    
    figure(2);   
    clf;
    hold on;
    plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 2);
    legend('Tracker Pose');
    xlabel('x [m]');
    ylabel('y [m]');    
    title('Trajectory of Tracker');
    grid on;

    %==========================================================
    % COMPUTE ODOMETRY Trajectory
    %==========================================================

    x = [0.1; 0.0106141; 1.4; 0];  % initial guess for the parameters : [ Ksteer Ktraction axis_length steer_offset ]
    
    U = zeros(n, 3);
    for i = 1:n
        ticks_i = ticks(i, :)';
        U(i,:) = h_odom(x, ticks_i)';
    endfor 

    T = compute_odometry_trajectory(U);
    

    %==========================================================
    % PLOT DATA: ODOMETRY TRAJECTORY AND TRACKER POSE
    %==========================================================

    figure(3);
    clf;
    hold on;
    plot(T(:,1),       T(:,2),       'b-', 'LineWidth', 1);
    plot(tracker(:,1), tracker(:,2), 'r-', 'LineWidth', 1);
    legend('Odometry (biased)', 'Model Pose');
    xlabel('x [m]');
    ylabel('y [m]');
    title('Trajectory comparison');
    grid on;

    %==========================================================
    % CALIBRATION
    %==========================================================

  % Generate Measurement matrix Z

    Z = zeros(5, n);
    Z(1:2, :) = ticks';   % 2 tick values
    Z(3:5,:) = tracker';       % 3 odometry measurements

  % Perform one round of Gauss-Newton optimization
    printf('Performing one round of Gauss-Newton optimization...\n');

    U = zeros(n, 3);
    for i = 1:1
      [x_new, chi] = oneRound(x, Z);
      printf('Round %d: chi = %.6f\n', i, chi);
      x = x_new;

    endfor
      

    printf('Updated parameters: k_steer = %.4f, k_traction = %.6f, b = %.2f, steer_offset = %.2f\n', x_new(1), x_new(2), x_new(3), x_new(4));  

   

   



endfunction  

%=========================================================
LOCAL FUNCTIONS
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

% Compute the trajectory from the odometry measurements

function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);   % pre-allocate the trajectory matrix 
	current_T=v2t(zeros(1,3)); % converts vector into a rotation matrix full of zeros (initial pose)
	for i=1:size(U,1),
		u=U(i,1:3)' ;
		
		current_T = current_T * v2t(u);  % (world) current_T(i) = (world) current_T (i-1) * (i-1) relative pose (i)
		T(i,1:3)=t2v(current_T)';  % converts the current pose into a vector and stores it in the trajectory matrix as a row vector
	endfor
end


% Compute the odometry measurement from the ticks and the parameters

function delta = h_odom(x, ticks)
  k_steer      = x(1);
  k_traction   = x(2);
  L            = x(3);
  steer_offset = x(4);

  steer_tick   = ticks(1);
  delta_tick   = ticks(2);

  phi = k_steer * steer_tick - steer_offset;   % o + offset, dipende dalla convenzione
  dl  = k_traction * delta_tick;               % distanza ruota anteriore

  dth = (dl / L) * sin(phi);
  dx  = dl * cos(phi);
  dy  = 0;

  delta = [dx; dy; dth];
end

%function delta = h_odom(x, ticks)
  %k_steer      = x(1);
  %k_traction   = x(2);
  %b            = x(3);
  %steer_offset = x(4);

  %t_steer_tick   = ticks(1);
  %delta_traction = ticks(2);

  % steering angle (absolute encoder corrected by offset)
  %dphi = k_steer *t_steer_tick - steer_offset;
  % distance traveled
  %dl = k_traction * delta_traction;

  % tricycle kinematic model
  %dth = dl * sin(dphi) / b;
  %dx  = dl * cos(dth)*cos(dphi);
  %dy  = dl * sin(dth)*cos(dphi);
  %delta = [dx; dy; dth];
%end

%function delta = h_odom(x, ticks)
  %k_steer      = x(1);
  %k_traction   = x(2);
  %b            = x(3);
  %steer_offset = x(4);

  %t_steer_tick   = ticks(1);
  %delta_traction = ticks(2);

  % steering angle (absolute encoder corrected by offset)
  %dphi = k_steer *t_steer_tick - steer_offset;
  % distance traveled
  %dl = k_traction * delta_traction;

  % tricycle kinematic model
  %dth = dl * tan(dphi) / b;
  %dx  = dl * cos(dth);
  %dy  = dl * sin(dth);
  %delta = [dx; dy; dth];
%end

% Compute the error and the Jacobian for one measurement

function J = computeJacobian(x, ticks_all, j)
  J = zeros(3, 4);
  epsilon = 1e-6;
  printf('E');
  for i = 1:4
    e_vec    = zeros(4, 1);
    e_vec(i) = epsilon;
    
    pred_plus  = h_odom_absolute(x + e_vec, ticks_all)(j, :)';
    pred_minus = h_odom_absolute(x - e_vec, ticks_all)(j, :)';
    
    J(:, i) = (pred_plus - pred_minus) / (2 * epsilon);
  endfor
endfunction

% Perform one round of Gauss-Newton optimization

function [x_new, chi] = oneRound(x, Z)
  H     = zeros(4, 4);
  b     = zeros(4, 1);
  nmeas = size(Z, 2);
  chi   = 0;
  printf('A')
  ticks_all = Z(1:2, :)';  % n x 2
  meas_all  = Z(3:5, :)';  % n x 3
  printf('B')
  % Calcola traiettoria completa UNA volta
  pred_all = h_odom_absolute(x, ticks_all);  % n x 3
  printf('C')
  for j = 1:nmeas
    pred = pred_all(j, :)';
    meas = meas_all(j, :)';
    e    = pred - meas;
    
    % Jacobiano numerico
    J = computeJacobian(x, ticks_all, j);
    
    H   += J' * J;
    b   += J' * e;
    chi += e' * e;  %'
  endfor
  printf('D')
  dx    = -H \ b;
  x_new = x + dx;
endfunction

function odom_absolute = h_odom_absolute(x, ticks)

  U = zeros(size(ticks, 1), 3);
  for i = 1:size(ticks, 1)
    ticks_i = ticks(i, :)';
    U(i,:) = h_odom(x, ticks_i)';
  endfor 

  odom_absolute = compute_odometry_trajectory(U);
    
endfunction
