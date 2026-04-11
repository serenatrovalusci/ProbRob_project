
function delta = h_odom(x, ticks)
  k_steer      = x(1);
  k_traction   = x(2);
  b            = x(3);
  steer_offset = x(4);

  t_steer          = ticks(1);
  delta_traction   = ticks(2);   % già il delta, calcolato fuori

  dph = k_steer * t_steer - steer_offset;
  dl  = k_traction * delta_traction;
  dth = dl * tan(dph) / b;
  dx  = dl * cos(dth);
  dy  = dl * sin(dth);

  delta = [dx, dy, dth]';
endfunction

function Z = generateMeasurements(n, x)
  % zi = [t_steer, delta_traction, dx, dy, dth]
  Z = zeros(5, n);
  Z(1:2, :) = rand(2, n);   % 2 tick values
  for i = 1:n
    ticks    = Z(1:2, i);
    Z(3:5,i) = h_odom(x, ticks);
  endfor
endfunction

function [e, J] = errorAndJacobian(x, z)
  ticks = z(1:2);   % 2 ticks
  meas  = z(3:5);   % measurements
  pred  = h_odom(x, ticks);
  e     = pred - meas;
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
  for i = 1:nmeas
    [e, J] = errorAndJacobian(x, Z(:, i));
    H   += J' * J;
    b   += J' * e;
    chi += e' * e;
  endfor
  dx    = -H \ b;
  x_new = x + dx;
endfunction