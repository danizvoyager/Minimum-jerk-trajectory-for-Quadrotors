% Define the waypoints
waypoints = [0 0 0; 1 1 1; 2 0 2; 3 1 1; 4 0 0];

% Define the time intervals
t_total = 4;
t = linspace(0, t_total, size(waypoints, 1));

% Compute the minimum snap trajectory
[p, v, a, j] = minSnapTrajectory(waypoints, t);

% Plot the results
figure;
subplot(4, 1, 1);
plot(t, p);
title('Position');
subplot(4, 1, 2);
plot(t, v);
title('Velocity');
subplot(4, 1, 3);
plot(t, a);
title('Acceleration');
subplot(4, 1, 4);
plot(t, j);
title('Jerk');

function [p, v, a, j] = minSnapTrajectory(waypoints, t)
% Compute the minimum snap trajectory
n = size(waypoints, 1);
T = t(end) - t(1);

  % Compute the polynomial coefficients
  A = zeros(4 * (n - 1), 4 * (n - 1));
  b = zeros(4 * (n - 1), 3);
  for i = 1:n-1
      A(4 * (i - 1) + 1:4 * i, 4 * (i - 1) + 1:4 * i) = [1 t(i) t(i)^2 t(i)^3;
                                                         0 1 2 * t(i) 3 * t(i)^2;
                                                         0 0 2 6 * t(i);
                                                         0 0 0 6];
      b(4 * (i - 1) + 1:4 * i, :) = [waypoints(i, :); waypoints(i + 1, :) - waypoints(i, :)];
  end
  
  % Solve for the polynomial coefficients
  c = A \ b;
  
  % Compute the position, velocity, acceleration, and jerk
  p = zeros(length(t), 3);
  v = zeros(length(t), 3);
  a = zeros(length(t), 3);
  j = zeros(length(t), 3);
  for i = 1:length(t)
      for k = 1:n-1
          if t(i) >= t(k) && t(i) < t(k+1)
              p(i, :) = c(4 * (k - 1) + 1:4 * (k - 1) + 4, :) * [1; t(i); t(i)^2; t(i)^3];
              v(i, :) = c(4 * (k - 1) + 2:4 * (k - 1) + 5, :) * [1; t(i); t(i)^2; t(i)^3];
              a(i, :) = c(4 * (k - 1) + 3:4 * (k - 1) + 6, :) * [1; t(i); t(i)^2; t(i)^3];
              j(i, :) = c(4 * (k - 1) + 4:4 * (k - 1) + 7, :) * [1; t(i); t(i)^2; t(i)^3];
              break;
          end
      end
  end
end