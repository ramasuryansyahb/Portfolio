%% Pole Placement Analysis for State-Space System
% The control law is u = -Kx, which leads to a closed-loop system:
% x_dot = (A - B*K)x
clc; clear; close all;
%% 1. Define System Matrices
% A matrix
A = [-1.5   0.12    0   -0.9;
     0    -0.3    1    0.05;
     8      0    -3      1;
     3      0     0    -0.8];
% B matrix
B = [ 1.2    0.1;
      0      0.05;
      0.1    1.5;
      0.05   0.8];
% C matrix (from second equation, y = Ix)
C = eye(4);
% D matrix (from second equation, D=0)
D = zeros(4, 2);

%% 2. Create Open-Loop System Model
sys_ol = ss(A, B, C, D);
n = size(A, 1); % Number of states

%% 3. Analyze Open-Loop System (Stability & Controllability)
% Get the open-loop poles (eigenvalues of A)
poles_ol = eig(A);
fprintf('--- Open-Loop System Analysis ---\n');
fprintf('Open-loop poles are at:\n');
disp(poles_ol);
if any(real(poles_ol) > 0)
    fprintf('System is UNSTABLE (one or more poles are in the RHP).\n');
elseif any(real(poles_ol) == 0)
    fprintf('System is MARGINALLY STABLE (one or more poles are on jw-axis).\n');
else
    fprintf('System is STABLE (all poles are in the LHP).\n');
end
% Check controllability
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('\nControllability matrix rank: %d\n', rank_Co);
if rank_Co == n
    fprintf('System is controllable. Pole placement can be performed.\n');
else
    fprintf('System is NOT controllable (rank < %d). Pole placement cannot arbitrarily place all poles.\n', n);
end
%% 4. Pole Placement Design
Wn_d = 2.014;  % Desired natural frequency (rad/s)
zeta_d = 0.624; % Desired damping ratio
p_desired = [-Wn_d*zeta_d + 1j*Wn_d*sqrt(1-zeta_d^2);
             -Wn_d*zeta_d - 1j*Wn_d*sqrt(1-zeta_d^2);
             -1.0;
             -8.0];
fprintf('\n--- Pole Placement Design ---\n');
fprintf('Desired closed-loop poles are at:\n');
disp(p_desired);
try
    % Use 'place' for pole placement
    K = place(A, B, p_desired);
        
    fprintf('Calculated feedback gain matrix K:\n');
    disp(K);
catch ME
    fprintf('Error during pole placement: %s\n', ME.message);
    fprintf('This can happen if the system is not controllable or if poles are repeated.\n');
    return;
end

%% 5. Create and Analyze Closed-Loop System
A_cl = A - B*K;
% Create the closed-loop state-space object.
sys_cl = ss(A_cl, B, C, D);
% Get the closed-loop poles
poles_cl = eig(A_cl);
fprintf('\n--- Closed-Loop System Analysis ---\n');
fprintf('Actual closed-loop poles are at:\n');
disp(poles_cl);
fprintf('These should match the desired poles.\n');

%% 6. Simulation
% initial condition.
t = 0:0.01:10; % Simulation time
% Initial condition: 0.1 rad (approx 5.7 deg) of sideslip
x0 = [0.346; 0; 0; 0]; 
% Simulate open-loop response
[y_ol, t_ol, x_ol] = initial(sys_ol, x0, t);
% Simulate closed-loop response
[y_cl, t_cl, x_cl] = initial(sys_cl, x0, t);
% Plot the results
figure;
sgtitle('Open-Loop vs. Closed-Loop Response to Initial Condition', 'FontSize', 14, 'FontWeight', 'bold');
% p(t), beta(t), beta_dot(t), r(t)
state_names = {'\rho(t) (roll angle?)', '\beta(t) (sideslip angle)', '\beta dot(t) (sideslip rate)', 'r(t) (yaw rate)'};
state_names = {'\rho (State 1)', '\beta (State 2)', '\beta dot (State 3)', 'r (State 4)'};
for i = 1:n
    subplot(n, 2, 2*i - 1);
    plot(t_ol, y_ol(:, i), 'r', 'LineWidth', 1.5);
    title(['Open-Loop: ' state_names{i}]);
    ylabel('Amplitude');
    grid on;
    if i == n, xlabel('Time (s)'); end
    subplot(n, 2, 2*i);
    plot(t_cl, y_cl(:, i), 'b', 'LineWidth', 1.5);
    title(['Closed-Loop: ' state_names{i}]);
    grid on;
    if i == n, xlabel('Time (s)'); end
end
legend(subplot(n,2,1), 'Open-Loop');
legend(subplot(n,2,2), 'Closed-Loop');
%% 7. Pole-Zero Map Visualization (Separated)
% Figure 1: Open-Loop
figure;
[p_ol, z_ol] = pzmap(sys_ol);
% Plot open-loop poles as red 'x'
h1 = plot(real(p_ol), imag(p_ol), 'rx', 'MarkerSize', 10, 'LineWidth', 1.5);
hold on;
% Plot open-loop zeros as red 'o' (if any)
h2 = plot(NaN, NaN, 'ro'); % Dummy handle for legend
if ~isempty(z_ol)
    h2 = plot(real(z_ol), imag(z_ol), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
end
title('Open Loop Pole-Zero Map', 'FontSize', 14);
% Create a robust legend
legend_handles_ol = [h1];
legend_labels_ol = {'Open-Loop Poles'};
if ~isempty(z_ol)
     legend_handles_ol = [legend_handles_ol, h2];
     legend_labels_ol = [legend_labels_ol, 'Open-Loop Zeros'];
end
legend(legend_handles_ol, legend_labels_ol, 'Location', 'best');
grid on;
sgrid; % Adds stability boundaries
% Add vertical and horizontal axes for clarity
ax = gca;
plot(ax.XLim, [0 0], 'k:', 'HandleVisibility', 'off'); % Real axis (dashed)
plot([0 0], ax.YLim, 'k:', 'HandleVisibility', 'off'); % Imaginary axis (dashed)
hold off;
% Figure 2: Closed-Loop 
figure;
[p_cl, z_cl] = pzmap(sys_cl);
% Plot closed-loop poles as blue 'x'
h3 = plot(real(p_cl), imag(p_cl), 'bx', 'MarkerSize', 10, 'LineWidth', 1.5);
hold on;
% Plot closed-loop zeros as blue 'o' (if any)
h4 = plot(NaN, NaN, 'bo'); % Dummy handle for legend
if ~isempty(z_cl)
    h4 = plot(real(z_cl), imag(z_cl), 'bo', 'MarkerSize', 10, 'LineWidth', 1.5);
end
title('Closed Loop Pole-Zero Map', 'FontSize', 14);
% Create a robust legend
legend_handles_cl = [h3];
legend_labels_cl = {'Closed-Loop Poles'};
if ~isempty(z_cl)
     legend_handles_cl = [legend_handles_cl, h4];
     legend_labels_cl = [legend_labels_cl, 'Closed-Loop Zeros'];
end
legend(legend_handles_cl, legend_labels_cl, 'Location', 'best');
grid on;
sgrid; % Adds stability boundaries
% Add vertical and horizontal axes for clarity
ax_cl = gca;
plot(ax_cl.XLim, [0 0], 'k:', 'HandleVisibility', 'off'); % Real axis (dashed)
plot([0 0], ax_cl.YLim, 'k:', 'HandleVisibility', 'off'); % Imaginary axis (dashed)
hold off;
fprintf('\n--- Pole-Zero Maps Generated (Separate) ---\n');
% Accurate description based on the system's actual stability
if any(real(poles_ol) > 0)
    fprintf('Red ''x'': Open-loop poles (at least one unstable).\n');
elseif any(real(poles_ol) == 0)
    fprintf('Red ''x'': Open-loop poles (at least one marginally stable).\n');
else
    fprintf('Red ''x'': Open-loop poles (all stable).\n');
end
fprintf('Blue ''x'': Closed-loop poles (moved to new desired stable locations).\n');