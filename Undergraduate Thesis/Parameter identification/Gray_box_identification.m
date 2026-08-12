clear; clc; close all;

%% 1 ── Load data ──────────────────────────────────────────────────────────────
raw = readtable('OL_6rad-s.csv', ...
    'Delimiter',        ';', ...
    'DecimalSeparator', ',');

t       = raw.Timestamp_s;
I_alpha = raw.I_alpha;
I_beta  = raw.I_beta;
the     = raw.Theta_enc;          % raw AS5600 angle (uncorrected for phi0)
we      = raw.Omega_enc_filtered;
V_alpha = raw.V_alpha;
V_beta  = raw.V_beta;

%% 2 ── Trim and DC-offset removal ────────────────────────────────────────────
idx     = t >= 0.2;
t       = t(idx);       t = t - t(1);
I_alpha = I_alpha(idx);
I_beta  = I_beta(idx);
the     = the(idx);
we      = we(idx);
V_alpha = V_alpha(idx);
V_beta  = V_beta(idx);

I_alpha = I_alpha - mean(I_alpha);
I_beta  = I_beta  - mean(I_beta);

%% 3 ── Quick joint regression estimate (Ke AND phi0) ─────────────────────────
Rs = 0.285;

y1 = Rs*I_alpha - V_alpha;
y2 = Rs*I_beta  - V_beta;
y  = [y1; y2];

col_A = [ we.*sin(the);  -we.*cos(the)];
col_B = [ we.*cos(the);   we.*sin(the)];
Phi   = [col_A, col_B];

sol      = Phi \ y;             % ordinary least squares
A        = sol(1);
B        = sol(2);
Ke_reg   = hypot(A, B);
phi0_reg = atan2(B, A);

resid = y - Phi*sol;
R2_reg = 1 - var(resid)/var(y);

fprintf('Ke   (regression) = %.6f V.s/rad\n', Ke_reg);
fprintf('phi0 (regression) = %.4f rad (%.2f deg)\n', phi0_reg, rad2deg(phi0_reg));
fprintf('R^2  (regression) = %.4f\n', R2_reg);

%% 4 ── Build iddata — 4 inputs (theta_enc is uncorrected; phi0 corrects it inside the ODE)
Ts_mean = mean(diff(t));

data = iddata([I_alpha, I_beta], [V_alpha, V_beta, the, we], Ts_mean, ...
    'Tstart',     0, ...
    'OutputName', {'i_alpha', 'i_beta'}, ...
    'OutputUnit', {'A', 'A'}, ...
    'InputName',  {'V_alpha', 'V_beta', 'theta_enc', 'omega_e'}, ...
    'InputUnit',  {'V', 'V', 'rad', 'rad/s'});

%% 5 ── Create idnlgrey model — TWO free parameters: Ke and phi0 ──────────────
Ke_init   = max(Ke_reg, 0.001);
phi0_init = phi0_reg;
x0_init   = double([I_alpha(1); I_beta(1)]);

nlgr = idnlgrey('bldc_ab_ode', [2, 4, 2], {Ke_init, phi0_init}, x0_init, 0, ...
    'OutputName', {'i_alpha', 'i_beta'}, ...
    'InputName',  {'V_alpha', 'V_beta', 'theta_enc', 'omega_e'});
%                   2 out, 4 in, 2 states ^

nlgr.Parameters(1).Minimum = 0;        % Ke >= 0
nlgr.Parameters(1).Maximum = 0.15;
nlgr.Parameters(2).Minimum = -pi;      % phi0 in [-pi, pi]
nlgr.Parameters(2).Maximum =  pi;
nlgr.InitialStates(1).Fixed = false;
nlgr.InitialStates(2).Fixed = false;

%% 6 ── Estimation options ─────────────────────────────────────────────────────
opt = nlgreyestOptions('Display', 'on');
opt.SearchOptions.MaxIterations     = 200;
opt.SearchOptions.FunctionTolerance = 1e-8;
opt.EstimateCovariance              = true;

%% 7 ── Run identification ─────────────────────────────────────────────────────
fprintf('\nRunning nlgreyest...\n');
nlgr_est = nlgreyest(data, nlgr, opt);

%% 8 ── Results ────────────────────────────────────────────────────────────────
Ke_est   = nlgr_est.Parameters(1).Value;
phi0_est = nlgr_est.Parameters(2).Value;

try
    P        = getcov(nlgr_est);
    Ke_std   = sqrt(P(1,1));
    phi0_std = sqrt(P(2,2));
catch
    Ke_std   = NaN;
    phi0_std = NaN;
end

fprintf('\n========== IDENTIFICATION RESULT ==========\n');
fprintf('Ke   (regression):  %.6f V.s/rad\n', Ke_reg);
fprintf('Ke   (nlgreyest):   %.6f  +/-  %.6f  V.s/rad\n', Ke_est, Ke_std);
fprintf('phi0 (regression):  %.4f rad\n', phi0_reg);
fprintf('phi0 (nlgreyest):   %.4f  +/-  %.4f  rad  (%.2f deg)\n', phi0_est, phi0_std, rad2deg(phi0_est));
fprintf('Model fit:          %.2f%%  /  %.2f%%  (i_alpha / i_beta)\n', ...
    nlgr_est.Report.Fit.FitPercent(1), nlgr_est.Report.Fit.FitPercent(2));

%% 9 ── Export CSV ────────────────────────────────────────────────────────────
ysim   = sim(nlgr_est, data);
t_plot = data.SamplingInstants;
y_meas = data.OutputData;
y_pred = ysim.OutputData;

out = table(t_plot, ...
            y_meas(:,1), y_meas(:,2), ...
            y_pred(:,1), y_pred(:,2), ...
    'VariableNames', {'time_s', ...
                      'i_alpha_meas_A', 'i_beta_meas_A', ...
                      'i_alpha_pred_A', 'i_beta_pred_A'});

writetable(out, 'greybox_id_result_new.csv');
fprintf('Saved -> greybox_id_result_new.csv  (%d rows)\n', height(out));
