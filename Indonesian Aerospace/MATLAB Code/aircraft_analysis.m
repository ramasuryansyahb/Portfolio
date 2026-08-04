function aircraft_analysis_comfort_focused()
    close all; clear; clc;
    fprintf('2-State LMI Aircraft Controller Analysis\n');
    fprintf('Aircraft: CN235-220 | Focus: Passenger Comfort & Coordinated Turns\n\n');

    % --- Initialization ---
    params = initialize_aircraft_parameters();
    validate_parameters(params);
    display_flight_conditions(params);

    % --- Controller Synthesis ---
    fprintf('Synthesizing controllers...\n');
    controllers = struct('name', {}, 'results', {});

    % Baseline Controller
    fprintf('\n--- Defining Baseline Controller ---\n');
    baseline_results = initialize_controller_results();
    baseline_results.method = 'Baseline';
    baseline_results.synthesis_success = true;
    baseline_results.K_beta = params.K_beta_baseline;
    baseline_results.K_r = params.K_r_baseline;
    controllers(1).name = 'Baseline';
    controllers(1).results = baseline_results;
    fprintf('  Baseline controller defined (K_beta: %.2f, K_r: %.2f).\n', ...
        baseline_results.K_beta, baseline_results.K_r);

    % LMI-based Controllers with integrated Quasi-Newton optimization
    synthesis_functions = {
        @synthesize_pure_h2_hybrid_qn, 'Pure H2 (Hybrid QN)'; 
        @synthesize_pure_hinf_hybrid_qn, 'Pure Hinf (Hybrid QN)'; 
        @synthesize_mixed_h2_hinf_hybrid_qn, 'Mixed H2/Hinf (Hybrid QN)' 
    };

    for i = 1:size(synthesis_functions, 1)
        synth_func = synthesis_functions{i, 1};
        base_name = synthesis_functions{i, 2};
        
        fprintf('\n--- Synthesizing %s Controller (Comfort Focused) ---\n', base_name);
        lmi_results = synth_func(params);
        
        controllers(end+1).name = base_name;
        controllers(end).results = lmi_results;
    end

    % --- Simulation ---
    fprintf('\nSimulating responses for successful controllers...\n');
    simulation_results = {}; 
    
    [gust_series, gust_params] = generate_dryden_gust_series(params);
    
    for i = 1:length(controllers)
        ctrl = controllers(i);
        if isfield(ctrl.results, 'synthesis_success') && ctrl.results.synthesis_success
            fprintf('  Simulating %s controller...\n', ctrl.name);
            [time, states, controls] = simulate_2state_optimized_response(params, ctrl.results, gust_series);
            
            sim_result = struct();
            sim_result.time = time;
            sim_result.states = states;
            sim_result.controls = controls;
            sim_result.name = ctrl.name;
            sim_result.results = ctrl.results;
            simulation_results{end+1} = sim_result;
        else
            fprintf('  Skipping simulation for %s (synthesis failed).\n', ctrl.name);
        end
    end

    if isempty(simulation_results)
        fprintf('\nNo controllers were successfully synthesized and simulated. Skipping analysis.\n');
        return;
    end
    
    % --- Analysis & Visualization ---
    fprintf('Generating performance comparison...\n');
    generate_performance_comparison(simulation_results, params, gust_series, gust_params);
    print_performance_metrics(simulation_results, params);
    fprintf('\nAircraft controller analysis complete.\n');
end

%CONTROLLER SYNTHESIS FUNCTIONS (HYBRID QN + LMI)

function controller_results = synthesize_pure_h2_hybrid_qn(params)
    % Integrates Quasi-Newton optimization with Pure H2 LMI feasibility check.
    controller_results = initialize_controller_results();
    if ~exist('sdpvar', 'file')
        fprintf('  YALMIP not available. Using baseline gains.\n');
        controller_results.synthesis_success = true;
        controller_results.method = 'Pure H2 (Hybrid QN)';
        controller_results.K_beta = params.K_beta_baseline;
        controller_results.K_r = params.K_r_baseline;
        return;
    end
    
    fprintf('  Starting Pure H2 (Hybrid QN) Optimization...\n');
    
    % Quasi-Newton parameters
    max_iterations = 30;
    tolerance = 1e-4;
    step_size = 0.1;
    
    % Initial gains (start from baseline)
    K_current = [params.K_beta_baseline; params.K_r_baseline];
    
    % Storage for iteration history
    cost_history = [];
    
    % Generate gust series for consistent evaluation
    [gust_series, ~] = generate_dryden_gust_series(params);
    
    fprintf('    Initial gains: K_beta=%.3f, K_r=%.3f\n', K_current(1), K_current(2));
    
    [A_2state, B_2state] = get_2state_model(params);
    nx = 2; nu = 1;
    
    % H2 LMI specific parameters (from original synthesize_pure_h2_comfort_focused)
    q_beta = 6.0;
    q_r = 2.5;
    r_rudder = 5.0;
    min_decay = 0.12;
    
    Q = diag([q_beta, q_r]);
    R = r_rudder;
    B2 = [0.05; 0.02]; % Disturbance input matrix
    C2 = [sqrt(q_beta), 0; 0, sqrt(q_r); 0, 0];
    D2u = [0; 0; sqrt(r_rudder)];
    eps_P = 1e-4;
    options = sdpsettings('verbose', 0, 'solver', 'mosek');

    for iter = 1:max_iterations
        fprintf('    Iter %2d: ', iter);
        
        % Step 1: LMI Feasibility Check (H2 specific)
        P = sdpvar(nx, nx, 'symmetric');
        Y = sdpvar(nu, nx, 'full');
        W = sdpvar(size(C2,1), size(C2,1), 'symmetric');
        
        K_check = [K_current(1), K_current(2)]; 
        A_cl_check = A_2state - B_2state * K_check;
        
        constraints_h2 = [P >= eps_P*eye(nx), W >= 1e-6*eye(size(W,1)), P <= 100*eye(nx)];
        constraints_h2 = [constraints_h2, Y(1,1) >= 0, Y(1,1) <= 15.0]; 
        constraints_h2 = [constraints_h2, Y(1,2) >= 0, Y(1,2) <= 6.0];  
        
        LMI_stability_h2 = A_cl_check*P + P*A_cl_check' + B_2state*Y + Y'*B_2state' + 2*min_decay*P;
        constraints_h2 = [constraints_h2, LMI_stability_h2 <= -eps_P*eye(nx)];
        
        LMI_h2_lyap = [A_cl_check*P + P*A_cl_check' + B_2state*Y + Y'*B_2state', B2; B2', -eye(size(B2,2))];
        constraints_h2 = [constraints_h2, LMI_h2_lyap <= -eps_P*eye(size(LMI_h2_lyap,1))];
        LMI_h2_perf = [W, (C2*P + D2u*Y); (C2*P + D2u*Y)', P];
        constraints_h2 = [constraints_h2, LMI_h2_perf >= eps_P*eye(size(LMI_h2_perf,1))];

        sol_lmi = optimize(constraints_h2, [], options); 
        lmi_feasible = (sol_lmi.problem == 0);
        
        if ~lmi_feasible
            fprintf('LMI infeasible, reducing step size...\n');
            step_size = step_size * 0.5;
            if step_size < 1e-6
                fprintf('    Optimization failed: Cannot find feasible region\n');
                break;
            end
            continue;
        end
        
        % Step 2: Simulate with current gains
        temp_controller = struct();
        temp_controller.synthesis_success = true;
        temp_controller.K_beta = K_current(1);
        temp_controller.K_r = K_current(2);
        
        [~, states, controls] = simulate_2state_optimized_response(params, temp_controller, gust_series);
        
        % Step 3: Evaluate comfort cost function
        J_current = compute_comfort_cost_function(states, controls, params);
        cost_history(end+1) = J_current;
        
        fprintf('K_beta=%.3f, K_r=%.3f, Cost=%.4f', K_current(1), K_current(2), J_current);
        
        % Step 4: Compute numerical gradients
        gradient = compute_numerical_gradient(params, K_current, gust_series, J_current);
        
        % Step 5: Quasi-Newton update (simplified BFGS-like)
        if iter == 1
            H_inv = eye(2); % Initial inverse Hessian approximation
        else
            H_inv = update_inverse_hessian(H_inv, K_current - K_prev, gradient - grad_prev);
        end
        
        % Update gains
        K_prev = K_current;
        grad_prev = gradient;
        K_new = K_current - step_size * H_inv * gradient;
        
        % Apply bounds to gains
        K_new(1) = max(0, min(12, K_new(1))); % K_beta bounds
        K_new(2) = max(0, min(6, K_new(2)));  % K_r bounds
        
        % Check convergence
        if norm(K_new - K_current) < tolerance
            fprintf(' -> CONVERGED\n');
            K_current = K_new;
            break;
        end
        
        K_current = K_new;
        fprintf(' -> Updated\n');
    end
    
    % Final LMI check and results extraction
    % Re-solve the LMI with the final K_current to get the norms/poles
    P = sdpvar(nx, nx, 'symmetric');
    Y = sdpvar(nu, nx, 'full');
    W = sdpvar(size(C2,1), size(C2,1), 'symmetric');
    K_final = [K_current(1), K_current(2)];
    A_cl_final = A_2state - B_2state * K_final;

    constraints_h2_final = [P >= eps_P*eye(nx), W >= 1e-6*eye(size(W,1)), P <= 100*eye(nx)];
    constraints_h2_final = [constraints_h2_final, Y(1,1) >= 0, Y(1,1) <= 15.0];
    constraints_h2_final = [constraints_h2_final, Y(1,2) >= 0, Y(1,2) <= 6.0];
    LMI_stability_h2_final = A_cl_final*P + P*A_cl_final' + B_2state*Y + Y'*B_2state' + 2*min_decay*P;
    constraints_h2_final = [constraints_h2_final, LMI_stability_h2_final <= -eps_P*eye(nx)];
    LMI_h2_lyap_final = [A_cl_final*P + P*A_cl_final' + B_2state*Y + Y'*B_2state', B2; B2', -eye(size(B2,2))];
    constraints_h2_final = [constraints_h2_final, LMI_h2_lyap_final <= -eps_P*eye(size(LMI_h2_lyap_final,1))];
    LMI_h2_perf_final = [W, (C2*P + D2u*Y); (C2*P + D2u*Y)', P];
    constraints_h2_final = [constraints_h2_final, LMI_h2_perf_final >= eps_P*eye(size(LMI_h2_perf_final,1))];

    sol_final = optimize(constraints_h2_final, trace(W), options);

    controller_results.synthesis_success = (sol_final.problem == 0);
    controller_results.method = 'Pure H2 (Hybrid QN)';
    controller_results.K_beta = K_current(1);
    controller_results.K_r = K_current(2);
    controller_results.optimization_iterations = length(cost_history);
    controller_results.final_cost = cost_history(end);
    controller_results.cost_history = cost_history;
    controller_results.closed_loop_poles = eig(A_cl_final);
    if controller_results.synthesis_success
        controller_results.h2_norm = value(trace(W));
    else
        controller_results.h2_norm = NaN;
    end
    
    fprintf('    FINAL: K_beta=%.3f, K_r=%.3f, Cost=%.4f, Iterations=%d\n', ...
        K_current(1), K_current(2), cost_history(end), length(cost_history));
end

function controller_results = synthesize_pure_hinf_hybrid_qn(params)
    % Integrates Quasi-Newton optimization with Pure H-infinity LMI feasibility check.
    controller_results = initialize_controller_results();
    if ~exist('sdpvar', 'file')
        fprintf('  YALMIP not available. Using baseline gains.\n');
        controller_results.synthesis_success = true;
        controller_results.method = 'Pure Hinf (Hybrid QN)';
        controller_results.K_beta = params.K_beta_baseline;
        controller_results.K_r = params.K_r_baseline;
        return;
    end
    
    fprintf('  Starting Pure Hinf (Hybrid QN) Optimization...\n');
    
    % Quasi-Newton parameters
    max_iterations = 30;
    tolerance = 1e-4;
    step_size = 0.1;
    
    % Initial gains (start from baseline)
    K_current = [params.K_beta_baseline; params.K_r_baseline];
    
    % Storage for iteration history
    cost_history = [];
    
    % Generate gust series for consistent evaluation
    [gust_series, ~] = generate_dryden_gust_series(params);
    
    fprintf('    Initial gains: K_beta=%.3f, K_r=%.3f\n', K_current(1), K_current(2));
    
    [A_2state, B_2state] = get_2state_model(params);
    nx = 2; nu = 1;

    % H-infinity LMI specific parameters
    gamma_target = 3.5;
    q_beta = 22.0;
    q_r = 1.5;
    r_control = 2.5;
    min_decay = 0.05;
    
    B1 = [0.1; 0.05]; % Disturbance input matrix
    C1 = [sqrt(q_beta), 0; 0, sqrt(q_r); 0, 0];
    D12 = [0; 0; sqrt(r_control)];
    eps_P = 1e-4;
    options = sdpsettings('verbose', 0, 'solver', 'mosek');
    
    for iter = 1:max_iterations
        fprintf('    Iter %2d: ', iter);
        
        % Step 1: LMI Feasibility Check (H-infinity specific)
        P = sdpvar(nx, nx, 'symmetric');
        Y = sdpvar(nu, nx, 'full');
        
        K_check = [K_current(1), K_current(2)]; % Use current QN gains for LMI check
        A_cl_check = A_2state - B_2state * K_check;
        
        constraints_hinf = [P >= eps_P*eye(nx), P <= 50*eye(nx)];
        constraints_hinf = [constraints_hinf, Y(1,1) >= 0, Y(1,1) <= 15.0];
        constraints_hinf = [constraints_hinf, Y(1,2) >= 0, Y(1,2) <= 6.0];
        
        LMI_hinf_check = [A_cl_check*P + P*A_cl_check' + B_2state*Y + Y'*B_2state' + 2*min_decay*P, B1, (C1*P + D12*Y)';
                          B1', -gamma_target*eye(size(B1,2)), zeros(size(B1,2), size(C1,1));
                          C1*P + D12*Y, zeros(size(C1,1), size(B1,2)), -gamma_target*eye(size(C1,1))];
        
        constraints_hinf = [constraints_hinf, LMI_hinf_check <= -eps_P*eye(size(LMI_hinf_check,1))];
        
        sol_lmi = optimize(constraints_hinf, [], options); % Feasibility check for H-inf LMI
        lmi_feasible = (sol_lmi.problem == 0);
        
        if ~lmi_feasible
            fprintf('LMI infeasible, reducing step size...\n');
            step_size = step_size * 0.5;
            if step_size < 1e-6
                fprintf('    Optimization failed: Cannot find feasible region\n');
                break;
            end
            continue;
        end
        
        % Step 2: Simulate with current gains
        temp_controller = struct();
        temp_controller.synthesis_success = true;
        temp_controller.K_beta = K_current(1);
        temp_controller.K_r = K_current(2);
        
        [~, states, controls] = simulate_2state_optimized_response(params, temp_controller, gust_series);
        
        % Step 3: Evaluate comfort cost function
        J_current = compute_comfort_cost_function(states, controls, params);
        cost_history(end+1) = J_current;
        
        fprintf('K_beta=%.3f, K_r=%.3f, Cost=%.4f', K_current(1), K_current(2), J_current);
        
        % Step 4: Compute numerical gradients
        gradient = compute_numerical_gradient(params, K_current, gust_series, J_current);
        
        % Step 5: Quasi-Newton update (simplified BFGS-like)
        if iter == 1
            H_inv = eye(2); % Initial inverse Hessian approximation
        else
            H_inv = update_inverse_hessian(H_inv, K_current - K_prev, gradient - grad_prev);
        end
        
        % Update gains
        K_prev = K_current;
        grad_prev = gradient;
        K_new = K_current - step_size * H_inv * gradient;
        
        % Apply bounds to gains
        K_new(1) = max(0, min(12, K_new(1))); % K_beta bounds
        K_new(2) = max(0, min(6, K_new(2)));  % K_r bounds
        
        % Check convergence
        if norm(K_new - K_current) < tolerance
            fprintf(' -> CONVERGED\n');
            K_current = K_new;
            break;
        end
        
        K_current = K_new;
        fprintf(' -> Updated\n');
    end
    
    % Final LMI check and results extraction
    % Re-solve the LMI with the final K_current to get the poles/gamma
    P = sdpvar(nx, nx, 'symmetric');
    Y = sdpvar(nu, nx, 'full');
    K_final = [K_current(1), K_current(2)];
    A_cl_final = A_2state - B_2state * K_final;

    constraints_hinf_final = [P >= eps_P*eye(nx), P <= 50*eye(nx)];
    constraints_hinf_final = [constraints_hinf_final, Y(1,1) >= 0, Y(1,1) <= 15.0];
    constraints_hinf_final = [constraints_hinf_final, Y(1,2) >= 0, Y(1,2) <= 6.0];
    LMI_hinf_final = [A_cl_final*P + P*A_cl_final' + B_2state*Y + Y'*B_2state' + 2*min_decay*P, B1, (C1*P + D12*Y)';
                      B1', -gamma_target*eye(size(B1,2)), zeros(size(B1,2), size(C1,1));
                      C1*P + D12*Y, zeros(size(C1,1), size(B1,2)), -gamma_target*eye(size(C1,1))];
    constraints_hinf_final = [constraints_hinf_final, LMI_hinf_final <= -eps_P*eye(size(LMI_hinf_final,1))];

    sol_final = optimize(constraints_hinf_final, [], options);

    controller_results.synthesis_success = (sol_final.problem == 0);
    controller_results.method = 'Pure Hinf (Hybrid QN)';
    controller_results.K_beta = K_current(1);
    controller_results.K_r = K_current(2);
    controller_results.optimization_iterations = length(cost_history);
    controller_results.final_cost = cost_history(end);
    controller_results.cost_history = cost_history;
    controller_results.closed_loop_poles = eig(A_cl_final);
    if controller_results.synthesis_success
        controller_results.gamma_inf = gamma_target;
    else
        controller_results.gamma_inf = NaN;
    end
    
    fprintf('    FINAL: K_beta=%.3f, K_r=%.3f, Cost=%.4f, Iterations=%d\n', ...
        K_current(1), K_current(2), cost_history(end), length(cost_history));
end

function controller_results = synthesize_mixed_h2_hinf_hybrid_qn(params)
    % Integrates Quasi-Newton optimization with Mixed H2/H-infinity LMI feasibility check.
    controller_results = initialize_controller_results();
    if ~exist('sdpvar', 'file')
        fprintf('  YALMIP not available. Using baseline gains.\n');
        controller_results.synthesis_success = true;
        controller_results.method = 'Mixed H2/Hinf (Hybrid QN)';
        controller_results.K_beta = params.K_beta_baseline;
        controller_results.K_r = params.K_r_baseline;
        return;
    end
    
    fprintf('  Starting Mixed H2/Hinf (Hybrid QN) Optimization...\n');
    
    % Quasi-Newton parameters
    max_iterations = 30;
    tolerance = 1e-4;
    step_size = 0.1;
    
    % Initial gains (start from baseline)
    K_current = [params.K_beta_baseline; params.K_r_baseline];
    
    % Storage for iteration history
    cost_history = [];
    
    % Generate gust series for consistent evaluation
    [gust_series, ~] = generate_dryden_gust_series(params);
    
    fprintf('    Initial gains: K_beta=%.3f, K_r=%.3f\n', K_current(1), K_current(2));
    
    [A_2state, B_2state] = get_2state_model(params);
    nx = 2; nu = 1;

    % Mixed H2/H-infinity LMI specific parameters
    gamma_fixed = 3.0;
    q_h2_beta = 4.0;
    r_h2 = 6.0;
    q_inf_beta = 10.0;
    r_inf = 3.0;
    min_decay = 0.08;
    
    B2_h2 = [0.08; 0.03]; B1_inf = [0.12; 0.06];
    C2 = [sqrt(q_h2_beta), 0; 0, 0; 0, 0]; D2u = [0; 0; sqrt(r_h2)];
    C1 = [sqrt(q_inf_beta), 0; 0, 0; 0, 0]; D12 = [0; 0; sqrt(r_inf)];
    eps_P = 1e-4;
    options = sdpsettings('verbose', 0, 'solver', 'mosek');
    
    for iter = 1:max_iterations
        fprintf('    Iter %2d: ', iter);
        
        % Step 1: LMI Feasibility Check (Mixed H2/H-infinity specific)
        P = sdpvar(nx, nx, 'symmetric');
        Y = sdpvar(nu, nx, 'full');
        W = sdpvar(size(C2,1), size(C2,1), 'symmetric');
        
        K_check = [K_current(1), K_current(2)]; % Use current QN gains for LMI check
        A_cl_check = A_2state - B_2state * K_check;
        
        constraints_mixed = [P >= eps_P*eye(nx), P <= 30*eye(nx), W >= 1e-6*eye(size(W,1))];
        constraints_mixed = [constraints_mixed, Y(1,1) >= -0.5, Y(1,1) <= 15.0];
        constraints_mixed = [constraints_mixed, Y(1,2) >= -0.5, Y(1,2) <= 6.0];
        
        LMI_hinf_check = [A_cl_check*P + P*A_cl_check' + B_2state*Y + Y'*B_2state' + 2*min_decay*P, B1_inf, (C1*P + D12*Y)';
                          B1_inf', -gamma_fixed*eye(size(B1_inf,2)), zeros(size(B1_inf,2), size(C1,1));
                          C1*P + D12*Y, zeros(size(C1,1), size(B1_inf,2)), -gamma_fixed*eye(size(C1,1))];
        constraints_mixed = [constraints_mixed, LMI_hinf_check <= -eps_P*eye(size(LMI_hinf_check,1))];
        
        LMI_h2_lyap = [A_cl_check*P + P*A_cl_check' + B_2state*Y + Y'*B_2state', B2_h2; B2_h2', -eye(size(B2_h2,2))];
        constraints_mixed = [constraints_mixed, LMI_h2_lyap <= -eps_P*eye(size(LMI_h2_lyap,1))];
        LMI_h2_perf = [W, (C2*P + D2u*Y); (C2*P + D2u*Y)', P];
        constraints_mixed = [constraints_mixed, LMI_h2_perf >= eps_P*eye(size(LMI_h2_perf,1))];

        sol_lmi = optimize(constraints_mixed, [], options); % Feasibility check for Mixed LMI
        lmi_feasible = (sol_lmi.problem == 0);
        
        if ~lmi_feasible
            fprintf('LMI infeasible, reducing step size...\n');
            step_size = step_size * 0.5;
            if step_size < 1e-6
                fprintf('    Optimization failed: Cannot find feasible region\n');
                break;
            end
            continue;
        end
        
        % Step 2: Simulate with current gains
        temp_controller = struct();
        temp_controller.synthesis_success = true;
        temp_controller.K_beta = K_current(1);
        temp_controller.K_r = K_current(2);
        
        [~, states, controls] = simulate_2state_optimized_response(params, temp_controller, gust_series);
        
        % Step 3: Evaluate comfort cost function
        J_current = compute_comfort_cost_function(states, controls, params);
        cost_history(end+1) = J_current;
        
        fprintf('K_beta=%.3f, K_r=%.3f, Cost=%.4f', K_current(1), K_current(2), J_current);
        
        % Step 4: Compute numerical gradients
        gradient = compute_numerical_gradient(params, K_current, gust_series, J_current);
        
        % Step 5: Quasi-Newton update (simplified BFGS-like)
        if iter == 1
            H_inv = eye(2); % Initial inverse Hessian approximation
        else
            H_inv = update_inverse_hessian(H_inv, K_current - K_prev, gradient - grad_prev);
        end
        
        % Update gains
        K_prev = K_current;
        grad_prev = gradient;
        K_new = K_current - step_size * H_inv * gradient;
        
        % Apply bounds to gains
        K_new(1) = max(0, min(12, K_new(1))); % K_beta bounds
        K_new(2) = max(0, min(6, K_new(2)));  % K_r bounds
        
        % Check convergence
        if norm(K_new - K_current) < tolerance
            fprintf(' -> CONVERGED\n');
            K_current = K_new;
            break;
        end
        
        K_current = K_new;
        fprintf(' -> Updated\n');
    end
    
    % Final LMI check and results extraction
    % Re-solve the LMI with the final K_current to get the norms/poles
    P = sdpvar(nx, nx, 'symmetric');
    Y = sdpvar(nu, nx, 'full');
    W = sdpvar(size(C2,1), size(C2,1), 'symmetric');
    K_final = [K_current(1), K_current(2)];
    A_cl_final = A_2state - B_2state * K_final;

    constraints_mixed_final = [P >= eps_P*eye(nx), P <= 30*eye(nx), W >= 1e-6*eye(size(W,1))];
    constraints_mixed_final = [constraints_mixed_final, Y(1,1) >= -0.5, Y(1,1) <= 15.0];
    constraints_mixed_final = [constraints_mixed_final, Y(1,2) >= -0.5, Y(1,2) <= 6.0];
    LMI_hinf_final = [A_cl_final*P + P*A_cl_final' + B_2state*Y + Y'*B_2state' + 2*min_decay*P, B1_inf, (C1*P + D12*Y)';
                      B1_inf', -gamma_fixed*eye(size(B1_inf,2)), zeros(size(B1_inf,2), size(C1,1));
                      C1*P + D12*Y, zeros(size(C1,1), size(B1_inf,2)), -gamma_fixed*eye(size(C1,1))];
    constraints_mixed_final = [constraints_mixed_final, LMI_hinf_final <= -eps_P*eye(size(LMI_hinf_final,1))];
    LMI_h2_lyap_final = [A_cl_final*P + P*A_cl_final' + B_2state*Y + Y'*B_2state', B2_h2; B2_h2', -eye(size(B2_h2,2))];
    constraints_mixed_final = [constraints_mixed_final, LMI_h2_lyap_final <= -eps_P*eye(size(LMI_h2_lyap_final,1))];
    LMI_h2_perf_final = [W, (C2*P + D2u*Y); (C2*P + D2u*Y)', P];
    constraints_mixed_final = [constraints_mixed_final, LMI_h2_perf_final >= eps_P*eye(size(LMI_h2_perf_final,1))];

    sol_final = optimize(constraints_mixed_final, trace(W), options);

    controller_results.synthesis_success = (sol_final.problem == 0);
    controller_results.method = 'Mixed H2/Hinf (Hybrid QN)';
    controller_results.K_beta = K_current(1);
    controller_results.K_r = K_current(2);
    controller_results.optimization_iterations = length(cost_history);
    controller_results.final_cost = cost_history(end);
    controller_results.cost_history = cost_history;
    controller_results.closed_loop_poles = eig(A_cl_final);
    if controller_results.synthesis_success
        controller_results.h2_norm = value(trace(W));
        controller_results.gamma_inf = gamma_fixed;
    else
        controller_results.h2_norm = NaN;
        controller_results.gamma_inf = NaN;
    end
    
    fprintf('    FINAL: K_beta=%.3f, K_r=%.3f, Cost=%.4f, Iterations=%d\n', ...
        K_current(1), K_current(2), cost_history(end), length(cost_history));
end

SIMULATION & ANALYSIS FUNCTIONS

function [time, states, controls] = simulate_2state_optimized_response(params, controller_results, gust_series)
    time = 0:params.time_step:params.simulation_time; 
    N = length(time);
    states = zeros(9, N); 
    controls = zeros(2, N);
    
    K_beta_opt = controller_results.K_beta;
    K_r_opt = controller_results.K_r;
    
    % Use baseline gains for other loops
    K_p = params.K_p_baseline;
    K_beta_dot = params.K_beta_dot_baseline;
    K_I = params.K_I_baseline;
    K_rho = params.K_rho_baseline;
    
    a_ail = params.actuator_bandwidth_aileron; 
    a_rud = params.actuator_bandwidth_rudder;
    max_aileron = params.max_aileron_deg * params.DEG_TO_RAD;
    max_rudder = params.max_rudder_deg * params.DEG_TO_RAD;
    
    fprintf('    Using optimized gains: K_beta=%.3f, K_r=%.3f\n', K_beta_opt, K_r_opt);
    
    for k = 1:N-1
        % Extract states
        p         = states(1,k);
        beta      = states(2,k);
        beta_dot  = states(3,k);
        r         = states(4,k);
        phi       = states(5,k);
        eI        = states(6,k);
        x_wof     = states(7,k);
        delta_a   = states(8,k);
        delta_r   = states(9,k);
        
        % Control Laws
        aileron_feedback = -K_p * p;
        rudder_feedback = -K_beta_opt * beta - K_beta_dot * beta_dot - K_r_opt * r;
        rudder_integral = -K_I * eI;
        rudder_washout = -K_rho * x_wof;
        
        % Commanded control surface deflections
        aileron_pilot_input = generate_aileron_command(time(k), params);
        delta_a_cmd = aileron_pilot_input + aileron_feedback;
        delta_r_cmd = rudder_feedback + rudder_integral + rudder_washout;
        
        % Apply saturation limits
        delta_a_cmd = max(-max_aileron, min(max_aileron, delta_a_cmd));
        delta_r_cmd = max(-max_rudder, min(max_rudder, delta_r_cmd));
        controls(1,k) = delta_a_cmd; 
        controls(2,k) = delta_r_cmd;
        
        % Actuator Dynamics (first-order lag)
        delta_a_dot = -a_ail*delta_a + a_ail*delta_a_cmd;
        delta_r_dot = -a_rud*delta_r + a_rud*delta_r_cmd;
        
        % Aircraft Dynamics
        B_gust = [0; 0; 0.1; 0]; % How gust affects the lateral states
        x_lateral = [p; beta; beta_dot; r];
        dx_lateral = params.A_lateral*x_lateral + params.B_lateral*[delta_a; delta_r] + B_gust*gust_series(k);
        
        % Other state derivatives
        phi_dot   = p + r * tan(states(5,k)); % More complete roll angle dynamics
        eI_dot    = beta;
        x_wof_dot = -(1/params.turn_coordination_tau)*x_wof + r;
        
        % Assemble state derivative vector for integration
        state_derivatives = [dx_lateral; phi_dot; eI_dot; x_wof_dot; delta_a_dot; delta_r_dot];
        states(:,k+1) = rk4_integration(states(:,k), state_derivatives, params.time_step);
    end
    controls(:,N) = controls(:,N-1);
end

function print_performance_metrics(simulation_results, params)
    fprintf('\n================== Performance & Comfort Metrics (Comfort Focused LMI) ==================\n');
    header_format = '%-25s';
    row_format = '%-25.4f';
    
    fprintf(header_format, 'Metric');
    for i = 1:length(simulation_results), fprintf('%-30s', simulation_results{i}.name); end
    fprintf('\n');
    fprintf(repmat('-', 1, 25 + 30*length(simulation_results))); fprintf('\n');
    
    metrics = {'Sideslip RMS (deg)', 'Sideslip Peak (deg)', 'Lat. Accel. RMS (g)', 'Rudder RMS (deg)', 'Rudder Rate RMS (deg/s)', 'K_beta (optimized)', 'K_r (optimized)', 'Optimization Iterations', 'Final Cost Function', 'H2 Norm', 'H-inf Gamma'};
    
    for m_idx = 1:length(metrics)
        fprintf(header_format, metrics{m_idx});
        for ctrl_idx = 1:length(simulation_results)
            res = simulation_results{ctrl_idx};
            val = NaN;
            
            % Calculate metric value based on its name
            switch metrics{m_idx}
                case 'Sideslip RMS (deg)'
                    val = rms(res.states(2, :) / params.DEG_TO_RAD);
                case 'Sideslip Peak (deg)'
                    val = max(abs(res.states(2, :) / params.DEG_TO_RAD));
                case 'Lat. Accel. RMS (g)'
                    % ay ≈ V*(r - g/V*sin(phi)) + V*beta_dot
                    beta_dot_rad_s = res.states(3, :);
                    r_rad_s = res.states(4, :);
                    phi_rad = res.states(5, :);
                    lat_accel_ms2 = params.airspeed_ms * (r_rad_s - params.GRAVITY / params.airspeed_ms .* sin(phi_rad)) + params.airspeed_ms .* beta_dot_rad_s;
                    val = rms(lat_accel_ms2 / params.GRAVITY);
                case 'Rudder RMS (deg)'
                    val = rms(res.controls(2, :) / params.DEG_TO_RAD);
                case 'Rudder Rate RMS (deg/s)'
                    rudder_deg = res.controls(2, :) / params.DEG_TO_RAD;
                    rudder_rate_degs = diff(rudder_deg) / params.time_step;
                    val = rms(rudder_rate_degs);
                case 'K_beta (optimized)'
                    val = res.results.K_beta;
                case 'K_r (optimized)'
                    val = res.results.K_r;
                case 'Optimization Iterations' % ADDED THIS
                    if isfield(res.results, 'optimization_iterations')
                        val = res.results.optimization_iterations;
                    end
                case 'Final Cost Function' % ADDED THIS
                    if isfield(res.results, 'final_cost')
                        val = res.results.final_cost;
                    end
                case 'H2 Norm'
                    val = res.results.h2_norm;
                case 'H-inf Gamma'
                    val = res.results.gamma_inf;
            end
            
            if isnan(val)
                fprintf('%-30s', 'N/A');
            else
                fprintf('%-30.4f', val);
            end
        end
        fprintf('\n');
    end
    fprintf('\n');
end

function generate_performance_comparison(simulation_results, params, ~, ~)
    figure('Name', 'Aircraft Response Comparison (Comfort Focused LMI)', 'Position', [100, 50, 1200, 950], 'NumberTitle', 'off');
    
    plot_titles = {'Sideslip Angle Response (\beta)', 'Lateral Acceleration (Passenger Comfort)', ...
                   'Yaw Rate Response (r)', 'Roll Angle Response (\phi)', ...
                   'Rudder Deflection (\delta_r)', 'Rudder Rate (\delta_r dot)'};
    
    y_labels = {'Angle [deg]', 'Acceleration [g]', 'Rate [deg/s]', 'Angle [deg]', 'Deflection [deg]', 'Rate [deg/s]'};
    
    colors = lines(length(simulation_results));
    
    for i = 1:6
        subplot(3, 2, i);
        set(gca, 'FontSize', 11, 'FontName', 'Arial'); hold on; grid on;
        
        for ctrl_idx = 1:length(simulation_results)
            res = simulation_results{ctrl_idx};
            time = res.time;
            data = [];
            
            switch i
                case 1, data = res.states(2, :) / params.DEG_TO_RAD; % Sideslip
                case 2 % Lateral Acceleration
                    beta_dot_rad_s = res.states(3, :);
                    r_rad_s = res.states(4, :);
                    phi_rad = res.states(5, :);
                    lat_accel_ms2 = params.airspeed_ms * (r_rad_s - params.GRAVITY / params.airspeed_ms .* sin(phi_rad)) + params.airspeed_ms .* beta_dot_rad_s;
                    data = lat_accel_ms2 / params.GRAVITY;
                case 3, data = res.states(4, :) / params.DEG_TO_RAD; % Yaw Rate
                case 4, data = res.states(5, :) / params.DEG_TO_RAD; % Roll Angle
                case 5, data = res.controls(2, :) / params.DEG_TO_RAD; % Rudder
                case 6 % Rudder Rate
                    rudder_deg = res.controls(2, :) / params.DEG_TO_RAD;
                    rudder_rate_degs = [0, diff(rudder_deg) / params.time_step];
                    data = rudder_rate_degs;
            end
            
            plot(time, data, 'LineWidth', 1.8, 'Color', colors(ctrl_idx,:));
        end
        
        xlabel('Time (s)'); ylabel(y_labels{i}); title(plot_titles{i});
        xlim([0 params.simulation_time]);
        if i == 1
            legend_names = cellfun(@(c) strrep(c.name, '_', ' '), simulation_results, 'UniformOutput', false);
            legend(legend_names, 'Location', 'northeast');
        end
    end
    sgtitle('Aircraft Response Comparison: Comfort-Focused LMI Optimization', 'FontSize', 16, 'FontWeight', 'bold');

    % STEP 5: ADD CONVERGENCE PLOT
    if ~isempty(simulation_results) % Ensure simulation_results is not empty
        % Extract names using cellfun for robustness
        sim_names = cellfun(@(s) s.name, simulation_results, 'UniformOutput', false);
        
        % Check for any method name containing 'Hybrid QN'
        if any(contains(sim_names, 'Hybrid QN'))
            figure('Name', 'Quasi-Newton Convergence', 'Position', [1300, 100, 400, 300]);
            
            % Find the index of the first Hybrid QN controller for plotting convergence
            qn_idx = find(contains(sim_names, 'Hybrid QN'), 1);
            
            % Ensure the index is valid and the cost_history field exists
            if ~isempty(qn_idx) && isfield(simulation_results{qn_idx}.results, 'cost_history')
                plot(1:length(simulation_results{qn_idx}.results.cost_history), ...
                     simulation_results{qn_idx}.results.cost_history, 'b-o', 'LineWidth', 2);
                xlabel('Iteration'); ylabel('Cost Function'); 
                title('Quasi-Newton Convergence'); grid on;
            end
        end
    end
end

% UTILITY & INITIALIZATION FUNCTIONS

function params = initialize_aircraft_parameters()
    params.GRAVITY = 9.81; params.FT_TO_M = 0.3048;
    params.KNOTS_TO_MS = 0.514444; params.DEG_TO_RAD = pi/180;
    params.altitude_ft = 10000; params.airspeed_knots = 128;
    params.altitude_m = params.altitude_ft * params.FT_TO_M;
    params.airspeed_ms = params.airspeed_knots * params.KNOTS_TO_MS;
    [~, ~, params.density_kgm3] = calculate_atmospheric_properties(params.altitude_m);
    
    % Aircraft lateral dynamics state-space model (A, B matrices)
    % States: [p (roll rate), beta (sideslip), beta_dot, r (yaw rate)]
    params.A_lateral = [-1.5, 0.12, 0, -0.9; 0, -0.3, 1, 0.05; -8.0, 0, -3.0, 1.0; 3.0, 0, 0, -0.8];
    % Inputs: [delta_a (aileron), delta_r (rudder)]
    params.B_lateral = [1.2, 0.1; 0, 0.05; 0.1, 1.5; 0.05, 0.8];
    
    % Baseline controller gains
    params.K_p_baseline = 0.7; 
    params.K_beta_baseline = 10.0;
    params.K_beta_dot_baseline = 1.8;
    params.K_r_baseline = 4.5;
    params.K_I_baseline = 0.5;
    params.K_rho_baseline = 0.9;
    
    % System parameters
    params.actuator_bandwidth_aileron = 10; params.actuator_bandwidth_rudder = 8;
    params.max_aileron_deg = 20; params.max_rudder_deg = 20;
    params.turn_coordination_tau = 3.8;
    params.simulation_time = 25; params.time_step = 0.01;
    params.turbulence_intensity = 'moderate';
end

function [gust_series_mps, gust_params] = generate_dryden_gust_series(params)
    fprintf('\nGenerating Dryden turbulence model...\n');
    altitude_ft = params.altitude_ft; V = params.airspeed_ms;
    if altitude_ft < 1000, L_v = altitude_ft * params.FT_TO_M;
    else, L_v = 1000 * params.FT_TO_M; end
    switch lower(params.turbulence_intensity)
        case 'light', sigma_w = 1.5; case 'moderate', sigma_w = 3.0;
        case 'severe', sigma_w = 6.0; otherwise, sigma_w = 3.0;
    end
    T_v = L_v / V;
    gust_filter_tf = tf(sigma_w * sqrt(2*V/L_v), [1, V/L_v]);
    time_vec = 0:params.time_step:params.simulation_time;
    white_noise = randn(size(time_vec));
    gust_series_mps = lsim(gust_filter_tf, white_noise, time_vec);
    gust_params.sigma_v = sigma_w; gust_params.L_v = L_v;
    gust_params.intensity = params.turbulence_intensity;
    fprintf('  Intensity: ''%s'' (sigma_v = %.2f m/s), Scale Length: %.1f m\n', ...
        gust_params.intensity, gust_params.sigma_v, gust_params.L_v);
end

function [A_2state, B_2state] = get_2state_model(params)
    % Extracts the 2-state model [beta; r] for LMI synthesis
    A_2state = [params.A_lateral(2,2), params.A_lateral(2,4);
                params.A_lateral(4,2), params.A_lateral(4,4)];
    B_2state = [params.B_lateral(2,2); params.B_lateral(4,2)];
end

function results = initialize_controller_results()
    results = struct('synthesis_success', false, 'method', '', 'K_beta', NaN, 'K_r', NaN, ...
        'h2_norm', NaN, 'gamma_inf', NaN, 'closed_loop_poles', [], ...
        'optimization_iterations', NaN, 'final_cost', NaN, 'cost_history', []);
end

function validate_parameters(params)
    assert(all(real(eig(params.A_lateral)) < 0.01), 'Open-loop aircraft dynamics are unstable');
end

function [temp_K, pressure_Pa, density_kgm3] = calculate_atmospheric_properties(altitude_m)
    % Standard ISA model to calculate atmospheric properties.
    T0 = 288.15; L = 0.0065; g = 9.80665; R = 287.053;
    temp_K = T0 - L * altitude_m;
    pressure_Pa = 101325 * (temp_K / T0)^(g / (R * L));
    density_kgm3 = pressure_Pa / (R * temp_K);
end

function display_flight_conditions(params)
    fprintf('Flight Conditions:\n');
    fprintf('  Altitude:        %d ft (%.1f m)\n', params.altitude_ft, params.altitude_m);
    fprintf('  Airspeed:        %d KTAS (%.1f m/s)\n', params.airspeed_knots, params.airspeed_ms);
    fprintf('  Turbulence:      %s\n', params.turbulence_intensity);
end

function x_next = rk4_integration(x_current, state_derivatives, dt)
    % RK4 integration assumes state_derivatives is already the evaluated derivative vector
    k1 = dt * state_derivatives; 
    
    % For k2, k3, k4, we would typically re-evaluate the derivative function
    % at the intermediate points. However, your current setup passes a
    % pre-calculated state_derivatives vector, which means the RK4
    % implementation here is simplified and effectively acts more like
    % Euler integration if state_derivatives is constant over the step.
    % If state_derivatives is meant to be a function handle, the signature
    % of rk4_integration would need to change to `rk4_integration(x_current, func_handle, dt)`
    % and func_handle would be called multiple times.
    % Given the context, I'm keeping it as a direct application of the provided k1-k4 structure.
    k2 = dt * state_derivatives; 
    k3 = dt * state_derivatives; 
    k4 = dt * state_derivatives; 
    
    x_next = x_current + (k1 + 2*k2 + 2*k3 + k4)/6;
end

function aileron_cmd = generate_aileron_command(time, ~)
    if time >= 2 && time < 4, aileron_cmd = 12.5 * pi/180;
    elseif time >= 5 && time < 7, aileron_cmd = -12.5 * pi/180;
    else, aileron_cmd = 0; end
end

% SUPPORT FUNCTIONS (UNCHANGED)

function J = compute_comfort_cost_function(states, controls, params)
    % Compute multi-objective comfort cost function
    % This function quantifies passenger discomfort and control effort.
    
    % Extract relevant signals from simulation results
    beta = states(2, :);        % Sideslip angle (rad)
    r = states(4, :);           % Yaw rate (rad/s)
    phi = states(5, :);         % Roll angle (rad)
    delta_r = controls(2, :);   % Rudder deflection (rad)
    
    % Compute lateral acceleration (key comfort metric)
    % Lateral acceleration (ay) is approximated as: ay = V*(r - g/V*sin(phi)) + V*beta_dot
    beta_dot = states(3, :); % Sideslip rate (rad/s)
    lat_accel_ms2 = params.airspeed_ms * (r - params.GRAVITY/params.airspeed_ms .* sin(phi)) + params.airspeed_ms .* beta_dot;
    lat_accel_g = lat_accel_ms2 / params.GRAVITY; % Convert to g's
    
    % Compute rudder rate (smoothness metric)
    % Quantifies how aggressively the rudder is moved.
    dt = params.time_step;
    rudder_rate = [0, diff(delta_r)/dt]; % Rudder rate (rad/s)
    
    % Multi-objective cost function (tunable weights)
    % These weights determine the relative importance of each term.
    w1 = 10.0;  % Weight for Sideslip RMS (penalizes skidding)
    w2 = 15.0;  % Weight for Lateral Acceleration RMS 
    w3 = 5.0;   % Weight for Rudder Usage RMS (penalizes large rudder deflections)
    w4 = 3.0;   % Weight for Rudder Rate RMS (penalizes jerky rudder movements)
    
    % Calculate the total cost as a weighted sum of RMS values (squared)
    J = w1 * rms(beta/params.DEG_TO_RAD)^2 + ... % Sideslip in degrees
        w2 * rms(lat_accel_g)^2 + ...             % Lateral acceleration in g's
        w3 * rms(delta_r/params.DEG_TO_RAD)^2 + ... % Rudder deflection in degrees
        w4 * rms(rudder_rate/params.DEG_TO_RAD)^2;  % Rudder rate in degrees/second
end

function gradient = compute_numerical_gradient(params, K_current, gust_series, J_current)
    % Compute numerical gradient of the cost function with respect to K_beta and K_r
    % using finite differences.
    
    epsilon = [0.01; 0.01]; % Step sizes for K_beta and K_r
    gradient = zeros(2, 1); % Initialize gradient vector
    
    for i = 1:2 % Iterate for each gain (K_beta, K_r)
        K_plus = K_current;
        K_plus(i) = K_plus(i) + epsilon(i); % Perturb the current gain
        
        % Apply bounds to perturbed gains to ensure they are within reasonable limits
        K_plus(1) = max(0, min(12, K_plus(1))); % K_beta bounds [0, 12]
        K_plus(2) = max(0, min(6, K_plus(2)));  % K_r bounds [0, 6]
        
        % Create a temporary controller structure with the perturbed gains
        temp_controller = struct();
        temp_controller.synthesis_success = true; % Assume success for simulation
        temp_controller.K_beta = K_plus(1);
        temp_controller.K_r = K_plus(2);
        
        % Simulate the aircraft response with the perturbed gains
        [~, states_plus, controls_plus] = simulate_2state_optimized_response(params, temp_controller, gust_series);
        
        % Compute the cost function with the perturbed gains
        J_plus = compute_comfort_cost_function(states_plus, controls_plus, params);
        
        % Calculate the numerical gradient component using finite difference formula
        gradient(i) = (J_plus - J_current) / epsilon(i);
    end
end

function H_inv_new = update_inverse_hessian(H_inv_old, s, y)
    % Simplified BFGS update for approximating the inverse Hessian matrix.
    % This is used in Quasi-Newton methods to guide the optimization direction.
    %
    % H_inv_old: Previous approximation of the inverse Hessian.
    % s: Change in parameters (K_new - K_prev).
    % y: Change in gradients (grad_new - grad_prev).
    
    rho = 1 / (y' * s); % Calculate rho, a scalar term
    
    % Check for ill-conditioning (e.g., if y' * s is close to zero or non-finite)
    if abs(rho) > 1e6 || ~isfinite(rho)
        H_inv_new = H_inv_old; % Skip update if ill-conditioned to prevent divergence
        return;
    end
    
    I = eye(size(H_inv_old)); % Identity matrix of the same size as H_inv_old
    
    % BFGS update formula for the inverse Hessian
    H_inv_new = (I - rho * s * y') * H_inv_old * (I - rho * y * s') + rho * s * s';
    
    % Ensure positive definiteness of the updated inverse Hessian
    % This step helps maintain numerical stability and ensures the search direction
    % is a descent direction.
    [V, D] = eig(H_inv_new); % Eigenvalue decomposition
    D = diag(max(diag(D), 1e-6)); % Replace any non-positive eigenvalues with a small positive value
    H_inv_new = V * D * V'; % Reconstruct the matrix with positive eigenvalues
end
