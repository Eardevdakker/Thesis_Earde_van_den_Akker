function Data = runModelNoPed(params)
% runModelNoPed runs the simulation without pedestrians.
% Input: params (a structure containing all initialization parameters)
% Output: Data, a structure containing logged outputs:
%   n_log           - [8 x (total_timesteps+1)] vehicle counts
%   q_log           - [8 x 3 x (total_timesteps+1)] queue lengths
%   green_times_log - [8 x 3 x (total_timesteps)] computed green times
%   alpha_leave_log - [8 x 3 x (total_timesteps)] leaving flows (proxy)

    yalmip('clear');

    %% Use parameters from the input structure
    % State variables
    current_state.n    = params.n;
    current_state.S    = params.S;
    current_state.q    = params.q;
    current_state.mu   = params.mu;
    current_state.beta = params.beta;

    % Simulation and control parameters
    total_timesteps    = params.total_timesteps;
    prediction_horizon = params.prediction_horizon;
    control_horizon    = params.control_horizon;
    cycle_times        = params.cycle_times;
    max_green_times    = params.max_green_times(:,:,1);  % [8 x 3]
    min_green_times    = params.min_green_times;  % [8 x 3]
    scaling_factor     = params.scaling_factor;
    model_choice = params.model_choice;
    H = max(prediction_horizon, control_horizon);

    %% Set up alpha_enter_ext_vals (as in Script 1.1)
    alpha_enter_ext_vals = zeros(8, total_timesteps+H);
   if model_choice == 1
    for k = 1:total_timesteps+H
        alpha_enter_ext_vals(1,k) = scaling_factor * (5 * sin(0.1 * k) + 100);
        alpha_enter_ext_vals(2,k) = scaling_factor * (4 * cos(0.1 * k) + 80);
        alpha_enter_ext_vals(3,k) = scaling_factor * (3 * sin(0.2 * k) + 70);
        alpha_enter_ext_vals(4,k) = scaling_factor * (6 * cos(0.2 * k) + 90);
        alpha_enter_ext_vals(5,k) = scaling_factor * (2 * sin(0.3 * k) + 60);
        alpha_enter_ext_vals(6,k) = scaling_factor * (5 * cos(0.3 * k) + 70);
        alpha_enter_ext_vals(7,k) = scaling_factor * (4 * sin(0.4 * k) + 50);
        alpha_enter_ext_vals(8,k) = scaling_factor * (3 * cos(0.4 * k) + 60);
    end    
   end
   if model_choice == 2
    for k = 1:(total_timesteps+H)
        alpha_enter_ext_vals(1,k) = scaling_factor * 30;
        alpha_enter_ext_vals(2,k) = scaling_factor * 20;
        alpha_enter_ext_vals(3,k) = scaling_factor * 30;
        alpha_enter_ext_vals(4,k) = scaling_factor * 20;
        alpha_enter_ext_vals(5,k) = scaling_factor * 30;
        alpha_enter_ext_vals(6,k) = scaling_factor * 20;
        alpha_enter_ext_vals(7,k) = scaling_factor * 30;
        alpha_enter_ext_vals(8,k) = scaling_factor * 20;
    end
   end
    current_state.alpha_enter_ext_vals = alpha_enter_ext_vals;
    current_state.alpha_enter_prev = alpha_enter_ext_vals(:,1);
    current_state.alpha_leave_prev = zeros(8,1);

    %% Preallocate logs (as in Script 1.1)
    green_times_log = zeros(8, 3, total_timesteps);
    q_log           = zeros(8, 3, total_timesteps+1);
    n_log           = zeros(8, total_timesteps+1);
    alpha_enter_log = zeros(8, total_timesteps);
    alpha_arrive_log= zeros(8, 3, total_timesteps);
    alpha_leave_log = zeros(8, 3, total_timesteps);

    % Record initial state
    q_log(:,:,1) = current_state.q;
    n_log(:,1)   = current_state.n;

    %% Simulation loop (Scripts 1.1–1.3)
    for timestep = 1:total_timesteps
        disp(['Model NoPed - Timestep: ', num2str(timestep)]);
        
        % Solve the MILP MPC (without pedestrian delay)
        [optimal_green_times, optimal_alpha_enter, optimal_alpha_arrive, optimal_alpha_leave, alpha_leave_update] = ...
            MAINMPCV7_Without_pedestrian_delay(params, current_state, timestep, prediction_horizon, control_horizon, cycle_times, min_green_times, max_green_times);
        
        % Log the first-step decision
        green_times_log(:,:,timestep) = optimal_green_times;
        current_state.alpha_enter_prev = optimal_alpha_enter(:,1);
        current_state.alpha_leave_prev = alpha_leave_update;
        alpha_enter_log(:,timestep)     = optimal_alpha_enter(:,1);
        alpha_arrive_log(:,:,timestep)  = optimal_alpha_arrive(:,:,1);
        alpha_leave_log(:,:,timestep)   = optimal_alpha_leave(:,:,1);
        
        % Update state (using the first-step solution)
        current_state = MAINCALCULATEQUESV7_Without_pedestrian_delay(current_state, alpha_enter_log(:,timestep), alpha_arrive_log(:,:,timestep), alpha_leave_log(:,:,timestep));
        q_log(:,:,timestep+1) = current_state.q;
        n_log(:,timestep+1)   = current_state.n;
    end
    t_action = 1:total_timesteps;
    t_queue  = 0:total_timesteps;
    %% --- Additional Plots ---
    % Define time vector for flows/green times (each timestep corresponds to an action)
    t = 1:total_timesteps;
    
    % (1) Combined Leaving Flows (merge the 3 directions per node)
    fig_leave = figure('Name','Combined Leaving Flows','Color','w','Position',[100 100 1200 800]);
    % Sum across the second dimension (directions): resulting in a [8 x total_timesteps] matrix.
    combinedLeavingFlows = squeeze(sum(alpha_leave_log, 2));  
    for i = 1:8
        plot(t, combinedLeavingFlows(i,:), 'LineWidth',2); hold on;
    end
    xlabel('Timestep'); ylabel('Leaving Flows (combined)');
    title('Combined Leaving Flows (Model NoPed)');
    legend(arrayfun(@(x) sprintf('Node %d', x), 1:8, 'UniformOutput', false), 'Location','Best');
    grid on;
    print(fig_leave, 'NoPed_CombinedLeavingFlows.png', '-dpng', '-r300');
    
    % (2) Combined Green Times (merge the 3 directions per node)
    fig_green = figure('Name','Combined Green Times','Color','w','Position',[100 100 1200 800]);
    % Sum green times along the second dimension: resulting in [8 x total_timesteps]
    combinedGreenTimes = squeeze(sum(green_times_log, 2));
    for i = 1:8
        plot(t, combinedGreenTimes(i,:), 'LineWidth',2); hold on;
    end
    xlabel('Timestep'); ylabel('Green Times (combined)');
    title('Combined Green Times (Model NoPed)');
    legend(arrayfun(@(x) sprintf('Node %d', x), 1:8, 'UniformOutput', false), 'Location','Best');
    grid on;
    print(fig_green, 'NoPed_CombinedGreenTimes.png', '-dpng', '-r300');
    
    % (3) Entering Flows (already logged per node)
    fig_enter = figure('Name','Entering Flows','Color','w','Position',[100 100 1200 800]);
    for i = 1:8
        plot(t, alpha_enter_log(i,:), 'LineWidth',2); hold on;
    end
    xlabel('Timestep'); ylabel('Entering Flows');
    title('Entering Flows (Model NoPed)');
    legend(arrayfun(@(x) sprintf('Node %d', x), 1:8, 'UniformOutput', false), 'Location','Best');
    grid on;
    print(fig_enter, 'NoPed_EnteringFlows.png', '-dpng', '-r300');
    
    % (4) Combined Arriving Flows (merge the 3 directions per node)
    fig_arrive = figure('Name','Combined Arriving Flows','Color','w','Position',[100 100 1200 800]);
    combinedArrivingFlows = squeeze(sum(alpha_arrive_log, 2));
    for i = 1:8
        plot(t, combinedArrivingFlows(i,:), 'LineWidth',2); hold on;
    end
    xlabel('Timestep'); ylabel('Arriving Flows (combined)');
    title('Combined Arriving Flows (Model NoPed)');
    legend(arrayfun(@(x) sprintf('Node %d', x), 1:8, 'UniformOutput', false), 'Location','Best');
    grid on;
    print(fig_arrive, 'NoPed_CombinedArrivingFlows.png', '-dpng', '-r300');
    
    % (5) Combined Queue Lengths
    fig_queue = figure('Name','Combined Queue Lengths','Color','w','Position',[100 100 1200 800]);
    combinedQueues = squeeze(sum(q_log, 2));  % [8 x (total_timesteps+1)]
    for i = 1:8
        plot(t_queue, combinedQueues(i,:), 'LineWidth',2); hold on;
    end
    xlabel('Timestep'); ylabel('Combined Queue Lengths');
    title('Combined Queue Lengths (Model NoPed)');
    legend(arrayfun(@(x) sprintf('Node %d', x), 1:8, 'UniformOutput', false), 'Location','Best');
    grid on;
    print(fig_queue, 'NoPed_CombinedQueueLengths.png', '-dpng', '-r300');

    %% Package logged data into output structure
    Data.n_log           = n_log;
    Data.q_log           = q_log;
    Data.green_times_log = green_times_log;
    Data.alpha_leave_log = alpha_leave_log;
    % (Optionally, include alpha_enter_log and alpha_arrive_log if needed)
    Data.alpha_enter_log = alpha_enter_log;
    Data.alpha_arrive_log = alpha_arrive_log;
end