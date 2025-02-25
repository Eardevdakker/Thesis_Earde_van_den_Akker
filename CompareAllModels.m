%% CompareAllModels.m
% This master script defines common simulation parameters and passes them
% via the structure 'params' to each run-model function.
% It then plots comparisons for:
%   - Vehicle counts (n_log)
%   - Queue lengths (q_log)
%   - Green times (green_times_log)
%   - Flows (using alpha_leave_log as a proxy)

clear; clc; close all;
yalmip clear
%% Define common parameters (same for all models)
params.n = 1 * [18; 24; 14; 28; 21; 11; 30; 15];
params.S = 1 * [30, 35, 40; 28, 32, 38; 25, 30, 35; 35, 40, 45; ...
                 29, 33, 37; 30, 35, 39; 32, 37, 42; 34, 38, 43];
params.q = 1 * [ 5   10   3;
                 8   12   4;
                 4    8   2;
                 7   15   6;
                 6   10   5;
                 3    5   3;
                 9   14   7;
                 5    8   2];
params.mu = 2 * [8; 10; 9; 11; 10; 8; 12; 9];
% params.beta = [0.2, 0.5, 0.3;
%                0.3, 0.4, 0.3;
%                0.25, 0.5, 0.25;
%                0.4, 0.4, 0.2;
%                0.3, 0.4, 0.3;
%                0.35, 0.4, 0.25;
%                0.3, 0.45, 0.25;
%                0.25, 0.5, 0.25];

params.beta = [0.3, 0.5, 0.2;
               0.3, 0.5, 0.3;
               0.3, 0.5, 0.2;
               0.3, 0.5, 0.2;
               0.2, 0.5, 0.3;
               0.2, 0.5, 0.3;
               0.2, 0.5, 0.3;
               0.2, 0.5, 0.3];


params.total_timesteps    = 50;
params.prediction_horizon = 5;
params.control_horizon    = 5;
params.cycle_times        = [1, 2, 1, 2];
% params.cycle_times        = [1, 1, 1, 1];
params.BigM = 1e3;
params.queusize = 100;
params.difference = 100;
params.model_choice = 2; 
params.greendiff = 15;
params.turnon = 0; 
%Using 0 is is not used
%Using 1 the green difference between green time and min green time to penalize usage that is unnecesary
% Choice 1: Stochastic model 1
% Choice 2: Fixed value model 1
% For the no-pedestrian model and as a base for the pedestrian models:
maxgreen = 2;
params.min_green_times    = 0.01 * ones(8, 3);   % [8 x 3]
params.scaling_factor     = 0.05;%0.05
H = max(params.prediction_horizon, params.control_horizon);
params.max_green_times    = repmat(maxgreen * ones(8, 3), [1, 1, params.total_timesteps+H]);    % [8 x 3]
[pedestriancrossing_cycletimes, pedestriancrossing_Maxgreen] = ...
PedestrianCalculator_Cycle_MaxGreen_delay(params.total_timesteps, H, params.cycle_times, params.max_green_times);

params.pedestriancrossing_cycletimes = pedestriancrossing_cycletimes;
params.pedestriancrossing_Maxgreen = pedestriancrossing_Maxgreen;

%% Run each model with the same input parameters
disp('Running Model 3: With Pedestrians (Cycle Times Delay)');
Data_CycleTimes = runModelPedCycleTimes(params);

disp('Running Model 1: Without Pedestrians');
Data_NoPed = runModelNoPed(params);

disp('Running Model 2: With Pedestrians (Max Green Delay)');
Data_MaxGreen = runModelPedMaxGreen(params);




%% ======================= Comparison Plots =======================

% (A) Vehicle Count Comparison
fig1 = figure('Name','Vehicle Count Comparison','Color','w','Position',[100 100 1200 800]);
t_n = 0:size(Data_NoPed.n_log,2)-1;
avgN_NoPed     = mean(Data_NoPed.n_log,1);
avgN_MaxGreen  = mean(Data_MaxGreen.n_log,1);
avgN_CycleTimes= mean(Data_CycleTimes.n_log,1);
plot(t_n, avgN_NoPed, '-ro','LineWidth',2); hold on;
plot(t_n, avgN_MaxGreen, '-b*','LineWidth',2);
plot(t_n, avgN_CycleTimes, '-gs','LineWidth',2);
xlabel('Timestep'); ylabel('Average Number of Vehicles');
title('Vehicle Count Comparison');
legend('No Pedestrians','Ped MaxGreen','Ped CycleTimes','Location','Best');
grid on;
% Save as high-quality PNG at 300 DPI
print(fig1, 'VehicleCountComparison.png', '-dpng', '-r300');

% (B) Queue Length Comparison
fig2 = figure('Name','Average Queue Length Comparison','Color','w','Position',[100 100 1200 800]);
TotalQueue_NoPed     = squeeze(sum(Data_NoPed.q_log,2));
TotalQueue_MaxGreen  = squeeze(sum(Data_MaxGreen.q_log,2));
TotalQueue_CycleTimes= squeeze(sum(Data_CycleTimes.q_log,2));
avgQueue_NoPed     = mean(TotalQueue_NoPed,1);
avgQueue_MaxGreen  = mean(TotalQueue_MaxGreen,1);
avgQueue_CycleTimes= mean(TotalQueue_CycleTimes,1);
plot(t_n, avgQueue_NoPed, '-ro','LineWidth',2); hold on;
plot(t_n, avgQueue_MaxGreen, '-b*','LineWidth',2);
plot(t_n, avgQueue_CycleTimes, '-gs','LineWidth',2);
xlabel('Timestep'); ylabel('Average Queue Length');
title('Average Queue Length Comparison');
legend('No Pedestrians','Ped MaxGreen','Ped CycleTimes','Location','Best');
grid on;
% Save as high-quality PNG at 300 DPI
print(fig2, 'AverageQueueLengthComparison.png', '-dpng', '-r300');

% (C) Green Times Comparison
fig3 = figure('Name','Average Green Time Comparison','Color','w','Position',[100 100 1200 800]);
t_green = 1:size(Data_NoPed.green_times_log,3);
avgGreen_NoPed     = squeeze(mean(mean(Data_NoPed.green_times_log,1),2));
avgGreen_MaxGreen  = squeeze(mean(mean(Data_MaxGreen.green_times_log,1),2));
avgGreen_CycleTimes= squeeze(mean(mean(Data_CycleTimes.green_times_log,1),2));
plot(t_green, avgGreen_NoPed, '-ro','LineWidth',2); hold on;
plot(t_green, avgGreen_MaxGreen, '-b*','LineWidth',2);
plot(t_green, avgGreen_CycleTimes, '-gs','LineWidth',2);
xlabel('Timestep'); ylabel('Average Green Time');
title('Average Green Time Comparison');
legend('No Pedestrians','Ped MaxGreen','Ped CycleTimes','Location','Best');
grid on;
% Save as high-quality PNG at 300 DPI
print(fig3, 'AverageGreenTimeComparison.png', '-dpng', '-r300');

% (D) Flow Comparison (using alpha_leave_log as proxy)
fig4 = figure('Name','Average Flow Comparison','Color','w','Position',[100 100 1200 800]);
t_flow = 1:size(Data_NoPed.alpha_leave_log,3);
Flows_NoPed     = squeeze(sum(Data_NoPed.alpha_leave_log,2));
Flows_MaxGreen  = squeeze(sum(Data_MaxGreen.alpha_leave_log,2));
Flows_CycleTimes= squeeze(sum(Data_CycleTimes.alpha_leave_log,2));
avgFlow_NoPed     = mean(Flows_NoPed,1);
avgFlow_MaxGreen  = mean(Flows_MaxGreen,1);
avgFlow_CycleTimes= mean(Flows_CycleTimes,1);
plot(t_flow, avgFlow_NoPed, '-ro','LineWidth',2); hold on;
plot(t_flow, avgFlow_MaxGreen, '-b*','LineWidth',2);
plot(t_flow, avgFlow_CycleTimes, '-gs','LineWidth',2);
xlabel('Timestep'); ylabel('Average Flow (veh/time)');
title('Average Flow Comparison');
legend('No Pedestrians','Ped MaxGreen','Ped CycleTimes','Location','Best');
grid on;
% Save as high-quality PNG at 300 DPI
print(fig4, 'AverageFlowComparison.png', '-dpng', '-r300');
