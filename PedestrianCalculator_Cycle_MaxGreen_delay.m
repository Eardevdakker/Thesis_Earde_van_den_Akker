function [pedestriancrossing_cycletimes, pedestriancrossing_maxgreentimes] = ...
    PedestrianCalculator_Cycle_MaxGreen_delay(total_timesteps, H, cycle_times, max_green_times)
% PEDSTRIANCROSSING_CYCLE_MAXGREEN_DELAY
%
% This function simulates pedestrian flows using Model 1’s approach and updates:
%   1. pedestriancrossing_cycletimes - cycle times computed as in Model 1.
%   2. pedestriancrossing_maxgreentimes - max green times updated per Model 2.
%
% Inputs:
%   total_timesteps - number of simulation time steps (without headway H)
%   H               - additional headway time steps (total iterations = total_timesteps+H)
%   cycle_times     - matrix of cycle times to be scaled
%   max_green_times - matrix (or 3D array) of max green times. If a 2D matrix is provided,
%                     it will be replicated along the third dimension.
%
% Outputs:
%   pedestriancrossing_cycletimes - scaled cycle times output.
%   pedestriancrossing_maxgreentimes - updated max green times output.

    % Model 1 pedestrian parameters
    num_iterations = total_timesteps + H;
    delay = 3;                          % Delay in iterations (Model 1)
    initial_pedestrians = [10, 10, 10, 10];  % Initial pedestrian counts
    entering_rate = [10, 10, 10, 10];        % Pedestrians entering per iteration

    % Initialize pedestrian counts and history
    pedestrians = initial_pedestrians;
    history = zeros(num_iterations + 1, 4);
    history(1, :) = pedestrians;

    % Preallocate the matrix for cycle time scaling factors
    crowding_matrix = zeros(1, 4, num_iterations);

    % Create a cell array of queues (one per point) for delayed transitions
    queues = cell(1, 4);
    for i = 1:4
        queues{i} = zeros(1, delay);
    end

    % If max_green_times is 2D, replicate it along the third dimension.
    if ndims(max_green_times) < 3 || size(max_green_times, 3) < num_iterations
        max_green_times = repmat(max_green_times, [1, 1, num_iterations]);
    end

    % Simulation loop over iterations
    for iteration = 1:num_iterations
        new_pedestrians = zeros(1, 4);
        % Temporary vector to store base (unscaled) green times per point
        green_time_vector = zeros(1, 4);

        for i = 1:4
            % Generate random turning rates that sum to 1
            turning_rates = rand(1, 4);
            turning_rates = turning_rates / sum(turning_rates);

            % Number of pedestrians leaving point i
            leaving_pedestrians = pedestrians(i);

            % Distribute leaving pedestrians to other points (with delay)
            for j = 1:4
                if j ~= i
                    queues{j} = [queues{j}, floor(leaving_pedestrians * turning_rates(j))];
                else
                    queues{j} = [queues{j}, 0];  % No one stays at the same point
                end
            end

            % Calculate crowding and corresponding green time
            crowding = leaving_pedestrians / max(1, sum(pedestrians));
            base_green = 75 + (1 - crowding) * 25;
            cycle_green = 0.01 * base_green; % Scale factor for cycle times

            % Store the computed green time in the cycle times matrix
            crowding_matrix(1, i, iteration) = cycle_green;
            % Save base green time for max green times update (will also be scaled by 0.01)
            green_time_vector(i) = base_green;

            % Compute pedestrians exiting the network
            exiting = floor(leaving_pedestrians * turning_rates(i));
            new_pedestrians(i) = new_pedestrians(i) - exiting;
        end

        % Update max green times per Model 2 mapping:
        % Mapping:
        %   Point 1 --> rows 4 & 5
        %   Point 2 --> rows 1 & 6
        %   Point 3 --> rows 2 & 7
        %   Point 4 --> rows 3 & 8
        cycle_green1 = 0.01 * green_time_vector(1);
        cycle_green2 = 0.01 * green_time_vector(2);
        cycle_green3 = 0.01 * green_time_vector(3);
        cycle_green4 = 0.01 * green_time_vector(4);

        max_green_times(4, :, iteration) = max_green_times(4, :, iteration) * cycle_green1;
        max_green_times(5, :, iteration) = max_green_times(5, :, iteration) * cycle_green1;
        max_green_times(1, :, iteration) = max_green_times(1, :, iteration) * cycle_green2;
        max_green_times(6, :, iteration) = max_green_times(6, :, iteration) * cycle_green2;
        max_green_times(2, :, iteration) = max_green_times(2, :, iteration) * cycle_green3;
        max_green_times(7, :, iteration) = max_green_times(7, :, iteration) * cycle_green3;
        max_green_times(3, :, iteration) = max_green_times(3, :, iteration) * cycle_green4;
        max_green_times(8, :, iteration) = max_green_times(8, :, iteration) * cycle_green4;

        % Update pedestrian counts from queues (delayed arrivals)
        for i = 1:4
            arriving = queues{i}(1);
            queues{i}(1) = []; % Remove the first element
            new_pedestrians(i) = new_pedestrians(i) + arriving;
        end

        % Add new entering pedestrians
        new_pedestrians = new_pedestrians + entering_rate;

        % Update the pedestrian counts and record history
        pedestrians = new_pedestrians;
        history(iteration + 1, :) = pedestrians;
    end

    % Final outputs:
    pedestriancrossing_cycletimes = cycle_times .* crowding_matrix;
    pedestriancrossing_maxgreentimes = max_green_times;

    % (Optional) Plot pedestrian counts over time and save the figure
    fig_ped = figure('Name','Pedestrian Counts Over Time','Color','w','Position',[100 100 1200 800]);
    hold on;
    for i = 1:4
        plot(0:num_iterations, history(:, i), 'LineWidth',2, 'DisplayName', ['Point ' num2str(i)]);
    end
    title('Pedestrian Counts at Each Point Over Time');
    xlabel('Iterations');
    ylabel('Number of Pedestrians');
    legend('Location','Best');
    grid on;
    hold off;
    
    % Save the figure as a high-resolution PNG
    print(fig_ped, 'PedestrianCounts.png', '-dpng', '-r300');
end
