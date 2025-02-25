function [optimal_green_times, optimal_alpha_enter, ...
          optimal_alpha_arrive, optimal_alpha_leave, alpha_leave_update] = ...
          MAINMPCV7_pedestrian_cycletimes_delay(params, current_state, timestep, prediction_horizon, ...
                  control_horizon, cycle_times, ...
                  max_green_times, min_green_times, pedestriancrossing_cycletimes)

    %% 1) Extract parameters
    num_links = 8;
    num_directions = 3;
    % Constant delay parameters (example values)
    tau_const = 1.5;            % constant delay in time steps (e.g., 1.2 steps)
    tau_int = floor(tau_const);   % integer part (here, 1)
    gamma_const = tau_const - tau_int;  % fractional part (here, 0.2)

    S = current_state.S;
    mu = current_state.mu;
    beta = current_state.beta;
    q0 = current_state.q;   % [num_links, num_directions]
    n0 = current_state.n;   % [num_links, 1]
    alpha_enter_ext_vals = current_state.alpha_enter_ext_vals(:, timestep : timestep + prediction_horizon); 
                            % [num_links, >=prediction_horizon]
    turnon = params.turnon;
    greendiffscale = params.greendiff;
                        
    alpha_enter_prev = current_state.alpha_enter_prev;  % [num_links x 1]
    alpha_leave_prev = current_state.alpha_leave_prev;                        
    T_c = lcm(cycle_times(1), ...
              lcm(cycle_times(2), ...
              lcm(cycle_times(3), cycle_times(4))));


    %% 2) Dimensions and Decision Variables
    H = max(prediction_horizon, control_horizon);

    green_times = sdpvar(num_links, num_directions, H, 'full');
    n = sdpvar(num_links, prediction_horizon+1, 'full');
    q = sdpvar(num_links, num_directions, prediction_horizon+1, 'full');

    alpha_enter  = sdpvar(num_links, prediction_horizon, 'full');
    alpha_arrive = sdpvar(num_links, num_directions, prediction_horizon, 'full');
    alpha_leave  = sdpvar(num_links, num_directions, prediction_horizon, 'full');

    % For min() linearization
    temp   = sdpvar(num_links, num_directions, prediction_horizon, 'full'); 
    delta1 = binvar(num_links, num_directions, prediction_horizon, 'full'); 
    delta2 = binvar(num_links, num_directions, prediction_horizon, 'full');

    constraints = [];
    cost = 0;

    %% 3) Initial Conditions
    constraints = [constraints, n(:,1) == n0];
    constraints = [constraints, q(:,:,1) == q0];

    %% 4) Green Time Bounds & Cycle Constraints
        intersection_links = [4, 5; 
                      1, 6; 
                      7, 2; 
                      8, 3]; 
    
    for k = 1:H
        for i = 1:num_links
            for d = 1:num_directions
                constraints = [constraints, ...
                    min_green_times(i,d) <= green_times(i,d,k) <= max_green_times(i,d)];
            end
        end
        % Intersection cycle-time constraint
        
         for inter = 1:size(intersection_links,1)
            L = intersection_links(inter, :);
            constraints = [constraints, ...
                sum(sum(green_times(L,:,k))) == pedestriancrossing_cycletimes(inter)];
         end
    end

    % Debug check
    for i = 1:num_links
        if sum(min_green_times(i,:)) > cycle_times( ceil(i/2) )
            disp(['Intersection ' num2str(ceil(i/2)) ...
                  ' min green times exceed cycle time.']);
        end
    end

    %% 5) Build Flow & State-Update Constraints for k=1..prediction_horizon
    bigM = params.BigM; 

    for k = 1:prediction_horizon


        % 5a) alpha_enter
        if k == 1
            % alpha_enter(:,1) = alpha_enter_ext_vals(:,1)
            constraints = [constraints, ...
                alpha_enter(:,1) == alpha_enter_ext_vals(:,1)+alpha_leave_prev];
%         else
%             % Single constraint per link "i"
%             constraints = [constraints, ...
%                 alpha_enter(1,k) == alpha_enter_ext_vals(1,k) + alpha_leave(7,3,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(2,k) == alpha_enter_ext_vals(2,k) + alpha_leave(8,1,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(3,k) == alpha_enter_ext_vals(3,k) + alpha_leave(5,1,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(4,k) == alpha_enter_ext_vals(4,k) + alpha_leave(1,3,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(5,k) == alpha_enter_ext_vals(5,k) + alpha_leave(4,3,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(6,k) == alpha_enter_ext_vals(6,k) + alpha_leave(3,3,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(7,k) == alpha_enter_ext_vals(7,k) + alpha_leave(6,3,k-1)];
%             constraints = [constraints, ...
%                 alpha_enter(8,k) == alpha_enter_ext_vals(8,k) + alpha_leave(2,1,k-1)];
%         end

        else
            % Single constraint per link "i"
            constraints = [constraints, ...
                alpha_enter(1,k) == alpha_enter_ext_vals(1,k) + alpha_leave(4,3,k-1)];
            constraints = [constraints, ...
                alpha_enter(2,k) == alpha_enter_ext_vals(2,k) + alpha_leave(1,3,k-1)];
            constraints = [constraints, ...
                alpha_enter(3,k) == alpha_enter_ext_vals(3,k) + alpha_leave(2,3,k-1)];
            constraints = [constraints, ...
                alpha_enter(4,k) == alpha_enter_ext_vals(4,k) + alpha_leave(3,3,k-1)];
            constraints = [constraints, ...
                alpha_enter(5,k) == alpha_enter_ext_vals(5,k) + alpha_leave(6,1,k-1)];
            constraints = [constraints, ...
                alpha_enter(6,k) == alpha_enter_ext_vals(6,k) + alpha_leave(7,1,k-1)];
            constraints = [constraints, ...
                alpha_enter(7,k) == alpha_enter_ext_vals(7,k) + alpha_leave(8,1,k-1)];
            constraints = [constraints, ...
                alpha_enter(8,k) == alpha_enter_ext_vals(8,k) + alpha_leave(5,1,k-1)];
        end


        % 5b) alpha_arrive(i,d,k) = beta(i,d)*alpha_enter(i,k)
        for i = 1:num_links
                for d = 1:num_directions
                    if k == 1
                        % For the first prediction step, use stored previous entering flow.
                        % Since (k - tau_int) would be <= 0, we use alpha_enter_prev.
                        constraints = [constraints, ...
                            alpha_arrive(i,d,1) == beta(i,d)*((1-gamma_const)*alpha_enter_prev(i) + gamma_const*alpha_enter_prev(i))];
                    else
                        % For k > 1, determine the indices for delayed entering flow.
                        idx1 = k - tau_int;      % intended index for delay
                        idx2 = k - tau_int - 1;  % previous time step for the convex combination

                        % If the index is below 1, use alpha_enter_prev.
                        if idx1 < 1
                            term1 = alpha_enter_prev(i);
                        else
                            term1 = alpha_enter(i, idx1);
                        end
                        if idx2 < 1
                            term2 = alpha_enter_prev(i);
                        else
                            term2 = alpha_enter(i, idx2);
                        end

                        constraints = [constraints, ...
                            alpha_arrive(i,d,k) == beta(i,d)*((1-gamma_const)*term1 + gamma_const*term2)];
                    end
                end
            end

        % 5c) alpha_leave <= min( capacity, queue+arrive, space )
        for i = 1:num_links
            cyc_time = cycle_times(ceil(i/2));
            for d = 1:num_directions

                a_expr = beta(i,d) * mu(i) * ...
                         green_times(i,d, min(k,control_horizon)) / cyc_time;
                b_expr = q(i,d,k)/cyc_time + alpha_arrive(i,d,k);
                c_expr = beta(i,d)*(S(i,d) - n(i,k))/cyc_time;


                % Step 1: temp = min(a_expr, b_expr)
                constraints = [constraints, temp(i,d,k) <= a_expr];
                constraints = [constraints, temp(i,d,k) <= b_expr];
                constraints = [constraints, ...
                    temp(i,d,k) >= a_expr - bigM*(1 - delta1(i,d,k))];
                constraints = [constraints, ...
                    temp(i,d,k) >= b_expr - bigM*(delta1(i,d,k))];

                % Step 2: alpha_leave(i,d,k) = min( temp, c_expr )
                constraints = [constraints, alpha_leave(i,d,k) <= temp(i,d,k)];
                constraints = [constraints, alpha_leave(i,d,k) <= c_expr];
                constraints = [constraints, ...
                    alpha_leave(i,d,k) >= temp(i,d,k) - bigM*(1 - delta2(i,d,k))];
                constraints = [constraints, ...
                    alpha_leave(i,d,k) >= c_expr - bigM*(delta2(i,d,k))];
            end
        end
        

        % 5d) State updates: n(k+1), q(k+1)
        
                alpha_leave_values = [
            alpha_leave(4,3),
            alpha_leave(1,3),
            alpha_leave(2,3),
            alpha_leave(3,3),
            alpha_leave(6,1),
            alpha_leave(7,1),
            alpha_leave(8,1),
            alpha_leave(5,1)
        ];

        
        sum_leave_k = sum(alpha_leave_values) * T_c;
        constraints = [constraints, ...
            n(:,k+1) == n(:,k) + alpha_enter(:,k)*T_c - sum_leave_k];

        new_q = q(:,:,k) + (alpha_arrive(:,:,k) - alpha_leave(:,:,k)) * T_c;
        constraints = [constraints, q(:,:,k+1) == new_q];
        constraints = [constraints, q(:,:,k+1) >= 0];
        constraints = [constraints, n(:,k+1) >= 0];

        
                % alpha_leave >= 0
          for i=1:num_links
            for d=1:num_directions
              constraints = [constraints, alpha_leave(i,d,k) >= 0];
            end
          end


        % alpha_enter >= 0
          constraints = [constraints, alpha_enter(:,k) >= 0];
        

        % alpha_arrive >= 0
          for i=1:num_links
            for d=1:num_directions
              constraints = [constraints, alpha_arrive(i,d,k) >= 0];
            end
          end
        

%% Costs
cost = 0;
        queuesize = (norm(q(:,:,k), 'fro'));
    scalar = params.queusize;
    cost = cost + scalar * queuesize;
%          leq1 =  q(1,3,k) + q(1,2,k) + q(1,1,k);   
%          leq2 =  q(2,3,k) + q(2,2,k) + q(2,1,k);
%          leq3 =  q(3,3,k) + q(3,2,k) + q(3,1,k);
%          leq4 =  q(4,3,k) + q(4,2,k) + q(4,1,k);
%          leq5 =  q(5,1,k) + q(5,2,k) + q(5,3,k);
%          leq6 =  q(6,1,k) + q(6,2,k) + q(6,3,k);
%          leq7 =  q(7,1,k) + q(7,2,k) + q(7,3,k);
%          leq8 =  q(8,1,k) + q(8,2,k) + q(8,3,k);
%         cost = cost + scalar*(leq1 + leq2 + leq3 + leq4 + leq5 +leq6 +leq7 +leq8);
% 



%    
%     if prediction_horizon > k+2
%     queuesize2 = scalar * (norm(value(q(:,:,k+2)), 'fro'));
%     cost = cost + queuesize2;
%     disp('Queue costs size:');
%     disp(value(queuesize2));  
%     end
    diff= params.difference;
         eq1 =  max(0, q(1,3,k) - (q(1,2,k) + q(1,1,k)));   
         eq2 =  max(0, q(2,3,k) - (q(2,2,k) + q(2,1,k)));
         eq3 =  max(0, q(3,3,k) - (q(3,2,k) + q(3,1,k)));
         eq4 =  max(0, q(4,3,k) - (q(4,2,k) + q(4,1,k)));
         eq5 =  max(0, q(5,1,k) - (q(5,2,k) + q(5,3,k)));
         eq6 =  max(0, q(6,1,k) - (q(6,2,k) + q(6,3,k)));
         eq7 =  max(0, q(7,1,k) - (q(7,2,k) + q(7,3,k)));
         eq8 =  max(0, q(8,1,k) - (q(8,2,k) + q(8,3,k)));
    
    cost = cost + diff*(eq1 + eq2 + eq3 + eq4 + eq5 +eq6 +eq7 +eq8);

%      %   Add linear penalty for "under-used" green times
%     for i = 1:num_links
%         for d = 1:num_directions
%             cost = cost + lambdaGreen * (max_green_times(i,d) - green_times(i,d,k));
%         end
%     end

    
%             alphaGreenNotMax = 10;  % <--- you choose the weight
%         for i = 1:num_links
%             for d = 1:num_directions
%                 % If you want to penalize the actual MPC step's green times:
%                 cost = cost + alphaGreenNotMax * ( ...
%                     max_green_times(i,d) - green_times(i,d,min(k,control_horizon)) );
%             end
%         end
%     end
%     lambdaImbalance = 1e3;  % Weight for inflow/outflow imbalance penalty
%         % Inflow/outflow imbalance penalty
%     imbalance = max(0,norm((alpha_arrive(:,:,k)-alpha_leave(:,:,k)), 'fro'));  % Difference between inflow and outflow
%     cost = cost + lambdaImbalance * imbalance;
%     lambdasize = 1e3;
%     cost = cost + lambdasize*(q(1,3,k+1)+q(2,3,k+1)+q(3,3,k+1)+q(4,3,k+1)+q(5,1,k+1)+q(6,1,k+1)+q(7,1,k+1)+q(8,1,k+1));
%     %% 6) Solve Once for the Entire Horizon

if turnon == 1
    scale = greendiffscale;
    % Create a temporary 2-D sdpvar for the difference at time step k.
    temp_excess = sdpvar(num_links, num_directions, 'full');
    for i = 1:num_links
        for d = 1:num_directions
            temp_excess(i,d) = green_times(i,d,k) - min_green_times(i,d);
        end
    end
    % Compute the Frobenius norm of the 2-D difference matrix.
    greendiff(k) = norm(temp_excess, 'fro');
    cost = cost + scale * greendiff(k);
end


    end
    options = sdpsettings('solver','gurobi','verbose',1,'debug',1);
    sol = optimize(constraints, cost, options);
    disp(value(queuesize));
    
            alpha_leave_values = [
            alpha_leave(4,3,1),
            alpha_leave(1,3,1),
            alpha_leave(2,3,1),
            alpha_leave(3,3,1),
            alpha_leave(6,1,1),
            alpha_leave(7,1,1),
            alpha_leave(8,1,1),
            alpha_leave(5,1,1)
            ];
    
            if sol.problem ~= 0
%         if value(queuesize) == 0
%             if k == 2
%             optimal_green_times  = value(green_times(:,:,2));   % first step
%             optimal_alpha_enter  = value(alpha_enter(:,2));
%             optimal_alpha_arrive = value(alpha_arrive(:,:,2));
%             optimal_alpha_leave  = value(alpha_leave(:,:,2)); 
%             else
%         disp('Optimal situation quees at minimum')
%         optimal_green_times  = value(green_times(:,:,1));   % first step
%         optimal_alpha_enter  = value(alpha_enter(:,1));
%         optimal_alpha_arrive = value(alpha_arrive(:,:,1));
%         optimal_alpha_leave  = value(alpha_leave(:,:,1));
%         alpha_leave_update = value(alpha_leave_values);
%             end
%         else
        warning('No feasible solution found');
%         end
        
    
            else   
                
                % 7) Extract Solutions
        
        optimal_green_times  = value(green_times(:,:,1));   % first step
        optimal_alpha_enter  = value(alpha_enter(:,1));
        optimal_alpha_arrive = value(alpha_arrive(:,:,1));
        optimal_alpha_leave  = value(alpha_leave(:,:,1));
        alpha_leave_update = value(alpha_leave_values);
%         disp(optimal_alpha_enter);
%         disp(optimal_alpha_arrive);
%         disp(optimal_alpha_leave);
%         disp(T_c)
%         disp(value(new_q(:,:,1)));
%   disp(value(imbalance));


        
            end
