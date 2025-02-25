function [updated_state] = MAINCALCULATEQUESV7_pedestrian_cycletimes_delay(current_state, ...
                                               alpha_enter_1, ...
                                               alpha_arrive_1, ...
                                               alpha_leave_1)
    % No loops for 'k=1..horizon' here, since we only apply the FIRST step

    % Constants
    cycle_times = [1, 2, 1, 2];
    T_c = lcm(cycle_times(1), ...
              lcm(cycle_times(2), ...
              lcm(cycle_times(3), cycle_times(4))));

    % Extract old states
    n_old = current_state.n;  % [8x1]
    q_old = current_state.q;  % [8x3]

% Extract the specific alpha_leave values based on the constraints
% alpha_arrive_values = [
%     alpha_arrive_1(4,3),
%     alpha_arrive_1(1,3),
%     alpha_arrive_1(2,3),
%     alpha_arrive_1(3,3),
%     alpha_arrive_1(6,1),
%     alpha_arrive_1(7,1),
%     alpha_arrive_1(8,1),
%     alpha_arrive_1(5,1)
% ];

% alpha_leave_values = [
%     alpha_leave_1(4,3),
%     alpha_leave_1(1,3),
%     alpha_leave_1(2,3),
%     alpha_leave_1(3,3),
%     alpha_leave_1(6,1),
%     alpha_leave_1(7,1),
%     alpha_leave_1(8,1),
%     alpha_leave_1(5,1)
% ];

alpha_leave_values = [
    alpha_leave_1(1,1,1)+alpha_leave_1(1,2,1) +  alpha_leave_1(1,3,1),
    alpha_leave_1(2,1,1)+alpha_leave_1(2,2,1) +  alpha_leave_1(2,3,1),
    alpha_leave_1(3,1,1)+alpha_leave_1(3,2,1) +  alpha_leave_1(3,3,1),
    alpha_leave_1(4,1,1)+alpha_leave_1(4,2,1) +  alpha_leave_1(4,3,1),
    alpha_leave_1(5,3,1)+alpha_leave_1(5,2,1) +  alpha_leave_1(5,1,1),
    alpha_leave_1(6,3,1)+alpha_leave_1(6,2,1) +  alpha_leave_1(6,1,1),
    alpha_leave_1(7,3,1)+alpha_leave_1(7,2,1) +  alpha_leave_1(7,1,1),
    alpha_leave_1(8,3,1)+alpha_leave_1(8,2,1) +  alpha_leave_1(8,1,1)
];

% alpha_leave_values = [
%     alpha_leave_1(4,1) + alpha_leave_1(4,2),
%     alpha_leave_1(1,1) + alpha_leave_1(1,2),
%     alpha_leave_1(2,1) + alpha_leave_1(2,2),
%     alpha_leave_1(3,1) + alpha_leave_1(3,2),
%     alpha_leave_1(6,3) + alpha_leave_1(6,2),
%     alpha_leave_1(7,3) + alpha_leave_1(7,2),
%     alpha_leave_1(8,3) + alpha_leave_1(8,2),
%     alpha_leave_1(5,3) + alpha_leave_1(5,2)
% ];

% disp(n_old);
% disp(alpha_enter_1);
% disp(alpha_leave_values);

% disp(sum(alpha_leave_1));
% Update the formula for updated_n
    updated_n = n_old + (alpha_enter_1(:,1) - alpha_leave_values) * T_c; 
    updated_q = q_old + (alpha_arrive_1(:,:,1) - alpha_leave_1(:,:,1)) * T_c;

    % Now updated_q is size [8x3], not 8x3xhorizon
    updated_state.n = updated_n;
    updated_state.q = updated_q;

    % Keep other fields
    updated_state.S = current_state.S;
    updated_state.mu = current_state.mu;
    updated_state.beta = current_state.beta;
    updated_state.alpha_enter_ext_vals = current_state.alpha_enter_ext_vals;
    updated_state.alpha_enter_prev = current_state.alpha_enter_prev;
    updated_state.alpha_leave_prev = current_state.alpha_leave_prev;
end