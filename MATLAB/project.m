%% Loading the Franka Emika Panda model

mdl_panda; % Loading the model from the Peter Corke toolbox

%% Initial configuration

% Initial joint configuration
q0 = [0, 0, 0, -pi/2, 0, pi/2, pi/4];
q = q0;

% Forward kinematics at q0
T0 = panda.fkine(q);
eePos_start = T0.t;       % Starting end-effector position
R_des = T0.R;             % Desired (constant) end-effector orientation

%fprintf('Position of the end-effector: [%f, %f, %f]\n', ...
    %eePos_start(1), eePos_start(2), eePos_start(3))

J = panda.jacob0(q);
%fprintf('Size of the Jacobian: [%d, %d]\n', size(J,1), size(J,2))
%fprintf('Rank of the Jacobian: %d\n', rank(J))

%panda.plot(q)

%% Parameters

Ts     = 1e-3;      % Sampling time
T_end  = 1;         % Total trajectory duration
omega  = 2*pi;      % Angular speed (1 full circle in 1 sec)
r      = 0.4;       % Circle radius
K_p    = 10*eye(3); % Proportional gain for position
K_o    = 10*eye(3); % Proportional gain for orientation
lambda = 1e-2;      % Damped Least Squares damping factor
k0     = 500;       % Gain for joint limit avoidance
k1     = 250;      % Gain for manipulability maximization
delta  = 1e-3;      % Perturbation step for numerical gradient

t = 0:Ts:T_end;
N_steps = length(t);

%% Desired circular trajectory

xd = eePos_start(1) + r*cos(omega*t) - r;
yd = eePos_start(2) + r*sin(omega*t);
zd = ones(size(t)) * eePos_start(3);

xd_dot = -r*omega*sin(omega*t);
yd_dot =  r*omega*cos(omega*t);
zd_dot = zeros(size(t));

%% Joint limits and null-space references

q_min = panda.qlim(:,1);
q_max = panda.qlim(:,2);
q_mid   = ((q_max + q_min) / 2)';
q_range = (q_max - q_min)';

%% Storage allocation

eePoshis  = zeros(3, N_steps);
Qhis      = zeros(N_steps, 7);
manipHis  = zeros(1, N_steps);
trackErr  = zeros(3, N_steps);
rankHis   = zeros(1, N_steps);
QdotHis = zeros(N_steps, 7);

%% Mode selection

mode = input('Choose mode (0=CLIK, 1=Joint limits avoidance, 2=Manipulability, 3=Both): ');

%% Control loop

for k = 1:N_steps
    % Forward kinematics
    T_curr = panda.fkine(q);
    eeP    = T_curr.t;
    R_curr = T_curr.R; 

    % Desired position and velocity at this step
    pos_des = [xd(k); yd(k); zd(k)];
    v_des   = [xd_dot(k); yd_dot(k); zd_dot(k)];

    % Position and orientation errors
    err_p = pos_des - eeP;
    Re    = R_des * R_curr';
    err_o = 0.5 * [Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)];

    % Jacobian and manipulability monitoring
    J = panda.jacob0(q);
    manipHis(k) = sqrt(det(J*J'));
    rankHis(k)  = rank(J);

    % CLIK control velocity (position + orientation)
    v_control = [v_des + K_p * err_p; K_o * err_o];

    % Damped Least Squares pseudoinverse
    pseudoInv = J' / (J*J' + lambda^2 * eye(6));
    
    % Secondary tasks
    null_task = zeros(7,1);
        if (mode == 1 || mode == 3)
            % Null-space task 1: joint limit avoidance
            grad_jointLimit = -(q - q_mid) ./ (q_range .^ 2);
            null_task = null_task + k0 * grad_jointLimit';
        end
        if (mode == 2 || mode == 3)
            % Null-space task 2: manipulability gradient (numerical derivative)
            grad_manip = zeros(7,1); 
            for i = 1:7
                q_plus = q; q_plus(i) = q_plus(i) + delta;
                q_minus = q; q_minus(i) = q_minus(i) - delta;
                J_plus  = panda.jacob0(q_plus);
                J_minus = panda.jacob0(q_minus);
                w_plus  = sqrt(det(J_plus*J_plus'));
                w_minus = sqrt(det(J_minus*J_minus'));
                grad_manip(i) = (w_plus - w_minus) / (2*delta);
            end
            null_task = null_task + k1 * grad_manip;
         end
        

    % Null-space projector
    N = eye(7) - pseudoInv * J;

    % Combined controller with null-space tasks
    q_dot = pseudoInv * v_control + N * null_task;
    
    % Euler integration
    q = q + Ts*q_dot';

    % Save data
    QdotHis(k,:) = q_dot';
    Qhis(k,:)      = q;
    eePoshis(:,k)  = eeP;
    trackErr(:,k)  = err_p;
end

%% Plotting

figure('Name', '3D Trajectory');
plot3(eePoshis(1,:), eePoshis(2,:), eePoshis(3,:), 'b', 'LineWidth', 2)
hold on
plot3(xd, yd, zd, 'r--', 'LineWidth', 1);
grid on; axis equal;
legend('Robot trajectory', 'Desired trajectory');
title('Tracking of a circle');

figure('Name', 'Joint positions');
colors = lines(7);
hold on;
for i = 1:7
    plot(t, Qhis(:,i), 'Color', colors(i,:));
end
for i = 1:7
    yline(panda.qlim(i,1), '--', 'Color', colors(i,:));
    yline(panda.qlim(i,2), '--', 'Color', colors(i,:), 'LineWidth', 0.8);
end
xlabel('Time (s)');
ylabel('Joint position (rad)');
title('Joint positions over time');
legend('q1','q2','q3','q4','q5','q6','q7')
grid on;
hold off;

figure('Name', 'Tracking Error');
% Computing the norm of the error vector
err_norm = vecnorm(trackErr);
plot(t, err_norm)
xlabel('Time (s)');
ylabel('Error (m)');
title('Tracking error');
grid on;

figure('Name', 'Manipulability');
plot(t, manipHis);
xlabel('Time (s)');
ylabel('w(q) = sqrt(det(JJ^T))');
title('Manipulability over time');
grid on;

%% Printing metrics

fprintf('Max tracking error:         %.4f m\n', max(err_norm));
fprintf('RMS tracking error:         %.4f m\n', rms(err_norm));
fprintf('Max dist from joint center: %.4f rad\n', max(max(abs(Qhis - q_mid))));
fprintf('Mean dist from joint center:%.4f rad\n', mean(mean(abs(Qhis - q_mid))));
fprintf('Max manipulability:         %.4e\n', max(manipHis));
fprintf('Mean manipulability:        %.4e\n', mean(manipHis));
violations = sum(sum(Qhis < q_min' | Qhis > q_max'));
fprintf('Joint limit violations:     %d / %d (%.1f%%)\n', violations, N_steps, violations/N_steps*100);
fprintf('Max joint velocity:         %.4f rad/s\n', max(max(abs(QdotHis))));