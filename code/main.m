% The main script that generates animation csv file with user specified
% configuration of cube and youbot. 
% RUN THIS SCRIPT DIRECTLY TO GET RESULTS or change parameters below and
% run.

% enter the initial config of the cube (x, y, phi)
cube_init_config = [1 0 0]';

% enter the final config of the cube (x, y, phi)
cube_fin_config = [0 -1 -pi/2]';

% enter the actual initial config of the youbot (chassis phi, chassis x,
% chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state)
% You can randomly generate other initial actual config which deviates properly from
% the initial reference config using generate_youbot_actual_config.m and
% substitute the first 8 numbers
youbot_actual_config = [0.1595   -0.5696   -0.1921    0.0705   -0.4105   -1.0327   -1.1359   -0.0507 ...
    0 0 0 0 0];

% enter the reference initial config of the youbot (chassis phi, chassis x,
% chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state)
youbot_ref_config = [0 -0.5 0 0 0.0532 -0.7974 -0.8266 0 0 0 0 0 0]; % this config create a Tse as specified in the Final Step

% enter feedback gains. 
Kp = eye(6)*1.5;
Ki = zeros(6);

% enter the number of trajectory reference configurations per 0.01 seconds
k = 1;

% enter the max speed of arm joints and wheels
arm_maxSpeed = ones(5,1)*pi*2/3;
wheel_maxSpeed = ones(4,1)*pi*10;


% PLEASE DO NOT MODIFY ANY CODE STARTING FROM HERE!!!!
% convert configs to column vectors
cube_init_config = cube_init_config(:);
cube_fin_config = cube_fin_config(:);
youbot_actual_config = youbot_actual_config(:);
youbot_ref_config = youbot_ref_config(:);

% Initialize constants related to youbot and calculate reference
% end-effector configuration
phi = youbot_ref_config(1);
x = youbot_ref_config(2);
y = youbot_ref_config(3);
Tsb_r = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [[0 0 1 0 0.033 0];
        [0 -1 0 -0.5076 0 0];
        [0 -1 0 -0.3526 0 0];
        [0 -1 0 -0.2176 0 0];
        [0 0 1 0 0 0]]';
T0e_r = FKinBody(M0e, Blist, youbot_ref_config(4:8));
Tse_r = Tsb_r * Tb0 * T0e_r;

% Calculate cube configuration and specify grasping & standoff configuration
x = cube_init_config(1);
y = cube_init_config(2);
phi = cube_init_config(3);
Tsc_init = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.025; 0 0 0 1];
x = cube_fin_config(1);
y = cube_fin_config(2);
phi = cube_fin_config(3);
Tsc_fin = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.025; 0 0 0 1];
r_grasp = [-1/2 0 -sqrt(3)/2; 0 1 0; sqrt(3)/2 0 -1/2]';
Tce_grasp = [r_grasp [0.011;0;-0.005]; [0 0 0 1]]; % make the origin slightly off the center to increase contact area
Tce_standoff = [r_grasp [0.011;0;0.08]; [0 0 0 1]];

% Constant related to wheels
l = 0.235;
w = 0.15;
r = 0.0475;
F = [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
    1 1 1 1;
    -1 1 -1 1]*r/4;
F6 = [zeros(1,4);zeros(1,4);F;zeros(1,4)];

% generate reference trajectory
disp("Generating reference trajectory");
trajectory = TrajectoryGenerator(Tse_r, Tsc_init, Tsc_fin, Tce_grasp, Tce_standoff, k);

curConfig = youbot_actual_config;
configVec = curConfig;
XerrVec = zeros(6, 1);
maxSpeed = [arm_maxSpeed(:); wheel_maxSpeed(:)];
disp("Entering control loop");
% control loop
for N = 1:length(trajectory)-1
    phi = curConfig(1);
    x = curConfig(2);
    y = curConfig(3);
    Tsb = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
    Tbe = Tb0 * FKinBody(M0e, Blist, curConfig(4:8));
    Tse = Tsb * Tbe;
    Tsed = [reshape(trajectory(N, 1:9), 3, 3)' trajectory(N, 10:12)'; [0 0 0 1]];
    Tsed_next = [reshape(trajectory(N+1, 1:9), 3, 3)' trajectory(N+1, 10:12)'; [0 0 0 1]];
    [V, Xerr] = FeedbackControl(Tse, Tsed, Tsed_next, Kp, Ki, 0.01/k);
    J_a = JacobianBody(Blist, curConfig(4:8));
    J_b = Adjoint(TransInv(Tbe)) * F6;
    J_e = [J_b J_a];
    speedVec = pinv(J_e, 0.001) * V;
    
    curConfig = [NextState(curConfig(1:end-1), [speedVec(5:9); speedVec(1:4)], 0.01/k, maxSpeed); 
        trajectory(N+1, end)];
    
    % Store config and error twist every k times
    if mod(N, k) == 0
        configVec = [configVec curConfig];
        XerrVec = [XerrVec Xerr];
    end
end
XerrVec = XerrVec(:, 2:end);
disp("Generating animation csv file");
csvwrite("youbot_config.csv", configVec');
disp("Writing error plot data");
csvwrite("err_twist.csv", XerrVec');
disp("Generating error plot");
X = (1:length(XerrVec)) * 0.01/k;
plot(X, XerrVec(1, :), X, XerrVec(2, :), X, XerrVec(3, :), X, XerrVec(4, :), X, XerrVec(5, :), X, XerrVec(6, :));
xlabel("time (s)");
ylabel("error");
disp("Done. Please clear workspace if you want to re-run the program");