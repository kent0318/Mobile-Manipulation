function trajectory = TrajectoryHelper(T1, T2, v_linear, v_angular, grasp, k)
% This function generate one segment of trajectory given starting
% and ending configuration of the end-effector
T = TransInv(T1) * T2;
[R,p] = TransToRp(T);
t_linear = norm(p)/v_linear;
t_angular = norm(so3ToVec(MatrixLog3(R))) / v_angular;
t = max(t_linear, t_angular);
t = round(t*100) / 100; % making the duration of this trajectory a integer multiple of 0.01s
traj = ScrewTrajectory(T1, T2, t, round(t*k/0.01), 5);
trajectory = zeros(length(traj), 13);
for i=1:length(traj)
    [R,p] = TransToRp(traj{i});
    trajectory(i,:) = [reshape(R', [1,9]) p' grasp];
end
end

