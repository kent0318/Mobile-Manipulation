function trajectory = TrajectoryGenerator(initial_T_se, initial_T_sc, final_T_sc, grasp_T_ce, standoff_T_ce, k)
% This function generate the reference trajectory for the end-effector
% frame {e}. Please see description of miletone 2 on
% http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone
% for more detail

    % liner and angular speed limit of end-effector
    v_linear = 0.15;
    v_angular = pi/4;

    % trajectory 1
    T1 = initial_T_se;
    T2 = initial_T_sc * standoff_T_ce;
    trajectory = TrajectoryHelper(T1, T2, v_linear, v_angular, 0, k);

    % trajectory 2
    T1 = initial_T_sc * standoff_T_ce;
    T2 = initial_T_sc * grasp_T_ce;
    trajectory = [trajectory; TrajectoryHelper(T1, T2, v_linear, v_angular, 0, k)];

    % trajectory 3
    trajectory = [trajectory; repmat([trajectory(end, 1:end-1) 1], 63 * k, 1)];

    % trajectory 4
    T1 = initial_T_sc * grasp_T_ce;
    T2 = initial_T_sc * standoff_T_ce;
    trajectory = [trajectory; TrajectoryHelper(T1, T2, v_linear, v_angular, 1, k)];

    % trajectory 5
    T1 = initial_T_sc * standoff_T_ce;
    T2 = final_T_sc * standoff_T_ce;
    trajectory = [trajectory; TrajectoryHelper(T1, T2, v_linear, v_angular, 1, k)];

    % trajectory 6
    T1 = final_T_sc * standoff_T_ce;
    T2 = final_T_sc * grasp_T_ce;
    trajectory = [trajectory; TrajectoryHelper(T1, T2, v_linear, v_angular, 1, k)];

    % trajectory 7
    trajectory = [trajectory; repmat([trajectory(end, 1:end-1) 0], 63 * k, 1)];

    % trajectory 8
    T1 = final_T_sc * grasp_T_ce;
    T2 = final_T_sc * standoff_T_ce;
    trajectory = [trajectory; TrajectoryHelper(T1, T2, v_linear, v_angular, 0, k)];
end

