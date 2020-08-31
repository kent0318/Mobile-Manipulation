% This script generates a random actual config of youBot that deviates properly from
% the reference config. The deviation on the end-effector configuration is
% between 30 to 60 degree on orientation and 0.2m to 0.4m on position. 
% The config here does not include wheel angle or gripper state, so it has
% dimension of 8 instead of 13. 

% Enter the reference config
ref = [0 -0.5 0 0 0.0532 -0.7974 -0.8266 0]';

% PLEASE DO NOT MODIFY ANY CODE STARTING FROM HERE
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e = [1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];
Blist = [[0 0 1 0 0.033 0];
        [0 -1 0 -0.5076 0 0];
        [0 -1 0 -0.3526 0 0];
        [0 -1 0 -0.2176 0 0];
        [0 0 1 0 0 0]]';
phi = ref(1);
x = ref(2);
y = ref(3);
Tsb_ref = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
Tse_ref = Tsb_ref * Tb0 * FKinBody(M0e, Blist, ref(4:8));
for i=1:10000
    offset = rand(8,1) - 0.5;
    offset(2) = offset(2) * 0.4;
    offset(3) = offset(3) * 0.4;
    actual = ref+offset;
    phi = actual(1);
    x = actual(2);
    y = actual(3);
    Tsb_actual = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
    Tse_actual = Tsb_actual * Tb0 * FKinBody(M0e, Blist, actual(4:8));
    [R,p] = TransToRp(TransInv(Tse_ref)*Tse_actual);
    if norm(p) > 0.2 && norm(so3ToVec(MatrixLog3(R))) > pi/6 && ...
       norm(p) < 0.4 && norm(so3ToVec(MatrixLog3(R))) < pi/3
        break
    end
    if i == 10000
        disp("You're really unlucky. Please run the script again");
    end
end
disp(actual');