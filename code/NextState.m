function newConfig = NextState(curConfig, speedVec, deltaT, maxSpeed)
% This is the main function for simulation. Given the speed of arm joints
% and wheel, compute the new config of youBot after deltaT

    speedVec = min(abs(speedVec), maxSpeed) .* sign(speedVec);
    armSpeedVec = speedVec(1:5);
    wheelSpeedVec = speedVec(6:9);
    chassisConfig = curConfig(1:3);
    armConfig = curConfig(4:8);
    wheelConfig = curConfig(9:12);
    newArmConfig = armConfig + deltaT*armSpeedVec;
    newWheelConfig = wheelConfig + deltaT*wheelSpeedVec;
    
    % odometry
    l = 0.235;
    w = 0.15;
    r = 0.0475;
    F = [-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
        1 1 1 1;
        -1 1 -1 1];
    Vb = r/4 * F * wheelSpeedVec * deltaT;
    if Vb(1) == 0
        delta_qb = [0; Vb(2); Vb(3)];
    else
        delta_qb = [Vb(1); 
            (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1); 
            (Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)];
    end
    delta_q = [1 0 0;
        0 cos(chassisConfig(1)) -sin(chassisConfig(1));
        0 sin(chassisConfig(1)) cos(chassisConfig(1))] * delta_qb;
    newChassisConfig = chassisConfig + delta_q;
    newConfig = [newChassisConfig; newArmConfig; newWheelConfig];
end

