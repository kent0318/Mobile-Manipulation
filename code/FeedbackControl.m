function [V, Xerr] = FeedbackControl(Tse, Tsed, Tsed_next, Kp, Ki, deltaT)
% Controller with feedforward control and feedback PI control feature

    persistent errIntegral
    if isempty(errIntegral)
        errIntegral = zeros(6,1);
    end

    err_X = TransInv(Tse) * Tsed;
    Vd = se3ToVec(MatrixLog6(TransInv(Tsed)*Tsed_next)/deltaT);
    Xerr = se3ToVec(MatrixLog6(err_X));
    V = Adjoint(err_X)*Vd + Kp*Xerr + Ki*errIntegral;
    errIntegral = errIntegral + Xerr*deltaT;
    
    % For cases when Ki is nonzero, if the error twist does not converge and
    % display chaotic pattern, it might be the internal computational
    % error from matlab. Uncomment this line of code below and re-run the
    % program. If it solved the problem, comment it back.
    
    % disp(norm(errIntegral));
end