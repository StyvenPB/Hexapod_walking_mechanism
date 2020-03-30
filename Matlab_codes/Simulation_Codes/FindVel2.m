function [xPdot, yPdot]=FindVel2(omega2, theta2, pX, pY)
    N = length(theta2);         % length of position vector
    xPdot = zeros(N,1);         % estimate of derivative vector x
    yPdot = zeros(N,1);         % estimate of derivative vector y
    dt = 2*pi/((N-1)*omega2);   % time increment between calculations
    
    % Forward derivative estimation
    for i = 1:N-1
        xPdot(i) = (pX(i+1) - pX(i))/dt;
        yPdot(i) = (pY(i+1) - pY(i))/dt;
        
    end
    
    % Assume final derivative is the same as the first
    xPdot(N) = xPdot(1);
    yPdot(N) = yPdot(1);
    
   
end
