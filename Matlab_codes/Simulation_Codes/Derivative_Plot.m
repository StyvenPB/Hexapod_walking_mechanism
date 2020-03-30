function Derivative_Plot(omega2, theta2, pX, pY)
    N = length(theta2);         % length of position vector
    xPdot = zeros(N,1);         % estimate of derivative vector x
    yPdot = zeros(N,1);         % estimate of derivative vector y
    dt = 2*pi/((N-1)*omega2);   % time increment between calculations
    
    % Forward derivative estimation
   for i = 1:N-1
        xPdot(i) = (pX(i+1) - pX(i))/dt;
        yPdot(i) = (pY(i+1) - pY(i))/dt; 
        % añadiendo 
   end
   
  % Aceleration 
   for i = 1:N-1
        xPdotdot(i) = (xPdot(i+1) - xPdot(i))/dt;
        yPdotdot(i) = (yPdot(i+1) - yPdot(i))/dt; 
        % añadiendo 
   end
    
    % Assume last derivative is the same as the first data
    xPdot(N) = xPdot(1);
    yPdot(N) = yPdot(1);
    
    % Assume last and the second last are the same as the first data 
    xPdotdot(N)=xPdotdot(1);
    yPdotdot(N)=yPdotdot(1);
    xPdotdot(N-1)=xPdotdot(1);
    yPdotdot(N-1)=yPdotdot(1);
        
    % Plot: derivative vs solving method 
    plot(theta2*180/pi, xPdot, 'k.'), hold on;
    plot(theta2*180/pi, yPdot, 'b.'),     
    legend('Vx estimated','Vy estimated'), grid on, grid minor;  title('Velocity Anlysis');  ylabel('Velocity (mm/s)'), xlabel('Crank angle (°)');
    set(gca,'xtick',0:60:360); xlim([0 360]);    
    figure,
    plot(theta2*180/pi, xPdotdot, 'k.'), hold on;
    plot(theta2*180/pi, yPdotdot, 'b.'), 
    legend('Ax estimated','Ay estimated'), grid on, grid minor;  title('Aceleration Anlysis');  ylabel('Aceleration (mm/s)'), xlabel('Crank angle (°)');
    set(gca,'xtick',0:60:360); xlim([0 360]);  
end