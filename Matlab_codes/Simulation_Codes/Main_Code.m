%%  KLANN MECHANISM
clear; close all; clc;

% LINKAGE DIMENSIONS
a = 13.21;                     % Crank 1(mm)
b = 29.92;                     % Coupler 2 (mm)
c = 19.8;                       % Rocker 3 (mm)
d = 30.997;                   % length between ground pins (mm)
p = 58.261;                   % Coupler ext 2 (mm)
u = 25.55;                    % Length of link 4 (mm)
v = 26.03;                    % Length of link 5 (mm)
mm = 66.01;                % Length of link 5 ext(mm)
phi = 26.83*pi/180;
gamma = 6.7*pi/180;      % Internal angle of coupler 2
alfa2 = 26.79*pi/180;       % Internal angle of link 5

% GROUND PINES
x0 = [ 0; 0]; % ground pin at A (origin)
a0 = [ 0; 0]; % acceleration of origin

% ACELERATION AND VELOCITY CRANK
omega2 = 20;     % angular velocity of crank (rad/sec)
alpha2 = 0;     % angular acceleration of crank (rad/sec^2)

% SAPACE FOR VARIABLES
N = 361;                                                         % number of times to perform position calculations
[xB,xC,xD,xE,xG,xF,xH,xM] = deal(zeros(2,N));  % position of B, C, E, F, G
[vB,vC,vE] = deal(zeros(2,N));                          % allocate space for vel of B, C, P
[theta2,theta3,theta4,theta5,theta6] = deal(zeros(1,N)); % angles

% MAIN LOOP
for i = 1:N
    % LOWER FOURBAR
    theta2(i) = (i-1)*pi/180+phi;   % crank angle
    Theta2(i) = (i-1)*pi/180;      
    r = d - a*cos(theta2(i));
    s = a*sin(theta2(i));
    f2 = r^2 + s^2;
    delta = acos((b^2+c^2-f2)/(2*b*c));
    g = b - c*cos(delta);
    h = c*sin(delta);
    theta3(i) = atan2((h*r - g*s),(g*r + h*s)); % coupler angle
    theta4(i) = theta3(i) + delta;              % rocker angle
    
    [e2,n2] = UnitVector(theta2(i)-phi);
    [e3,n3] = UnitVector(theta3(i)-phi);
    [eBE,nBE] = UnitVector(gamma+theta3(i)-phi);
    [eAF,nAF] = UnitVector(atan2(9.51,27.66));
    [eAD,nAD] = UnitVector(atan2(-13.99,27.66));
       
    xB(:,i) = FindPos(x0,a, e2);
    xD(:,i) = FindPos(x0,30.997, eAD);
    xC(:,i) = FindPos(xB(:,i),b, e3);
    xE(:,i) = FindPos(xB(:,i),p, eBE);
    xF(:,i) = FindPos(x0,29.249, eAF);
    
    % UPPER FOURBAR
    xFC = xF(1,i) - xC(1,i); yFC = xF(2,i) - xC(2,i);
    xEC = xE(1,i) - xC(1,i); yEC = xE(2,i) - xC(2,i); 
    beta = atan2(yFC, xFC);
    alpha = atan2(yEC, xEC);
    aPrime = sqrt(xEC^2 + yEC^2);   % virtual crank length on upper fourbar
    dPrime = sqrt(xFC^2 + yFC^2);   % virtual ground length on upper fourbar
    theta2Prime = alpha - beta;     % virtual crank angle on upper fourbar
    r = dPrime - aPrime*cos(theta2Prime);
    s = aPrime*sin(theta2Prime);
    f2 = r^2 + s^2;
    delta = acos((u^2+v^2-f2)/(2*u*v));
    g = u - v*cos(delta);
    h = v*sin(delta);
    theta5Prime = atan2((h*r - g*s),(g*r + h*s)); % coupler and rocker
    theta6Prime = theta5Prime + delta;            % angles on upper fourbar
    theta5(i) = theta5Prime + beta - pi;          % return angles to fixed
    theta6(i) = theta6Prime + beta - pi;          % fixed CS
    % Calculate remaining unit vectors
    [e5,n5] = UnitVector(theta5(i));
    [e6,n6] = UnitVector(theta6(i));
    % Calculate position of point G
    xG(:,i) = FindPos(xF(:,i), u, e5);
    
    [e7,n7] = UnitVector(theta6(i)-pi-alfa2);   
    xH(:,i) = FindPos(xE(:,i), mm, e7);
  
    
    
    % LINKS PLOTS
    figure(1),
    plot([0 xD(1,i)],[0 xD(2,i)],'linewidth',3 ,'color',[70/255 70/255 70/255]); hold on;
    plot([0 xF(1,i)],[0 xF(2,i)],'linewidth',3,'color',[70/255 70/255 70/255])  
    plot([xD(1,i) xF(1,i)],[xD(2,i) xF(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]) 
    plot([0 xB(1,i)],[0 xB(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xB(1,i) xC(1,i)],[xB(2,i) xC(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xD(1,i) xC(1,i)],[xD(2,i) xC(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xC(1,i) xE(1,i)],[xC(2,i) xE(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xF(1,i) xG(1,i)],[xF(2,i) xG(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xG(1,i) xE(1,i)],[xG(2,i) xE(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
    plot([xE(1,i) xH(1,i)],[xE(2,i) xH(2,i)],'linewidth',3,'color',[70/255 70/255 70/255]);
       
    % POINTS LABLES
    text( x0(1), x0(2) + 5,'A','HorizontalAlignment','center');
    text( xB(1,i), xB(2,i) + 5,'B','HorizontalAlignment','center');
    text( xC(1,i), xC(2,i) + 5,'C','HorizontalAlignment','center');
    text( xE(1,i), xE(2,i) + 5,'E','HorizontalAlignment','center');
    text( xD(1,i), xD(2,i) - 5,'D','HorizontalAlignment','center');
    text( xF(1,i), xF(2,i) + 5,'F','HorizontalAlignment','center');
    text( xG(1,i), xG(2,i) + 5,'G','HorizontalAlignment','center');
    text( xH(1,i)-5, xH(2,i) + 5,'H','HorizontalAlignment','center');
       
    % LINKS PLOTS
    plot([x0(1) xB(1,i) xC(1,i) xD(1,i) xE(1,i) xF(1,i) xG(1,i) xH(1,i)], ...
        [x0(2) xB(2,i) xC(2,i) xD(2,i) xE(2,i) xF(2,i) xG(2,i) xH(2,i)], ...
        'ko','MarkerSize',5,'MarkerFaceColor','k');
    plot(xH(1,:),xH(2,:),'r.');
    title('Foot Trayectory');
    xlabel('X position (mm)'); grid on; grid minor; ylabel('Y position (mm)')
    xlim([-20 120]); ylim([-80 60]); hold off; 
    pause(0.001) 
end

% VELOCITY (DERIVATIVE METHOD)
figure; title('Foot Velocity');
Derivative_Plot(omega2, Theta2, xH(1,:), xH(2,:));

% Save data from the simulation
    [xdH, ydH]=FindVel2(omega2, Theta2, xH(1,:), xH(2,:));
     xdH=xdH';
      ydH=ydH';
      
 % Load the experimental data.
      load('velX.mat');
      load('velY.mat');
 
 %  Plot the comparation between the real and simulation one data
 figure,  plot(theta2*180/pi-26.83, xdH, 'r.'), hold on;
plot(theta2*180/pi-26.83, velX, 'k.'); grid on, grid minor;  title('Velocity Anlysis');  ylabel('Velocity (mm/s)'), xlabel('Crank angle (°)'); legend('Vx estimated','Vx mesuared')
set(gca,'xtick',0:60:360); xlim([0 360]);    

figure,  plot(theta2*180/pi-26.83, ydH, 'r.'), hold on;
plot(theta2*180/pi-26.83, velY, 'k.'); grid on, grid minor;  title('Velocity Anlysis');  ylabel('Velocity (mm/s)'), xlabel('Crank angle (°)'); legend('Vy estimated','Vy mesuared')
set(gca,'xtick',0:60:360); xlim([0 360]);    