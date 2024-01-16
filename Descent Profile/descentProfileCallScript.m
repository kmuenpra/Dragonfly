% Purdue DRAGONfly Mission Profile Calling Function
% Alexander Terry 10/16/2022
% Last Updated: 12/7/2022
% Integrates the equations of motion for the node descent

clear all
close all

%% Initialization
alt0 = 36.5*10^3;   % Drop altitude of 120,000 ft (m)
altF = 1225;        % Landing altitude at Ft Sumner (m)
relBurnAlt = 750;   % Altitude above landing to begin velocity burn off (m)
relParaAlt = 500;   % Altitude above landing to deploy parachute if not already triggered (m)

t0 = 0; % Initial time (s)
tf = 3600; % Simulation end time (s)
state0 = [alt0, 0, 0, 0, 0, 0]; % Initial state values (altitude, vert vel, downrange, horz vel, pitch start check, pitch finish check)

% x = [Wingspan [m], Chord Length [m], Pitching Altitude [m], Velocity Burn Off Altitude [m], Parachute Deployment Altitude [m]]
% Recommended from optimization: x = [1.1479;0.19961;9093.4;relBurnAlt+altF;relParaAlt+altF;altF];
x = [1.1479;0.19961;9093.4;relBurnAlt+altF;relParaAlt+altF;altF];

%% Calculations
% Integration Runge-Kutta-45: order 4 degree 5
[T, Y] = ode45(@(t,y) descentEoM(t,y,x), [t0 tf], state0); % THE Y HERE IS THE FULL POSITION/VEL OUTPUT ARRAY

% Mach Number Calculations
mach = zeros(length(Y),1);
for i = 1:1:(length(Y))
    [~, a, ~, rho] = atmosisa(Y(i,1));
    mach(i) = abs(sqrt(Y(i,2)^2 + Y(i,4)^2) / a);
end

% Return Values
endInd = find(Y(:,1) >= altF, 1, "last");   % Index when node lands
tf = T(endInd);                             % Time when node lands [s]
R = Y(endInd,3);                            % Final downrange distance [m]
mMax = max(mach(1:endInd));                 % Maximum velocity [mach]

pitchBegInd = find(Y(:,5) == 0, 1, "last");         % Index when node begins pitching
vPitch = abs(Y(pitchBegInd,2));                     % Downward velocity when node begins pitching [m/s]
[~, ~, ~, rhoPitch] = atmosisa(Y(pitchBegInd,1));   % Air density when node begins pitching

pitchEndInd = find(diff(Y(:,5)) > 0, 1, "last") + 1;    % Index when node finishes pitching
mMaxGlide = max(mach(pitchEndInd:endInd));              % Maximum velocity after pitching [mach]

% Drag Computation for Dive
for i = 1:pitchEndInd
    CdFuse = 0.106; % Coefficient of drag for fuselage, wind tunnel testing
    CdBrake = 1.63; % Coefficient of drag for airbrakes, flat plate
    SAFuse = (pi()*0.1^2)+(4*0.27*0.03);    % Surface area of face of fuselage [m^2]
    % Full Node
    %SABrake = 4*(0.15*0.14)*sind(90);       % Surface area of deployed airbrakes [m^2]
    %CdSA = CdFuse*SAFuse + CdBrake*SABrake; % Sum of individual Cd*SA [m^2]
    % Single Airbrake
    SABrake = (0.15*0.14)*sind(90);       % Surface area of deployed airbrakes [m^2]
    CdSA = CdFuse*SAFuse + CdBrake*SABrake; % Sum of individual Cd*SA [m^2]

    [~,~,~,rho] = atmosisa(Y(i,1)); % Atmosphere properties at current altitude

    L(i) = 0;
    D(i) = 0.5*CdSA*rho*(sqrt(Y(i,2)^2 + Y(i,4)^2))^2;
end

% Drag and Lift Computations for Glide
for i = pitchEndInd:endInd
    l = x(1);                   % Wingspan [m]
    c = x(2);                   % Chord Length [m]
    velVect = atan(Y(i,2)/Y(i,4));  % Flight Path Angle [rad]
    [~,~,~,rho] = atmosisa(Y(i,1)); % Atmosphere properties at current altitude
    g = 9.81;   % Gravitational acceleration (m/s^2)

    SAWing = c*l;                                       % Planform area of wings [m^2]
    SAFuse = 0.0625;                                    % Surface area of face of fuselage [m^2]
    CdFuse = 0.106;                                     % Coefficient of drag for fuselage, wind tunnel testing
    AR = l/c;                                           % Aspect Ratio
    LoDEst = (0.4*l*c)/(0.02*l*c + CdFuse*SAFuse);      % Estimation of LoD using 2D Clark-Y airfoil properties
    aEst = atan(1/LoDEst);                              % Estimation of glide angle [rad]
    aoaEst = -aEst - velVect;                           % Estimation of angle of attack
    eSpan = 0.76;                                       % Span efficiency factor for wing, PA-28 rectangular wing
    ClWing = 2*pi()/(1 + 2/AR)*(aoaEst - -0.06981);     % Coefficient of lift for wing at slight negative aoa, Clark-Y airfoil 
    CdWing = 0.02 + ClWing^2/(pi()*eSpan*AR);           % Coefficient of drag for wing, Clark-Y airfoil
    
    L(i) = 0.5*ClWing*SAWing*rho*(sqrt(Y(i,2)^2 + Y(i,4)^2))^2;
    D(i) = 0.5*CdWing*SAWing*rho*(sqrt(Y(i,2)^2 + Y(i,4)^2))^2;
end

%% Plot
% Velocity across altitude
figure(1)
subplot(2,3,1)
plot(Y(:,2), Y(:,1)/1000)
title({'Vertical Velocity', 'along Altitude Descent'})
ylabel('Altitude (km)'); xlabel('Vertical Velocity (m/s)'); grid on
axis([-200 0 0 37])
yticks(0:2:37)

subplot(2,3,2)
plot(Y(:,4), Y(:,1)/1000)
title({'Horizontal Velocity', 'along Altitude Descent'})
ylabel('Altitude (km)'); xlabel('Horizontal Velocity (m/s)'); grid on
axis([0 100 0 37])
yticks(0:2:37)

% Altitude over time (see what time the object hits the ground)
subplot(2,3,3)
plot(T, Y(:,1)/1000)
title('Altitude vs. Time')
ylabel('Altitude (km)'); xlabel('Time (s)'); grid on
axis([0 1500 0 37])
yticks(0:2:37)

% Mach across altitude
subplot(2,3,4)
plot(mach, Y(:,1)/1000)
title('Mach along Altitude Descent')
ylabel('Altitude (km)'); xlabel('Mach'); grid on
axis([0 1 0 37])
yticks(0:2:37)

% Range over altitude
subplot(2,3,5)
plot(Y(:,3)/1000, Y(:,1)/1000)
title('Downrange Distance vs Altitude')
xlabel('Downrange Distance (km)'); ylabel('Altitude (km)'); grid on
axis([0 65 0 37])
yticks(0:2:37)

sgtitle('Run 4 Flight Performance')

% Lift over altitude
figure(2)
plot(L(pitchEndInd:endInd), Y(pitchEndInd:endInd,1)/1000)
title('Lift over Altitude')
ylabel('Altitude (km)'); xlabel('Lift (N)'); grid on

% Drag over altitude
figure(3)
plot(D(1:endInd), Y(1:endInd,1)/1000)
title('Drag over Altitude')
ylabel('Altitude (km)'); xlabel('Drag (N)'); grid on

%end