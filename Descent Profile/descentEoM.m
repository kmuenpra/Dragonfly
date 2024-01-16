function state = descentEoM(t,y,x)
% Purdue DRAGONfly Mission Profile Eqs of Motion for Optimization
% Alexander Terry 10/16/2022
% Last Updated: 12/1/2022
% Equations of motion for the node descent
% Accepts add. input of x = [Wingspan [m], Chord Length [m], Pitching Altitude [m], Velocity Burn Off Altitude [m], Parachute Deployment Altitude [m], Landing Altitude [m]]

%% Input
% y(1) = Altitude [m]
% y(2) = Vertical Velocity [m/s]
% y(3) = Downrange Distance [m]
% y(4) = Horizontal Velocity [m/s]
% y(5) = Checking variable for pitch maneuver start
% y(6) = Checking variable for pitch maneuver completion [m/s]

l = x(1);                   % Wingspan [m]
c = x(2);                   % Chord Length [m]
pAlt = x(3);                % Pitching Altitude [m]
burnAlt = x(4);             % Velocity Burn Off Altitude [m]
paraAlt = x(5);             % Parachute Deployment Altitude [m]
landAlt = x(6);             % Landing Altitude [m]
velVect = atan(y(2)/y(4));  % Flight Path Angle [rad]

% Atmosphere properties at current altitude
[~,~,~,rho] = atmosisa(y(1)); 

%% Aircraft Properties
% Weight
m = 9;     % Mass (kg)
g = 9.81;   % Gravitational acceleration (m/s^2)

% TO ADD ADDITIONAL AERO ELEMENTS:
% If using standard drag equation, add their surface area and drag
% coefficient then add them to the CdSA computation for all applicable
% stages
% If not using standard drag equation then their acceleration can be added
% in manually for the equations of motion for each stage

% Surface Area Calculations
SAFuse = (pi()*0.1^2)+(4*0.27*0.03);    % Surface area of face of fuselage [m^2]
SAWing = c*l;                           % Planform area of wings [m^2]
SABrake = 4*(0.15*0.14)*sind(90);       % Surface area of deployed airbrakes [m^2]
%SABrake = 0;                            % Surface area of deployed airbrakes [m^2]
SAChute = pi()*(1.7/2)^2;               % Surface area of parachute [m^2]

% Aero Coefficients
CdFuse = 0.106; % Coefficient of drag for fuselage, wind tunnel testing
CdBrake = 1.63; % Coefficient of drag for airbrakes, flat plate
CdChute = 2.2;  % Coefficient of drag for parachute, parachute

AR = l/c;                                           % Aspect Ratio
LoDEst = (0.4*l*c)/(0.02*l*c + CdFuse*SAFuse);      % Estimation of LoD using 2D Clark-Y airfoil properties
aEst = atan(1/LoDEst);                              % Estimation of glide angle [rad]
aoaEst = -aEst - velVect;                           % Estimation of angle of attack
eSpan = 0.76;                                       % Span efficiency factor for wing, PA-28 rectangular wing
ClWing = 2*pi()/(1 + 2/AR)*(aoaEst - -0.06981);     % Coefficient of lift for wing at slight negative aoa, Clark-Y airfoil 
CdWing = 0.02 + ClWing^2/(pi()*eSpan*AR);           % Coefficient of drag for wing, Clark-Y airfoil

%% Compute EOM for Current Flight Stage
if y(1) <= landAlt % Node has landed
    y1Dot = 0;
    y2Dot = 0;
    y3Dot = 0;
    y4Dot = 0;
    y5Dot = 0;
    y6Dot = 0;

elseif y(1) <= paraAlt || (y(1) <= burnAlt && y(4) <= 10) % Landing chute deployed, nose down, airbrake enabled, assume drag acts in both directions
    CdSA = CdFuse*SAFuse + CdBrake*SABrake + CdChute*SAChute; % Sum of individual Cd*SA [m^2]
    
    y1Dot = y(2);
    y2Dot = -g + (0.5*CdSA*rho*y(2)^2)/m;
    y3Dot = y(4);
    if y(4) <= 0
        y4Dot = (0.5*CdSA*rho*y(4)^2)/m;
    else
        y4Dot = -(0.5*CdSA*rho*y(4)^2)/m;
    end    
    y5Dot = 0;
    y6Dot = 0;

elseif y(1) <= burnAlt % Velocity burn off
    aEst = 0;                                           % Estimation of glide angle [rad]
    aoaEst = -aEst - velVect;                           % Estimation of angle of attack
    ClWing = 2*pi()/(1 + 2/AR)*(aoaEst - -0.06981);     % Coefficient of lift for wing at slight negative aoa, Clark-Y airfoil 
    CdSA = CdFuse*SAFuse + CdWing*SAWing;               % Sum of individual Cd*SA [m^2]

    y1Dot = y(2);
    y2Dot = -g + 0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(0) + 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(0);
    y3Dot = y(4);
    y4Dot = 0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(0) - 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(0);
    y5Dot = 0;
    y6Dot = 0;

elseif y(1) > pAlt && y(5) == 0 % High altitude dive, nose down, wings stowed, airbrake enabled
    CdSA = CdFuse*SAFuse + CdBrake*SABrake; % Sum of individual Cd*SA [m^2]
    
    y1Dot = y(2);
    y2Dot = -g + 0.5*CdSA*rho*y(2)^2/m;
    y3Dot = y(4);
    y4Dot = 0;
    y5Dot = 0;
    y6Dot = -g + 0.5*CdSA*rho*y(2)^2/m;

elseif y(6) <= -1 % Pitching maneuver, assume all vertical velocity linearly becomes horizontal, conditional value can be adjusted
    y1Dot = y(2);
    y2Dot = -y(2);
    y3Dot = y(4);
    y4Dot = -y(2);
    y5Dot = 1;
    y6Dot = abs(y(2));

elseif y(2) > 0 % Floating, upward along velocity vector, wings deployed, airbrake disabled
    CdSA = CdFuse*SAFuse + CdWing*SAWing;   % Sum of individual Cd*SA [m^2]
    a = atan(y(2)/y(3));                    % Angle of Attack

    y1Dot = y(2);
    y2Dot = -g + 0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(a) - 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(a);
    y3Dot = y(4);
    y4Dot = -0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(a) - 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(a);
    y5Dot = 0;
    y6Dot = 0;
    
else % Gliding, downward at glide angle, wings deployed, airbrake disabled
    CdSA = CdFuse*SAFuse + CdWing*SAWing;   % Sum of individual Cd*SA [m^2]
    LoD = (ClWing*SAWing) / CdSA;           % Lift to Drag Ratio
    a = atan(1/LoD);                        % Glide angle [rad]

    y1Dot = y(2);
    y2Dot = -g + 0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(a) + 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(a);
    y3Dot = y(4);
    y4Dot = 0.5*ClWing*SAWing*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*sin(a) - 0.5*CdSA*rho*(sqrt(y(4)^2 + y(2)^2))^2/m*cos(a);
    y5Dot = 0;
    y6Dot = 0;
end

% Return values
state = [y1Dot; y2Dot; y3Dot; y4Dot; y5Dot; y6Dot];
end