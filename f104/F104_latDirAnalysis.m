%% Lateral-Directional Handling Qualities Design for the Lockheed F-104 Starfighter Aircraft
%
% Source: "Flight Dynamics Principles" by Michael V. Cook, Assignment 3.

%% Aerodynamic Data
alt = [0 0 0]; % Altitude (ft)
airDensity = [0.00238 0.00238 0.00238]; % (slug/ft^3)
speedOfSound = [1116.44 1116.44 1116.44]; % (ft/s)
g = [32.2 32.2 32.2]; % Gravitational constant (ft/s^2)
trimMach = [0.257 0.8 1.1];
trimPitchAttitude = [2.30 2 1]; % (°)
tas = trimMach.*speedOfSound; % True airspeed (ft/s)

yV =    [-0.178     -0.452      -0.791]; % (1/s)
lSs =   [-20.9      -146.0      -363.0]; % (1/s^2)
nSs =   [ 2.68       13.60       42.70]; % (1/s^2)
lRollRate = [-1.38      -4.64       -7.12]; % (1/s)
nRollRate = [-0.0993    -0.188      -0.341]; % (1/s)
lYawRate =  [ 1.16       3.67        7.17]; % (1/s)
nYawRate =  [-0.157     -0.498      -1.06]; % (1/s)
yAil =  [ 0          0           0]; % (1/s)
lAil =  [ 4.76       49.6        81.5]; % (1/s^2)
nAil =  [ 0.266      3.510       6.500]; % (1/s^2)
yRud =  [ 0.0317     0.0719      0.0621]; % (1/s)
lRud =  [ 5.35       41.5        57.6]; % (1/s^2)
nRud =  [-0.923     -7.070      -8.720]; % (1/s^2)

%% Open Loop A/C Model
acMdlz = {};
for fCondIdx = 1:numel(alt)
    A = [
        yV(fCondIdx)    sin(trimPitchAttitude(fCondIdx))    -cos(trimPitchAttitude(fCondIdx))   -g(fCondIdx)*cos(trimPitchAttitude(fCondIdx))/tas(fCondIdx)
        lSs(fCondIdx)   lRollRate(fCondIdx)                      lYawRate(fCondIdx)                      0
        nSs(fCondIdx)   nRollRate(fCondIdx)                      nYawRate(fCondIdx)                      0
        0               1                                    tan(trimPitchAttitude(fCondIdx))    0
        ];
    B = [
        yAil(fCondIdx) yRud(fCondIdx)
        lAil(fCondIdx)  lRud(fCondIdx)
        nAil(fCondIdx)  nRud(fCondIdx)
        0               0
        ];
    C = eye(size(A));
    D = zeros(size(B));
    acMdlz{fCondIdx} = ss(A, B, C, D, ...
        'StateName', {'ss', 'rollRate', 'yawRate', 'roll'}, ..., ...
        'OutputUnit', {'°', '°/s', '°/s', '°'}, ...
        'InputName', {'ail', 'rud'}, ...
        'InputUnit', {'°', '°'}, ...
        'OutputName', {'ss', 'rollRate', 'yawRate', 'roll'}, ...
        'OutputUnit', {'°', '°/s', '°/s', '°'});

    %%
    % The response transfer functions at each flight condition are:
    disp(['Mach = ' num2str(trimMach(fCondIdx)) ':']);
    zpk(acMdlz{fCondIdx})
end
