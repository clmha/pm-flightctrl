%% Learjet 24 Cruise

%% Units
rad = 1;
deg = 1/180*pi()*rad;

m = 1;
ft = 0.3048*m;

kg = 1;
lb = 0.45359237*kg;
slug = 14.59390*kg;

s = 1;

%% Constants
g = 9.81*m/s^2;

% Geometric Data
mac = 7*ft; % Mean Aerodynamic Cord (MAC)

%% Mass and Inertial Data (Max Weight)
m = 13000*lb;
Ix = 28000*slug*ft^2;
Iy = 18800*slug*ft^2;
Iz = 47000*slug*ft^2;
Ixz = 1300*slug*ft^2;

%% Flight Condition (Max Weight)
h = 40000*ft;
M = 0.7;
tas = 677*ft/s;
dynPres = 134.6*lb/ft^2;
xCG = 0.32; % [1/MAC]
alpha0 = 2.7*deg;
theta0 = alpha0;