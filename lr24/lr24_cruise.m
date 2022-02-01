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
u0 = tas*cos(alpha0);
w0 = tas*sin(alpha0);

%% Longitudinal Aerodynamic Coefficients (Max Weight)
% Steady State
CL_1 = 0.41;
CD_1 = 0.0335;
Cm_1 = 0;
CT_X1 = 0.0335;
Cm_T1 = 0;

% Stability Derivatives
CD_0 = 0.0216;
CD_u = 0.104;
CD_alpha = 0.30;
CT_Xu = -0.07;
CL_0 = 0.13;
CL_u = 0.40;
CL_alpha = 5.84;
CL_alphaRate = 2.2;
CL_q = 4.7;
Cm_0 = 0.050;
Cm_u = 0.050;
Cm_alpha = -0.64;
Cm_alphaRate = -6.7;
Cm_q = -15.5;
Cm_Tu = -0.003;
Cm_Talpha = 0;

% Control Derivatives
CD_elv = 0;
CD_hstab = 0;
CL_elv = 0.46;
CL_hstab = 0.94;
Cm_elv = -1.24;
Cm_hstab = -2.5;