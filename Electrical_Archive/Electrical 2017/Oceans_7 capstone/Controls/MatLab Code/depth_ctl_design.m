%%  Clear workspace
clear all
close all
clc

%%  Constants: Electronics

sensorPeriod = 1/100;   % Second
sensorResolution = 1/100;

outputResolution = 0.5;
outputDeadZone = 0.1*4;

%%  Physical Constants
G = 9.8;    %meters per second squared, acceleration due to gravity

M = 45;     %Kilograms, mass of submarine
V = .045;      %Cubic Meters, displacement of submarine
Cd = 1.2;   %Moment of inertia about the yaw axis
Fmax = 64;  %Newtons, maximum thrust output of motor
rho = 1000; %Kilograms per cubic meter.  Density of freshwater
A = 1;  % m^2, verticle area of sub
Kd = .5*rho*Cd*A; % combination of constants for the drag force.

Vmax = sqrt(Fmax/Kd);


%%  PID Gain Scheduling
mdl = 'depthPhysics';
opPoints = linspace(0,Vmax,5);

% Preallocate array of state space models
plants = rss(2,1,1,length(opPoints));

for( ii = 1:length(opPoints))
    % linearize the model around the operating points
    [A, B, C, D] = linmod(mdl,[0;opPoints(ii)],0);
    plants(:,:,ii,1) = ss(A,B,C,D);
end

%% Plot Plant Dynamics
% bode(plants)

%% Build Controllers
controllers = pidtune(plants,'p',.1);

%% Simulate and Plot
simMdl = 'depth_system';

sim(simMdl);

%% Plot Simulated Data
close all

plot(Pos);
hold on;
plot(err);
plot(speed);
plot(torqueOut);
legend({'pos','err','speed','torque'})


