%%  Clear workspace
clear all
close all
clc

%%  Constants: Electronics

sensorPeriod = 1/400;
sensorResolution = 1/1000;

%%  Physical Constants
lenZaxis = 1; %Length of submarine
Lyaw = 0.91375475796;   %Moment of inertia about the yaw axis
Fmax = 32;  %Newtons, maximum thrust output of motor
h = .5;     %Meters, height of the main sub body
armZMotor = .5; %length of moment arm on
Tmax = Fmax*armZMotor;

Cd = 1.2;   %Unitless, Coefficient of drag for a rectangle
rho = 1000; %Kilograms per cubic meter.  Density of freshwater

Kd = rho*Cd*h*(lenZaxis/2)^3/3; % combination of constants for the drag force.

Vmax = sqrt(Tmax/Kd);


%%  PID Gain Scheduling
mdl = 'physics';
opPoints = linspace(0,Vmax,5);

plants = rss(2,1,1,length(opPoints));

for( ii = 1:length(opPoints))
    [A, B, C, D] = linmod(mdl,[0;opPoints(ii)],5);
    plants(:,:,ii,1) = ss(A,B,C,D);
end

%% Plot Plant Dynamics
bode(plants)

%% Build Controllers
controllers = pidtune(plants,'p',1);

%% Simulate and Plot
simMdl = 'MotorSystem';

sim(simMdl);

%% Plot Simulated Data
close all

plot(Pos);
hold on;
plot(err);
plot(speed);
plot(torqueOut);
legend({'pos','err','speed','torque'})


