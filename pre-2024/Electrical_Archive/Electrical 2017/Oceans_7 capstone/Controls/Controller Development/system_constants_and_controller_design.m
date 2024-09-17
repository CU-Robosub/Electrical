% Developed by Ocean's 7, Senior Design Lab Team, Academic Year 2016
% Point of contact for this script and control system designing is
% Cassandra Noice, cassandra.noice@colorado.edu

clear all
close all
clc

%% System to be controlled

% physModel is the simulink model to be used for linearizations
% position control: 'physicsPos'
% speed control: 'physicsSpd'

physModel = 'physicsPos';

% simModel is the simulink model used for evaluating plant responses.  It
% utilizes all system constraints and limitations and is non-linear.
% position simulation: 'simPos'
% speed simulation: 'simSpd'

simModel = 'simPos';


%% Physical Constants
% Values inherent to the system for which the controller/compensator is
% being designed

% M (Kg or Kg*m^2) is the measure of the Mass or moment of of the system
M = 0.91375475796;      

% Kd is an approximation of a combination of the variables p, Cd and A,
% where p is the density of the fluid the vehicle is operating in, Cd is
% the coefficient of drag for the body of the vehicle about the appropriate
% axis and A is the surface area in the direction of motion.  For
% rotational dynamics, the equation is different and cannot be represented
% analytically.
%
%        A*Cd*p
% Kd =   ------
%           2

Kd = 25;

% Arm (meters) is the length of the moment arm, or the distance an actuator is from
% the axis of rotation.  If this is for a non rotational model, the Arm is
% equal to 1.

Arm = 0.5;

% Fmax (Newtons) is a measure of the maximum output of each motor.  This value can be
% arbitrarily lowered to simulate artificial saturation limits.  Output
% clamping must be implemented with software independent of the controller 
% mathematics

Fmax = 16;

% NumMotor is the number of motors available to operate on a given
% maneuver.  Given the layout of the CU Robotics submarine, that is 4
% motors for Pitch, Roll, and Depth, and two motors for Yaw and Forward
% Velocity

NumMotor = 2;

% Tmax (N or Nm) is a measure of the maximum amount of thrust or torque able to be
% exerted by the combined motors.  It assumes symmetry of motor outputs.
% This value is used as the controller output saturation limit.

Tmax = Fmax*NumMotor;

% Vmax (m/s or rad/s) is the maximum attainable velocity (translational or rotational)
% achievable by the motors and is used to limit the range of values to look
% at when linearizing the non-linear phyiscal plant.

Vmax = sqrt(Tmax/Kd);




%%  Constants: Electronics
% Constraints and limitations of the motors and sensors being used.

% sensorPeriod (seconds) is the expected duration between updated 
% measurements.

sensorPeriod = 1/100;

% sensorResolution is the maximum granularity of the sensor readings.

sensorResolution = 1/100;

% motorResolution (N) is the granularity of the motor output

motorResolution = 0.5;

% motorDeadZone (N) is a the minimum amout of thrust the motors are able to
% output

motorDeadZone = 0.1;


%% System linearization

% numLinear is the number of linearizations of the plant to make.
numLinear = 5;

% opPoints or operating points is a linearly spaced vector of velocity
% values with which to linearize the plant at.

opPoints = linspace(0,Vmax,5);

% Preallocate array of state space models.  rss() creates an array of
% random state-space models and is useful for lumping all of the
% linearizations into one data structure.
% The inputs are:
% SYSTEM TYPE, for the plant used here, it is a second order differential
% equation, so it is 2 is used as the argument.  
% INPUTS, OUTPUTS: The controllers are being developed as a SISO system
% so they are naturally 1 and 1
% NUMBER OF MODELS:  Equal to the number of linearizations desired

plants = rss(2,1,1,length(opPoints));

for( ii = 1:length(opPoints))
    % linearize the model around the operating points
    [A, B, C, D] = linmod(physModel,[0;opPoints(ii)],0);
    plants(:,:,ii,1) = ss(A,B,C,D);
end

%% Observing plant characteristics (optional)
% bode(plant)

%% Developing a controller/compensator
% From experimental data, it was determined that the motors could spin up
% significantly faster than the entire system could.  Therefore, the
% transfer function of the motors was abstracted and it is assumed that the
% output force of the motors is instantaneously what is requested of them.

% For positional control, the linearization of the plant near zero velocity
% is equivalent to a double integrator.  This makes using an I term
% unfeasible since this will push the phase margin to -90 degrees (VERY
% BAD!)

% A differential term is unnecessary since drag will act as a pretty heavy
% differential gain.

% Using just a P controller is ok, but when velocities are near zero the
% system becomes uncontrollable.

% Through experimentation, it was determined that a gain scheduled lead lag
% compensator network was an optimal solution that could guarantee
% stability across all operational ranges of velocity, regardless of
% whether the compensator being designed is for speed or positional
% control.

% During the AY2016 Senior Design Lab, Ocean's 7, the team responsible for
% this document, developed the lead lag compensator network aiming for a
% bandwidth near 10 radians/second.  To achieve this, a lead compensator
% was used to increase the phase margin at 10 rad/s by 60 degrees.  The lag
% compensator was varied to achieve desirable effects using the plant
% linearized around a zero velocity.

% To do this, open the C block and right click to add a lead or lag
% compensator.  For the lead compensator, set the desired frequency and
% phase gain and let the applet place the pole and zero location. For the
% lag controller, I found this was more 

% For  gain scheduling, the controller gain set to achieve the 10 rad/s
% bandwidth when observing the plant linearized around zero velocity.  The
% control system designer was then switched to observe the plant linearized
% at Vmax.  Through observations during development of the controller, it
% was observed that there was a strong linear relation between the gain
% constants and the system speed.  This allowed for easy development of the
% gain scheduling term.

%        K(v = Vmax) - K(v = 0)
% Gain:  ---------------------- v + K(v = 0)
%                Vmax

% The simModel is already set up to implement this.  To get the two K
% values, export the controllers from the controlSystemDesigner.  These are
% exported as ZPK models.  K(v = 0) = C0.K, K(v = Vmax) = CVmax.K

% Once the two gain values are extracted, set the gain constant of C0 to 1.
% C0.K = 1;

% This provided a simulated system response with approximately 40% 
% overshoot and a 10 second settle time. The controller developed during 
% the course is saved in the file controller.mat.

% Open the MatLab Control System Designer applet using bode plot based
% tuning
controlSystemDesigner('bode',plants);

pause

%%

K0 = C0.K;
C0.K = 1;
KVmax = CVmax.K;



%% Run simulation on simModel

sim(simModel);

%% Plot Simulated Position Data

% if plotting data from a speed controller simulation, comment out the
% plotting of Pos and adjust the strings in the legend accordingly.

figure
hold on;
plot(...
    ref,...
    'LineWidth',2,...
    'Color','black'...
    );
plot(...
    Pos,...
    'LineWidth',2,...
    'Color','blue'...
    );
plot(...
    err,...
    'LineWidth',2,...
    'Color','red'...
    );
plot(...
    speed,...
    'LineWidth',2,...
    'Color','yellow'...
    );
plot(...
    torqueOut);
hold off

legend(...
    {'Reference','Position','Error','Speed','Ctl Output'},...
    'FontSize',16,...
    'Location','east'...
    );
title(...
    'Simulated Response To A 90 Degree Turn',...
    'FontSize',32);
xlabel(...
    'Time [seconds]',...
    'FontSize', 16 ...
    );
ylabel(...
    'Position [radians]',...
    'FontSize', 16 ...
    );
set(gca,...
    'FontSize',16 ...
    );
grid on;

%% End Notes

% Once a controller is settled upon, use the ZPK model of the compensator
% C0 to develop the algorithm for the microcontroller

disc_C = c2d(C0,sensorPeriod);

% From here, take the pole and zero locations from the z transform, set up
% a symbolic variable and recreate the transform equation, then use
% iztrans(sys) to generate the discrete time mathematical model of the
% system.

% The gain  scheduling does not need to be discretized, it can be
% transfered directly to code.

% Example of controller developed for simulations during senior design lab
% U(n) = 1.2743(R-Y) - 0.2743*(0.7598)^n
% Where U(n) is the controller output at the nth time sample.
% n is the number of sample periods elapsed since a new reference was
% provided
% R is the reference input
% Y is the system output (Position or speed)