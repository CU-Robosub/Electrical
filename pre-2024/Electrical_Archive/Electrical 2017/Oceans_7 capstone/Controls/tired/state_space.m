% State Space: 

% constants: 
Ke = 0.05788; 
Kt = 0.05788; % see notebook
Jm = 3.814776e-6;  % This is without motor inertia bc I can't measure that
La = 0.00012;  % from datasheet
Ra = 0.24; % from datasheet
Cd = 1.4; 
r = 7*2.54/100;
rho = 1000; % Kg/m^3
Kd = rho*Cd*pi*r/12;
Vmax = 0.133; % rad/sec
d1 = 0.5; % meters
Jr = 1; % NEED THIS VALUE
s = tf('s');

% This was old design, I don't think we are allowed to use transfer
% functions for A, B, C, and/or D
% A = [0, 1, 0, 0; 0, -Kd*Vmax/2*Jr, 0, -2*d1*Kt*Ke/Jr*(Ra+s*La); 0, 0, 0, 1; 0, 0, 0, -Kt*Ke/Jm*(Ra+s*La)];
% B = [0; 2*d1*Kt/Jr*(Ra+s*La); 0; Kt/Jm*(Ra+s*La)];
% C = [1, 1, 0, 0];
% D = 0;
% C_maybs = [1,0,0,0];

%This iteration doesnt work due to the low rank of A
% New design with i as input (will need two observers?)
% A = [0, 1, 0, 0, 0; 0, -Kd*Vmax/2*Jr, 0, 0, Kt*d1/Jr; 0, 0, 0, 1, 0; 0, 0, 0, 0, Kt/Jm; 0, 0, 0, -Ke/La, -Ra/La];
% B = [0; 0; 0; 0; 1/La]; 
% C = [1, 1, 0, 0, 0]; 
% C_maybs = [1, 0, 0, 0, 0];
% D = 0; 


%Third attept, remo_e theta_r and psi
A = [-Kd*Vmax/2*Jr, 0, Kt*d1/Jr; 0, 0, Kt/Jm; 0, -Ke/La, -Ra/La];
B = [0; 0; 1/La]; 
C = [1, 0, 0]; 
D = 0; 


poles = eig(A); % as of now, 2 poles out of place. 
% states = {'theta_r' 'theta_r_dot' 'psi' 'psi_dot' 'ia'};
% inputs = {'Vin'};
% outputs = {'theta_r'; 'theta_r_dot'};

sys_ss = ss(A,B,C,D);

co = ctrb(sys_ss);
controllability = rank(co);

p1 = -26.8+24.04i;
p2 = -26.8-24.04i;
p3 = -120;

K = place(A,B,[p1,p2,p3]);
Nbar = rscale(sys_ss,K);
t = 0:0.001:1;
u = 0.001*ones(size(t));
sys_cl = ss(A-B*K,B,C,D);
% step(sys_cl)
% lsim(Nbar*u,t)