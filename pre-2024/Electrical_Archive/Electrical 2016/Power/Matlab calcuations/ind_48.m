


Vout = 48;
Iomax = .5;
Vin = 12;
Isw = .9;
f = 300000;

Dmax = (Vout - Vin)/Vout

Ilmax = Iomax*(1/(1-Dmax))


Iomax_lessthan = (Vin/Vout)*(3.3 -.5*Isw)

L = (Vin/(Isw*f))*Dmax

Cout_greater = Iomax/(.01*Vout*f)

Irms_greater = Iomax*sqrt(Dmax/(1-Dmax))