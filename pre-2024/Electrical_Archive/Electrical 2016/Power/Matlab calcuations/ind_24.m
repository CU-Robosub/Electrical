


Vout = 24;
Vin = 12;
Isw = .6; %data sheet recomends .6 but that makes the inductor unreasonably large
f = 300000;

Dmax = (Vout - Vin)/Vout

Iomax = 1;

Ilmax = Iomax*(1/(1-Dmax))

Iomax_lessthan = (Vin/Vout)*(3.3 -.5*Isw)

L = (Vin/(Isw*f))*Dmax