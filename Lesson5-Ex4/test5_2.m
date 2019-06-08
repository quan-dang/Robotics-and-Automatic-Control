% simulate motor speed
Rm = 37.5;
Lm = 0.29 * 10^(-3);
Jm = 0.015 * 10^(-7);
Kt = 2.63 * 10^(-3);
Ke = 0.002488;

% motor constants
Tm = (Rm*Jm)/(Ke*Kt);
Te = Lm/Rm;

% transfer function of plant of DC motor
p = tf([1], [Ke*Tm*Te, Ke*Tm, Ke, 0]);

% transfer funcion of controller
c1 = 320; % this is k parameter in P formula.

% transfer function of feedforward block
g1 = series(c1,p);

% transfer function of feedback system
f1 = feedback(g1, 1);

% simulation settings
t = [0:0.0001:0.1];
r = 1000*ones(size(t));

% simulation
y1 = lsim(f1, r, t);

figure(1);
plot(t,y1);

% transfer funcion of controller
kc = 320; % this is k parameter in P formula.
tc = 0.1/61.0; % By zieger method

% P-control
%cp = 0.5 * kc;
%cp = 0.01*kc;

% pid control
cpi = 0.45*kc*(1+(0.8*ti)/s);

% transfer function of feedforward block
gpi = series(cpi,p);

% transfer function of feedback system
fpi = feedback(gpi, 1);

% simulation
ypi = lsim(fpi, r, t);

figure(2);
plot(t,ypi);







