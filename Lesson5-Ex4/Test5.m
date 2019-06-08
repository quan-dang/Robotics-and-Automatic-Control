s = tf('s');

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
cp = 0.5 * kc;
% pi control
cpi = 0.45*kc*( 1+ (0.83*tc)/s );
% pid control
cpid = 0.6*kc*( 1+ (0.5*tc)/s + 0.125*tc*s );

% transfer function of feedforward block
gp = series(cp,p);
gpi = series(cpi,p);
gpid = series(cpid,p);

% transfer function of feedback system
fp = feedback(gp, 1);
fpi = feedback(gpi, 1);
fpid = feedback(gpid, 1);

% simulation
yp = lsim(fp, r, t);
ypi = lsim(fpi, r, t);
ypid = lsim(fpid, r, t);

figure(2);
plot(t,yp,t,ypi,t,ypid);







