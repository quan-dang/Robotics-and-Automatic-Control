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

% simulation settings
t = [0:0.0001:0.1];
r = ones(size(t));

% simulation
y1 = lsim(p, r, t);


figure(1);
plot(t,y1);

% process reaction curve method
R = 370;
L = 0.005;

% p control
cp = 1/(R*L);
% pi control
cpi = 0.9/(R*L) * (1 + 3.33*L/s);
% pid control
cpid = 1.2/(R*L) * (1 + 2*L/s + 0.5*L*s);

% transfer function of feedforward block
gp = series(cp,p);
gpi = series(cpi,p);
gpid = series(cpid,p);


% transfer function of feedback system
fp = feedback(gp, 1);
fpi = feedback(gpi, 1);
fpid = feedback(gpid, 1);

% simulation
r = 1000*ones(size(t));
yp = lsim(fp, r, t);
ypi = lsim(fpi, r, t);
ypid = lsim(fpid, r, t);


figure(2);
plot(t,yp,t,ypi,t,ypid);



