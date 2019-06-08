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
p = tf([1], [Ke*Tm*Te, Ke*Tm, Ke]);

% transfer funcion of controller
c1 = 0.1 + tf([0.1], [1 0]);
c2 = 0.1 + tf([1.0], [1 0]);
c3 = 0.1 + tf([10.0], [1 0]);

 
% transfer function of feedforward block
g1 = series(c1,p);
g2 = series(c2,p);
g3 = series(c3,p);

% transfer function of feedback system
f1 = feedback(g1, 1);
f2 = feedback(g2, 1);
f3 = feedback(g3, 1);

% simulation settings
t = [0:0.001:1];
r = 1000*ones(size(t));

% simulation
y1 = lsim(f1, r, t);
y2 = lsim(f2, r, t);
y3 = lsim(f3, r, t);

figure(1);
plot(t,y1,t,y2,t,y3);
ylim([900 1100]);
legend('ki_1 = 0.1', 'ki_2 = 1.0', 'ki_3 = 10.0');