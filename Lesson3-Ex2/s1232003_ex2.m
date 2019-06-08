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

% transfer function of controller
c1 = 0.001;
c2 = 0.01;
c3 = 0.1;
c4 = 1;
 
% transfer function of feedforward block
g1 = series(c1,p);
g2 = series(c2,p);
g3 = series(c3,p);
g4 = series(c4,p);

% transfer function of feedback system
f1 = feedback(g1, 1);
f2 = feedback(g2, 1);
f3 = feedback(g3, 1);
f4 = feedback(g4, 1);

% simulation settings
t = [0:0.001:1];
r = 1000*ones(size(t));

% simulation
y1 = lsim(f1, r, t);
y2 = lsim(f2, r, t);
y3 = lsim(f3, r, t);
y4 = lsim(f4, r, t);

figure(1);
plot(t,y1,t,y2,t,y3,t,y4);
ylim([0 1000]);
legend('c1 = 0.001', 'c2 = 0.01', 'c3 = 0.1', 'c4 = 1');

figure(2);
plot(t, c1*(r-y1), t, c2*(r-y2), t, c3*(r-y3), t, c4*(r-y4));
ylim([0 10]);