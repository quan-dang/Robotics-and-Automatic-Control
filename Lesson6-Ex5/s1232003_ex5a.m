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
p = tf([1], [Ke*Tm*Te, Ke*Tm, Ke]);

% transfer function of controller
cp = 1;

% transfer function of feedforward block
gp = series(cp,p);


% transfer function of feedback system
fp = feedback(gp, 1);

% simulation settings
t = [0:0.001:1];
r1 = 1 * sin(10*t);
r2 = 1 * sin(100*t); % speed up frequency
r3 = 1 * sin(300*t);

% simulation
y1 = lsim(fp, r1, t);
y2 = lsim(fp, r2, t);
y3 = lsim(fp, r3, t);

% plot
figure(1);
plot(t, r1 ,t, y1);
legend('reference', 'actual');
title('w = 10');

figure(2);
plot(t, r2 ,t, y2);
legend('reference', 'actual');
title('w = 100');

figure(3);
plot(t, r3 ,t, y3);
legend('reference', 'actual');
title('w = 300');

% bode plot
figure(4);
bode(gp);
grid on;
margin(gp);