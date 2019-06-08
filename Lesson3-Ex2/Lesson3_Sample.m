% simulate motor speed
Rm = 37.5;
Lm = 0.29 * 10^(-3);
Jm = 0.015 * 10^(-7);
Kt = 2.63 * 10^(-3);
Ke = 0.002488;

% motor constants
Tm = (Rm*Jm)/(Ke*Kt);
Te = Lm/Rm;

% transfer function of DC motor
g = tf([1], [Ke*Tm*Te, Ke*Tm, Ke]);

% simulation settings
t = [0: 0.001: 1];
u1 = 1.5 * ones(size(t));
u2 = 3.0 * ones(size(t));
u3 = 4.5 * ones(size(t));
u4 = 6.0 * ones(size(t));

% simulation
y1 = lsim(g, u1, t);
y2 = lsim(g, u2, t);
y3 = lsim(g, u3, t);
y4 = lsim(g, u4, t);

% plot 
figure(1);
plot(t, y1);
title('Em = 1.5');

figure(2);
plot(t, y2);
title('Em = 3.0');

figure(3);
plot(t, y3);
title('Em = 4.5');

figure(4);
plot(t, y4);
title('Em = 6.0');

figure(5);
plot(t, y1, t, y2, t, y3, t, y4);
legend('Em = 1.5', 'Em = 3.0', 'Em = 4.5', 'Em = 6.0');
