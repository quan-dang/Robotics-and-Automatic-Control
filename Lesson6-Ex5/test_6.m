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
ci = 1/s;
cd = s;

% transfer function of feedforward block
gp = series(cp,p);
gi = series(ci,p);
gd = series(cd,p);

% transfer function of feedback system
fp = feedback(gp, 1);
fi = feedback(gi, 1);
fd = feedback(gd, 1);

% simulation settings
t = [0:0.001:1];
r1 = 1 * sin(10*t);
r2 = 1 * sin(100*t); % speed up frequency
r3 = 1 * sin(300*t);

% simulation
y1 = lsim(fi, r1, t);
y2 = lsim(fi, r2, t);
y3 = lsim(fi, r3, t);

% plot
figure(1);
plot(t, r2 ,t, y2);
legend('reference', 'actual');

% bode plot
figure(2);
bode(gi);
grid on;
margin(gi);