m = 10;
c = 0;
k = 1;


g = tf([1], [m, c, k]);
t = [0:0.01:100];
[x, t] = impulse(g, t);

figure(1);
plot(t, x);

k = 5;
g = tf([1], [m, c, k]);
t = [0:0.01:100];
[x, t] = impulse(g, t);

figure(2);
plot(t, x);

k = 10;
g = tf([1], [m, c, k]);
t = [0:0.01:100];
[x, t] = impulse(g, t);

figure(3);
plot(t, x);