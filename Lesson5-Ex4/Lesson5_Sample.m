s = tf('s');
%k = 122;
k = 1000;
p = k / ((0.1*s + 1) * (s+1) *(10*s + 1));
g = feedback(p,1);
t = 0:0.001:1;
r = ones(size(t));
y = lsim(g,r,t);

figure(1);
plot(t,y);