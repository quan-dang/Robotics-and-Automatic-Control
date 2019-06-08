clear;
% vehicle mechanical parameter
% distance between two wheels
b = 0.3;

% simulation parameter
% time step
dt = 0.01;

% 1st segment: rotation pi/2 [rad/s] for 2 sec
% vR = 0.2355 [m/s], vL = -0.2355 [m/s]
t1 = [0:dt:2-dt];
vR1 = 0.2355 * ones(size(t1));
vL1 = -0.2355 * ones(size(t1));


% 2nd segment: 
t2 = [2:dt:11.99];
vR2 = 1 * ones(size(t2));
vL2 = 1 * ones(size(t2));

% 3rd segment: translation 
t3 = [12:dt:13-dt];
vR3 = 0.2355 * ones(size(t3));
vL3 = -0.2355 * ones(size(t3));


% 4th segment: translation 
t4 = [13:dt:16-dt];
vR4 = 1 * ones(size(t4));
vL4 = 1 * ones(size(t4));

% simulation setup
t = [t1, t2, t3, t4];
vR = [vR1, vR2, vR3, vR4];
vL = [vL1, vL2, vL3, vL4];

% position variables
x = zeros(size(t));
y = zeros(size(t));
q = zeros(size(t));

% initial values
x(1) = 0;
y(2) = 0;
q(1) = pi;

% loop
for k = 1:1599
    x(k+1) = x(k) + dt*cos(q(k))/2*vL(k) + dt*cos(q(k))/2*vR(k);
    y(k+1) = y(k) + dt*sin(q(k))/2*vL(k) + dt*sin(q(k))/2*vR(k);
    q(k+1) = q(k) + dt*vR(k)/b - dt*vL(k)/b;
end

% plot x-y position
figure(1);
plot(x,y);
%xlim([-1 11]);
%ylim([-1 4]);

% plot of orietntation
figure(2);
plot(t, q);