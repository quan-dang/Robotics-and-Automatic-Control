clear;

% vehicle mechanical parameter
% distance between two wheels
b = 0.3;

% simulation params
% time step
dt = 0.1;
% simulate up to 20s
t = [0:dt:20];

% constants
% state dimension
cN = 3;

% two target positions
rD1 = [10; 0];
rD2 = [10; 3];
rD = [rD1, rD2];

% simulation setup
% variables
% true position variables
% rT(:,k) = [xT(k), yT(k), qT(k)]
% [m/s, m/s, rad]
rT = zeros(cN, length(t));
rT(:,1) = [0; 0; pi];

% control: transitional and rotational speed
% [m/s, rad/s]
v = zeros(1, length(t));
w = zeros(1, length(t));

% control: left and right wheel speed: vL, vR
vL = zeros(1, length(t));
vR = zeros(1, length(t));
u = zeros(2, length(t));

% task number
cL = 2; % rD1 and rD2
l = 1; % index

% maximum vehicle transitional and rotational speed
vc = 1;
wc = pi/2;

% margin distance to reach to target
dc = 0.1; % 0.01 is also ok

% main loop
for k = 1:200
    % find a vehicle control from desired and current plan
    % plan a transitional and rotational speed of a vehicle
    r = rD(:,l) - rT(1:2,k);
    v(k) = norm(r) / dt;
    
    % speed limit
    if v(k) > vc
       v(k) = vc; 
    end
    
    % rotational speed
    w(k) = (atan2(r(2), r(1)) - rT(3,k)) / dt;
    
    % speed limit
    if w(k) > wc
        w(k) = wc;
    elseif w(k) < -wc
        w(k) = -wc;
    end
    
    % convert to left and right wheel speed
    vL(k) = v(k) - b*w(k)/2;
    vR(k) = v(k) + b*w(k)/2;
    u(:,k) = [vL(k); vR(k)];
    
    % forward kinematrics of vehicle
    rT(:,k+1) = rT(:,k) + dt * J(rT(3,k),b) * u(:,k);
    
    % if the vehicle is close to target, move to next target
    if norm(rD(:,l) - rT(1:2,k+1)) < dc
        l = l + 1;
    end
    if (l > cL)
        break;
    end    
end

% plot true and estimated x-y position
figure(1);
hold on;
plot(rT(1,:), rT(2,:), 'bo');
plot(rD(1,1), rD(2,1), 'ro');
plot(rD(1,2), rD(2,2), 'ro');
xlim([-1, 11]);
ylim([-1, 4]);
axis equal
hold off;

% J function
function [m] = J(q, b)
    m = [cos(q)/2, cos(q)/2; sin(q)/2, sin(q)/2; -1/b, 1/b];
end





