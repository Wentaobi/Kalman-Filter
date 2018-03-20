%% Five equation explaination
% 1: variable relationship between this moment and last moment
% 2: figure out estimated value and actual value variance, to estimate
% current estimated value
% 3: kf gain (matrix)
% 4: estimated value
% 5: adjust/correct estimated value and actual value var
%% initialized inforamtion
clear; 
clc;
close all;

N = 100;            % kf tracking points
r = 3;              % estimated number of variables
t = 1:N;            % time stamp
T = 1;              % scan interval
s = zeros(1,N);     % declare distance
v = zeros(1,N);     % declare speed
a = zeros(1,N);     % declare acccelerator
a(t) = 20;          % accelerator, const as default
s0 = 1000;          % tracked distance, initial 1000
v0 = 50;            % initial speed
for n = 1:N
    v(n) = v0 + a(n)*n;
    s(n) = 1000 + v0*n + 0.5*a(n)*n*n;
end
wt = randn(1,N);    % system noise
wt = sqrt(4)*wt./std(wt);
s = s + wt;
v = v + wt;
a(t) = a(t) + wt(t);
%% kf part
Y0 = [900 0 0]';    % state matrix initial
Y = [Y0 zeros(r,N-1)];% stste matrix: k_s,k_v,k_a
A = [1 1 0;         % transfer matrix
     0 1 1;         
     0 0 1];
H = [1 0 0];        % measure matrix
Qk = [0 0 0;        % system noise matrix
      0 0 0;
      0 0 20];
Rk = 16;            % measure noise matrix
P0 = [30 0 0;       % avg-square error matrix initial
      0 20 0;
      0 0 10];
P1 = P0;
P2 = zeros(r);
for k = 2:N
    Y(:,k) = A*Y(:,k-1);
    P2 = A*P1*A' + Qk;
    Kk = P2*H'*inv(H*P2*H'+Rk);
    Y(:,k) = Y(:,k)+Kk*(s(:,k)-H*Y(:,k));
    P1 = (eye(r)-Kk*H)*P2;
end
k_s = Y(1,:);
k_v = Y(2,:);
k_a = Y(3,:);
subplot(3,1,1);
plot(t,s(t),'-',t,k_s(t),'o');
title('distance');
legend('actual avlue','estimated value');
xlabel('t');
ylabel('s');
subplot(3,1,2);
plot(t,v(t),t,k_v(t),'+');
title('speed');
legend('actual avlue','estimated value');
xlabel('t');
ylabel('v');
subplot(3,1,3);
plot(t,a(t),t,k_a(t),'-x');
title('accelerator');
legend('actual avlue','estimated value');
xlabel('t');
ylabel('a');
axis([0,N,0,30]);


















