clear
close all
clc

%% パラメータ設定

Ts = 0.01;
%左モータ
Kl = 6.54;
Tl = 0.125;
Ll = 0.01;
%右モータ
Kr = 6.54;
Tr = 0.145;
Lr = 0.01;

%% 参照モデル
sigma = 0.04; %立ち上がり
delta = 1; %減衰

mu =0.25*(1-delta)+0.51*delta;
rho = Ts/sigma;

p1 = -2*exp(-rho/(2*mu))*cos(sqrt(2*mu-1)*rho/2*mu);
p2 = exp(-rho/mu);

%% A
sigma_A = 0.2; %参照モデルの立ち上がり時間

% 極配置法
Kp_l_A = (1/Kl)*(((4*Tl)/sigma_A)-1);
Ki_l_A = (4*Tl)/(Kl*sigma_A*sigma_A);

Kp_r_A = (1/Kr)*(4*Tr/sigma_A-1);
Ki_r_A = (4*Tr)/(Kr*sigma_A*sigma_A);

%% B
sigma_B = 0.17; %参照モデルの立ち上がり時間

% 極配置法
Kp_l_B = (1/Kl)*(((4*Tl)/sigma_B)-1);
Ki_l_B = (4*Tl)/(Kl*sigma_B*sigma_B);

Kp_r_B = (1/Kr)*(4*Tr/sigma_B-1);
Ki_r_B = (4*Tr)/(Kr*sigma_B*sigma_B);

%% C
sigma_C = 0.15; %参照モデルの立ち上がり時間

%極配置法
Kp_l_C = (1/Kl)*(((4*Tl)/sigma_C)-1);
Ki_l_C = (4*Tl)/(Kl*sigma_C*sigma_C);

Kp_r_C = (1/Kr)*(4*Tr/sigma_A-1);
Ki_r_C = (4*Tr)/(Kr*sigma_C*sigma_C);
