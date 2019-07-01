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

sigma = 0.4; %参照モデルの立ち上がり時間

%極配置法
Kp_l = (1/Kl)*(((4*Tl)/sigma)-1);
Ki_l = (4*Tl)/(Kl*sigma*sigma);
% Ti_l = Kp_l/Ki_l;

Kp_r = (1/Kr)*(4*Tr/sigma-1);
Ki_r = (4*Tr)/(Kr*sigma*sigma);
% Ti_r = Kp_r/Ki_r;
