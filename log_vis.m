clear,clc,close('all')
%
addpath ../
load('path_2_follow.mat')
load('10_21_NED.mat')
load('10_20_NED.mat')
figure(1)
clf(1)
hold all
plot3(VarName1-mean(VarName1),VarName2-mean(VarName2),-1*(VarName3))
plot3(PN,PE,-1*PD)
plot3(PN1,PE1,-1*PD1)
legend('P2F','Today','Yesterday')