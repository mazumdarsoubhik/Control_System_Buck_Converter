clc
clear all
%TF of the open loop Buck converter...
s= tf('s');
Open_loop_TF= 2181818.182/(s^2+1000*s+2.2e-5)
%Step response of the open loop buck converter..
step(Open_loop_TF)
%values of Kp,Ki and Kd
kp=4.68108869404948;
ki=1781.09807280784;
kd=0.00282449827895437;
%pid Tf
C= pid(kp,ki,kd)
%Closed loop tf of the system(Buck+PID)
Closed_loop_TF= feedback(C*Open_loop_TF,1)
%finding state space model
[A B C D]= ssdata(Closed_loop_TF);
H= ss(A,B,C,D)
%finding zeros-poles and gain of the closed loop system..
[z,p,k]=zpkdata(H,'v')