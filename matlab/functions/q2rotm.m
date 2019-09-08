function [ R ] = q2rotm(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);
qh=[q1 q2 q3]';
Qh=[0 -q3 q2
    q3 0 -q1
    -q2 q1 0 ];

% R=[q0^2+q1^2-q2^2-q3^2  2*(q1*q2-q0*q3) 2*(q0*q2+q1*q3)
%     2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1)
%     2*(q1*q3-q0*q2) 2*(q0*q1+q2*q3) q0^2-q1^2-q2^2+q3^2];

R=(q0^2-qh'*qh)*eye(3)+2*qh*qh'+2*q0*Qh;

end

