function [ c ] = qmul( a,b )
%QMUL qmul(a,b): quaternion multiplication, input must be 1X4 vectors

[rwa,cla]=size(a);
[rwb,clb]=size(b);
if cla~=4 || rwa~=1 || clb~=4 || rwb~=1
    error('input vectors must be 1x4')
end

A=[a(1) -a(2) -a(3) -a(4)
    a(2) a(1) -a(4) a(3)
    a(3) a(4) a(1) -a(2)
    a(4) -a(3) a(2) a(1)];
c=A*b';
c=c';
end

