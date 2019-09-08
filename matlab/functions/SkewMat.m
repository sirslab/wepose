function [ A ] = SkewMat( a )
%SkewMat: Create Skew simmetric matrix from a R3 vector

A=[0 -a(3) a(2)
    a(3) 0 -a(1)
    -a(2) a(1) 0];
end

