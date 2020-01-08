function v=inv_skew(M)
%@author Jihao Xu
%@create Dec., 2019

%inverse skew-symmetric operator

v=zeros(3,1);
v(1)=M(3,2);
v(2)=M(1,3);
v(3)=M(2,1);
end