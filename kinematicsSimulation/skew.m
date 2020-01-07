function M=skew(v)
%@author Jihao Xu
%@create Dec., 2019

%skew-symmetric operator

M=[0,-v(3),v(2);
   v(3),0,-v(1);
   -v(2),v(1),0];
end