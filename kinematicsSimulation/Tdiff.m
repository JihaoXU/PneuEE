function dx=Tdiff(Td,Te)
%@author Jihao Xu
%@create Dec., 2019

%calculate the difference of Td and Te in base coordinate
%param Td: desired pose 
%param Te: current pose
%return dx: pose difference

Rd=Td(1:3,1:3);
pd=Td(1:3,4);
Re=Te(1:3,1:3);
pe=Te(1:3,4);
dx=zeros(6,1);
d_p=pd-pe;
d_beta=inv_skew(Rd*(Re')-eye(3));
dx(1:3,1)=d_p;
dx(4:6,1)=d_beta;
end