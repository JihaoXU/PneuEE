function J=jacobian(theta)
%@author Jihao Xu
%@create Dec., 2019

%jacobian based on iterative solution in base coordinate
%param theta: joint vector
%return J: jacobian

% DH parameters
%d
d1=0.1273;
d4=0.163941;
d5=0.1157;
d6=0.0922;
de=0.54;% length of end effector
d=[d1, 0, 0, d4, d5, d6];
%a
a2=0.612;
a3=0.5723;
a=[0, a2, a3, 0, 0, 0];
%alpha
alpha=[pi/2, 0, 0, pi/2, -pi/2, 0];
theta=theta(:);

for i=1:6  
    n_i=[cos(theta(i)); sin(theta(i)); 0];
    o_i=[-sin(theta(i))*cos(alpha(i)); cos(theta(i))*cos(alpha(i));sin(alpha(i))];
    a_i=[sin(theta(i))*sin(alpha(i)); -cos(theta(i))*sin(alpha(i)); cos(alpha(i))];
    p_i=[a(i)*cos(theta(i)); a(i)*sin(theta(i)); d(i)];
    T{i}=[n_i,o_i,a_i,p_i;
        0, 0, 0, 1];
end

for j=1:6
    T0_{j}=eye(4);
    for i=1:j
        T0_{j}=T0_{j}*T{i};
    end
end

T6_e=[eye(3),[0;0;de];
    0,0,0,1];
Te=T0_{6}*T6_e;

z0=[0;0;1;0];
p0=[0;0;0;1];

pe=Te*p0;

J=zeros(6,6);
J(:,1)=[cross(z0(1:3,1),pe(1:3,1)-p0(1:3,1));z0(1:3,1)];
for k=2:6
    zk_1=T0_{k-1}*z0;
    pk_1=T0_{k-1}*p0;
    J(:,k)=[cross(zk_1(1:3,1),pe(1:3,1)-pk_1(1:3,1));zk_1(1:3,1)];
end

end