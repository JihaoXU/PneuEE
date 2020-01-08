function theta=ik_num(Td,q0)
%@author Jihao Xu
%@create Dec., 2019

%inverse kinematics based on numerical solution
%param Td: desired pose
%param q0: initial guess of joint vector,
%return theta: joint vector in [-pi, pi]

%DH parameters
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

if isempty(q0)
    q0=[0,pi/2,0,pi/2,0,0]+0.2*(rand(1,6)-0.5);
end
q0=q0(:)';
q0=satu(q0);

epsilon=1e-6;
limit=1e3;
gamma=1;
i=0;
q=q0;
while true
    Te=fk(q);
    dx=Tdiff(Td,Te);
    if norm(dx)<epsilon
        break
    end
    q=q+gamma*(pinv(jacobian2(q))*dx)';
    q=satu(q);
    
    i=i+1;
    if i>limit
        break
    end
end
% fprintf("steps:"+num2str(i));
theta=q;
end