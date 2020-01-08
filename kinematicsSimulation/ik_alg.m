function theta=ik_alg(Td,q0)
%@author Jihao Xu
%@create Dec., 2019

%inverse kinematics based on analytical solution
%param Td: desired pose
%param q0: initial guess of joint vector,
%           q0==[] return all solutions
%return theta: joint vector in [-pi, pi]

% DH parameters
%d
d1=0.1273;
d4=0.163941;
d5=0.1157;
d6=0.0922;
de=0.54;% length of end effector
%a
a2=0.612;
a3=0.5723;
%Te
n_xyz=Td(1:3,1);
nx=n_xyz(1);
ny=n_xyz(2);
nz=n_xyz(3);
o_xyz=Td(1:3,2);
ox=o_xyz(1);
oy=o_xyz(2);
oz=o_xyz(3);
a_xyz=Td(1:3,3);
ax=a_xyz(1);
ay=a_xyz(2);
az=a_xyz(3);
p_xyz=Td(1:3,4);
px=p_xyz(1);
py=p_xyz(2);
pz=p_xyz(3);
q0=q0(:)';

theta=zeros(8,6);
i=1;
for i1=-1:2:1
    rou_1=sqrt(((d6+de)*ay-py)^2+(px-(d6+de)*ax)^2);
    phi_1=atan2(px-(d6+de)*ax,(d6+de)*ay-py);
    
    theta1=atan2(sign(i1)*sqrt(max(0,1-(d4/rou_1)^2)),d4/rou_1)+phi_1;
    theta1=satu(theta1);
    c1=cos(theta1);s1=sin(theta1);
    for i5=-1:2:1
        theta5=atan2(sign(i5)*sqrt(max(0,1-(s1*ax-c1*ay)^2)),s1*ax-c1*ay);
        c5=cos(theta5);s5=sin(theta5);
        theta6=atan2(-s5*(ox*s1-oy*c1),s5*(nx*s1-ny*c1));
        c6=cos(theta6);s6=sin(theta6);
        for i3=-1:2:1
            t14=c1*(-ax*(d6+de)+px+d5*ox*c6+d5*nx*s6)+...
                s1*(-ay*(d6+de)+py+d5*oy*c6+d5*ny*s6);
            t24=-d1-az*(d6+de)+pz+d5*oz*c6+d5*nz*s6;
            c3=(t14^2+t24^2-a2^2-a3^2)/(2*a2*a3);
            theta3=atan2(sign(i3)*sqrt(max(0,1-c3^2)),c3);
            s3=sin(theta3);
            theta2=atan2((a3*c3+a2)*t24-a3*s3*t14,(a3*c3+a2)*t14+a3*s3*t24);
            theta4=atan2(-c6*(ox*c1+oy*s1)-s6*(nx*c1+ny*s1),c6*oz+s6*nz)-theta3-theta2;
            theta4=satu(theta4);            
            theta(i,:)=[theta1,theta2,theta3,theta4,theta5,theta6];
            i=i+1;
        end
    end
end


if ~isempty(q0)
    index=1;
    min_delta=inf;
    for i=1:8
        if norm(theta(i,:)-q0)<min_delta
            index=i;
            min_delta= norm(theta(i,:)-q0);
        end
    end
    theta=theta(index,:);
end

end



