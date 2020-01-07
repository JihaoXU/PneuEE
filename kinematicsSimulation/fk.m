function Te=fk(theta)
%@author Jihao Xu
%@create Dec., 2019

%forward kinematics
%param theta: joint vector
%return Te: operational pose

theta=theta(:);
% theta=[t1 t2 t3 t4 t5 t6];
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

for i=1:6  
    n_i=[cos(theta(i)); sin(theta(i)); 0];
    o_i=[-sin(theta(i))*cos(alpha(i)); cos(theta(i))*cos(alpha(i));sin(alpha(i))];
    a_i=[sin(theta(i))*sin(alpha(i)); -cos(theta(i))*sin(alpha(i)); cos(alpha(i))];
    p_i=[a(i)*cos(theta(i)); a(i)*sin(theta(i)); d(i)];
    T{i}=[n_i,o_i,a_i,p_i;
        0, 0, 0, 1];
%     fprintf("T"+num2str(i)+":\n");
%     disp(T{i});
%     fprintf("===========================\n\n")
end

T0_6=eye(4);
for i=1:6
    T0_6=T0_6*T{i};
end

T6_e=[eye(3),[0;0;de];
    0,0,0,1];

Te=T0_6*T6_e;
end
