function J=jacobian2(theta)
%@author Jihao Xu
%@create Dec., 2019

%jacobian based on explicit expression in base coordinate
%param theta: joint vector
%return J: jacobian

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
%theta
theta=theta(:);
c1=cos(theta(1));s1=sin(theta(1));
c2=cos(theta(2));s2=sin(theta(2));
c3=cos(theta(3));s3=sin(theta(3));
c4=cos(theta(4));s4=sin(theta(4));
c5=cos(theta(5));s5=sin(theta(5));
c6=cos(theta(6));s6=sin(theta(6));
c23=cos(theta(2)+theta(3));s23=sin(theta(2)+theta(3));
c234=cos(theta(2)+theta(3)+theta(4));s234=sin(theta(2)+theta(3)+theta(4));
J=zeros(6,6);J(6,1)=1;

J(1,1)=(d6+de)*(c1*c5+c234*s1*s5)+d4*c1-a2*c2*s1-d5*s234*s1-a3*s1*c23;
J(2,1)=(d6+de)*(c5*s1-c234*c1*s5)+d4*s1+a2*c1*c2+d5*s234*c1+a3*c1*c23;
J(1,2)=c1*(d5*c234-a3*s23-a2*s2+(d6+de)*s5*s234);
J(2,2)=s1*(d5*c234-a3*s23-a2*s2+(d6+de)*s5*s234);
J(3,2)=a3*c23+a2*c2+d5*s234-(d6+de)*c234*s5;
J(4,2)=s1;
J(5,2)=-c1;
J(1,3)=c1*(d5*c234-a3*s23+(d6+de)*s234*s5);
J(2,3)=s1*(d5*c234-a3*s23+(d6+de)*s234*s5);
J(3,3)=a3*c23+d5*s234-(d6+de)*c234*s5;
J(4,3)=s1;
J(5,3)=-c1;
J(1,4)=c1*(d5*c234+(d6+de)*s234*s5);
J(2,4)=s1*(d5*c234+(d6+de)*s234*s5);
J(3,4)=d5*s234-(d6+de)*c234*s5;
J(4,4)=s1;
J(5,4)=-c1;
J(1,5)=-(d6+de)*(c234*c5*c1+s1*s5);
J(2,5)=-(d6+de)*(c234*c5*s1-c1*s5);
J(3,5)=-(d6+de)*s234*c5;
J(4,5)=s234*c1;
J(5,5)=s234*s1;
J(6,5)=-c234;
J(4,6)=c5*s1-c234*c1*s5;
J(5,6)=-c1*c5-c234*s1*s5;
J(6,6)=-s234*s5;
end