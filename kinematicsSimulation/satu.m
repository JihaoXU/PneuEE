function theta=satu(q)
%@author Jihao Xu
%@create Dec., 2019

%saturate vector q in [-pi, pi]

while sum(q>pi)
    q(q>pi)=q(q>pi)-2*pi;
end
while sum(q<-pi)
    q(q<-pi)=q(q<-pi)+2*pi;
end
theta=q;
end