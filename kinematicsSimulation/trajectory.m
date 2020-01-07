%@author Jihao Xu
%@create Dec., 2019

%generate polishing trajectory in operational space

clc
clear all;
close all;

% waypoints
n=4;
L=100;
waypoint=zeros(2*n,4);
waypoint(1,:)=[0,-700,-235,-355];
for i=2:2*n
    waypoint(i,:)=waypoint(i-1,:);
    if mod(i,2)==0
        waypoint(i,3)=-waypoint(i,3);
    else
        waypoint(i,2)=waypoint(i,2)-L/(n-1);
    end
end

% trajectory
a=10;
v=10;
traj=[0,-700,-235,-355];
j=1;
dt=0.05;
for i=2:2*n
    if mod(i,2)==0
        k=3;
    else
        k=2;
    end
    s=waypoint(i,k)-waypoint(i-1,k);
    flag=sign(s);
    s=abs(s);
    t0=traj(end,1);
    t1=v/a;
    t3=t1;
    t2=(s-a*t1^2)/v;
    for t=dt:dt:t1+t2+t3
        if t<=t1
            dis=0.5*a*t^2;
        elseif t<=t1+t2
            dis=0.5*a*t1^2+(t-t1)*v;
        else
            dis=s-0.5*a*(t1+t2+t3-t)^2;
        end
        j=j+1;
        traj(j,:)=waypoint(i-1,:);
        traj(j,1)=t0+t;
        traj(j,k)=traj(j,k)+flag*(dis);
    end
end

p_traj=traj;
p_traj(:,2:4)=p_traj(:,2:4)/1000;
t=p_traj(:,1);
x=p_traj(:,2);
y=p_traj(:,3);
z=p_traj(:,4);

figure;
subplot(311)
plot(t,x,'r','LineWidth',1);
ylabel('x (m)');
axis([0 250 -1 -0.5])
set(gca, 'XTick',0:25:250); 
set(gca, 'YTick',-1:0.1:-0.5);
set(gca,'FontSize',11,'Fontname','Times New Roman');
box on;
grid on;

subplot(312)
plot(t,y,'g','LineWidth',1);
ylabel('y (m)');
axis([0 250 -0.25 0.25])
set(gca, 'XTick',0:25:250); 
set(gca, 'YTick',-0.25:0.1:0.25); 
set(gca,'FontSize',11,'Fontname','Times New Roman');
box on;
grid on;

subplot(313)
plot(t,z,'b','LineWidth',1);
axis([0 250 -0.5 0])
xlabel('Time (s)');
ylabel('z (m)');
set(gca, 'XTick',0:25:250); 
set(gca, 'YTick',-0.5:0.1:0); 
set(gca,'FontSize',11,'Fontname','Times New Roman');
box on;
grid on;

% save('trajectory.mat','p_traj');