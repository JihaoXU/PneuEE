%@author Jihao Xu
%@create Dec., 2019

%kinematics simulation of the polishing process

clc;
clear all;
close all;

load('trajectory.mat');
t=p_traj(:,1);
N=length(t);

% figure;
% for i=1:N
%     plot3(p_traj(:,2),p_traj(:,3),p_traj(:,4),'-b','MarkerSize',2,'LineWidth',2)
% end
% grid on;
% box on;

%% robot model
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

%robot
%standard D-H convention
L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));
L2 = Link('d', d(2), 'a', a(2), 'alpha',alpha(2));
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4));
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5));
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6));
L7 = Link('d', de, 'a', 0, 'alpha', 0);
robot=SerialLink([L1,L2,L3,L4,L5,L6,L7],'name','polishBot');

%initial point
Re=[0,1,0;
    1,0,0;
    0,0,-1];
pd0=p_traj(1,2:4);
Td0=[Re,pd0';
    0,0,0,1];
q0_guess=[0,pi*2/3,pi*2/3,pi/4,-pi/2,0];
q0=ik_alg(Td0,q0_guess);
figure;
robot.plot([q0 0],'jointdiam',1,'tilesize',0.2,...
    'workspace',[-1,0.25,-0.8,0.8,-1.3,1],'tile1color',[1 1 1],...
    'noshadow','linkcolor','g');
xlabel('x_0/m')
ylabel('y_0/m')
zlabel('z_0/m')
set(gca,'FontSize',10.5,'Fontname','Times New Roman');
hold on;
%% inverse kinematics
% 0: analytical solution
% 1: numerical solution
flag=0;

if flag == 0
    disp("analytical solution");
else
    disp("numerical solution");
end

q=zeros(N,6);
q_1=q0;
for i=1:N
    pd=p_traj(i,2:4);
    Td=[Re,pd';
        0,0,0,1];
    if flag==0
        q(i,:)=ik_alg(Td,q_1);
    else
        q(i,:)=ik_num(Td,q_1);
    end
    q_1=q(i,:);
    Te=fk(q(i,:));
    pe=Te(1:3,4);
    if mod(i,floor(N/200))==1 %plot the process
        robot.plot([q(i,:) 0],'jointdiam',1,'tilesize',0.2,...
            'workspace',[-1,0.25,-0.8,0.8,-1.3,1],'tile1color',[1 1 1],...
            'noshadow','linkcolor','g')
        plot3(pe(1),pe(2),pe(3),'b.','MarkerSize',1,'LineWidth',1)
    end
end

pend=p_traj(end,2:4);
pend(3)=pend(3)+0.3;
Tend=[Re,pend';
    0,0,0,1];
qend=ik_alg(Tend,q(end,:));

robot.plot([qend 0],'jointdiam',1,'tilesize',0.2,...
    'workspace',[-1,0.25,-0.8,0.8,-1.3,1],'tile1color',[1 1 1],...
    'noshadow','linkcolor','g')
hold off;

%% plot angle results
linetype={'-^','-p','-o','-v','-s','--*'};
color=[1    0    0;
    0.8500    0.3250    0.0980;
    0.4    0.45    0.45;
    0.9290    0.6940    0.1250;
    0    0.79    0.34;
    0    0.3    1;];
indices=(1:floor(N/10):N);
indices6=(floor(N/20):floor(N/10):N);

figure;% plot angle
for i=1:5    
    plot(t,q(:,i)/pi*180,linetype{i},'MarkerIndices',indices,...
        'LineWidth',1.5,'MarkerSize',5,'color',color(i,:));
    hold on;
end
plot(t,q(:,6)/pi*180,linetype{6},'MarkerIndices',indices6,...
    'LineWidth',1.2,'MarkerSize',6,'color',color(6,:));
hold off;
legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','Location','East')
xlabel('Time (s)');
ylabel('angle (бу)');
axis([0 250 -120 150])
set(gca, 'XTick',0:25:250);
set(gca, 'YTick',-120:30:150);
set(gca,'FontSize',14,'Fontname','Times New Roman');
box on;
grid on;

file_name="ik_result"+num2str(flag);
% t: (s)
% q: (rad)
% save(file_name,'t','q');

figure;% plot angle velocity
dt=mean(diff(t));
dq=diff(q)/dt;
for i=1:5
    plot(t(1:end-1),dq(:,i)/pi*180,linetype{i},'MarkerIndices',indices,...
        'LineWidth',1.5,'MarkerSize',5,'color',color(i,:));
    hold on;
end
plot(t(1:end-1),dq(:,6)/pi*180,linetype{6},'MarkerIndices',indices6,...
    'LineWidth',1.2,'MarkerSize',6,'color',color(6,:));
hold off;
legend('\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6','Location','East')
xlabel('Time (s)');
ylabel('angle velocity (бу/s)');
axis([0 250 -1.5 1.5])
set(gca, 'XTick',0:25:250); 
set(gca,'FontSize',14,'Fontname','Times New Roman');
box on;
grid on;
