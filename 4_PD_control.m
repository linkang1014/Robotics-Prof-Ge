%% robot 
clear all;clc;
angle = [0.5*pi 0 0 0.5*pi -0.5*pi 0];
a = [0 -0.425 -0.392 0 0 0] ;% a=[0 a2 a3 0 0 0];
d = [0.090 0 0 0.10975 0.09475 0.0815];% d=[d1 0 0 d4 d5  d6];
m = [3.40; 6.19; 5.95; 1.01;1.12;0.64];
r = 0.001*[0,         -25.61,     1.93; ...
           212.5,     0,          113.36;...
           119.93,    0,          26.5;...
           0,         -1.8,       16.34;...
           0,         1.8,        16.34;...
           0,         0,          -1.159]';
I = Get_Inertia_Matrices();
for i = 1:6
    L(i)= Link('d', d(i), 'a', a(i), 'alpha', angle(i),'m',m(i),'r',r(:,i),'I',I(:,:,i));
end
G = [0;0;9.81];
robot = SerialLink(L,'gravity',G);

%% control paramater
Kv=5; 
Kp=0.25*Kv^2;

%% desired end-effect position
t=0:0.01:10;%time
q0 = zeros(6,1);%desired initial position
q2 = [pi;0.5*pi;-0.5*pi;0;pi;0 ];%desired end position
[qr,dqr,ddqr] = jtraj(q0, q2, t);

%% initizal paramater
y=zeros(6,size(qr,1));
dy=zeros(6,size(qr,1));
ddy=zeros(6,size(qr,1));
y(:,1)=q0+0.1;
dy(:,1)=zeros(6,1);
ddy(:,1)=zeros(6,1);

%% Caculation
for i=1:length(t)
ddqr(i,:) = ddqr(i,:)+Kp*(qr(i,:)-y(:,i)')+Kv*(dqr(i,:)-dy(:,i)');%controller
tau=robot.rne(qr(i,:),dqr(i,:),ddqr(i,:));
temp=robot.accel(qr(i,:), dqr(i,:),tau);
dy(:,i+1)=dy(:,i)+temp*0.01;
y(:,i+1)=y(:,i)+dy(:,i)*0.01;
end

%% Show the error
error=y(:,1:size(y,2)-1)-qr';
plot(t,error(1,:),'b');
hold on;
grid on;
plot(t,error(2,:),'r');
plot(t,error(3,:),'g');
plot(t,error(4,:),'c');
plot(t,error(5,:),'m');
plot(t,error(6,:),'k');
xlabel(gca,'Time (s)','FontSize',16);
ylabel(gca,'Error','FontSize',16);
set(gca,'FontSize',14)
discription2 = sprintf('PD Controller');
title(discription2,'FontSize',16);
legend('link1','link2','link3','link4','link5','link6')

 