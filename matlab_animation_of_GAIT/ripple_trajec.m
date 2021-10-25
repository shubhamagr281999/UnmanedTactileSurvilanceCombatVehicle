clc
clear
%% assignment of DH paramaters to be used by the function built and in link formation
l1=0.1;
l2=0.2092;
l3=0.3294;
l4=0.2;
alpha=[pi/2,0,0,0];
d=[0,0,0,0];
a=[l1,l2,l3,l4];

%% defining link and initializing bot 
L1=Link('revolute','d',d(1,1),'a',a(1,1),'alpha',alpha(1,1));
L2=Link('revolute','d',d(1,2),'a',a(1,2),'alpha',alpha(1,2));
L3=Link('revolute','d',d(1,3),'a',a(1,3),'alpha',alpha(1,3));
% L4=Link('revolute','d',d(1,4),'a',a(1,4),'alpha',alpha(1,4));
bot=SerialLink([L1,L2,L3],'name','hexapod');
%% comparing the forward dynamincs with our code and fkine
% T=ForwardKinematics([pi,pi/2,pi/4,pi/3,pi/2,pi/6])
theta=[0.1,1.3,-2.4];
% theta(4)=-pi/2-theta(2)-theta(3);
T=bot.fkine(theta);

%% defining trajectory mechnatronal
dutyFactor=4/8;
Cycle_time=2;
points_per_Cycle=20;
stance_time=Cycle_time*dutyFactor;
swing_time=Cycle_time*(1-dutyFactor);

Lstance=0.3;
Lswing=Lstance;
nominal_height=0.1;
%% Trajectory
%swing trjacetory
Ys=-Lswing/2:Lswing/((1-dutyFactor)*points_per_Cycle-1):Lswing/2+0.004;
Ys=flip(Ys);
Zs=0.1*sin((Ys+Lswing/2)*pi/Lswing)-nominal_height;
Xs=0.3+zeros(1,floor((1-dutyFactor)*points_per_Cycle+0.02));
%stance trjectory
Yst=-Lstance/2:Lstance/(dutyFactor*points_per_Cycle-1):Lstance/2+0.004;
Xst=0.3+zeros(1,floor(dutyFactor*points_per_Cycle+0.02));
Zst=-nominal_height+zeros(1,floor(dutyFactor*points_per_Cycle+0.02));
%final path
Xt=[Xs,Xst];
Yt=[Ys,Yst];
Zt=[Zs,Zst];
% Xt=Xst;
% Zt=Zst;
% Yt=Yst;
%% inverse kinematics

trajec=zeros(points_per_Cycle,3);
trajec_real=zeros(points_per_Cycle,3);
for i =1:1:points_per_Cycle
    eff_pose=[Xt(i),Yt(i),Zt(i)];
    t0_3=[eff_pose(1),eff_pose(2),eff_pose(3)];
    theta=zeros(1,3);
    theta(1)=atan2(t0_3(2),t0_3(1));
    t0_3(1)=t0_3(1)-l1*cos(theta(1));
    t0_3(2)=t0_3(2)-l1*sin(theta(1));
    k= sqrt(t0_3(1).^2+t0_3(2).^2);
    J=t0_3(3);
    theta(3)=-acos((k.^2+J.^2-l2.^2-l3.^2)/(2*l2*l3));
    syms x
    alpha=solve(l2*sin(x)+l3*sin(x+theta(3))==J,x);
    if(real(vpa(alpha(1)))>0)
        theta(2)=(vpa(alpha(1)));
    else
        theta(2)=(vpa(alpha(2)));
    end
    trajec(i,:)=theta;
    trajec_real(i,:)=real(theta);
end

%% plotting

