clc
clear
close

%% inputs to the code
%initial pose 
xi=5;   
yi=4;
hi=0;   %intial heading

%final point to go to
xf=6.5;
yf=5.5;

%perspective view from 
xc=3;
yc=4;
zc=2;

%% assignment of robot dimension
%link lengths
l1=0.25;
l2=0.13;
l3=0.18;

%base dimensions
l=0.8;
b=0.4;
h=0.05;

%% defining trajectory of foot mechatronal giat pattern
S=[0,1,2,2,3,0];  % defines the gait pattern pahse difference
dutyFactor=5/8;             %param of gait picked from paper mentioned in report
points_per_Cycle=40;        %number of points sampled in the foor trajectory
Lstance=0.3;                %distance moved by leg in each cycle
Lswing=Lstance;         
nominal_height=0.1;         %nominal height of bot above ground

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
rotation_per_cycle=Lstance/(0.3+b/2); %if bot is roating then it would rotate by this much degree in each cycle
%final path after combining the swing and stance phase
Xt=[Xs,Xst];  
Yt=[Ys,Yst];
Zt=[Zs,Zst];

%% inverse kinematics for calculating joint angles from the trajectory defiend above
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

%% animation code below
% rotation
j = 1;
figh = figure;
hf=atan((yf-yi)/(xf-xi));
h_diff=hf-hi;
number_of_rotation_point=ceil(points_per_Cycle*abs(h_diff)/rotation_per_cycle);
for i=0:1:number_of_rotation_point
    h=i*rotation_per_cycle/points_per_Cycle*h_diff/abs(h_diff)+hi;
    % calculating the next set of cordinates for each link and base
    q0=trajec_real(points_per_Cycle - rem(S(1)*10+i,points_per_Cycle),:);
    q1=trajec_real(points_per_Cycle - rem(S(2)*10+i,points_per_Cycle),:);
    q2=trajec_real(points_per_Cycle - rem(S(3)*10+i,points_per_Cycle),:);
    q3=trajec_real(rem(S(4)*10+i,points_per_Cycle)+1,:);
    q4=trajec_real(rem(S(5)*10+i,points_per_Cycle)+1,:);
    q5=trajec_real(rem(S(6)*10+i,points_per_Cycle)+1,:);

    base_pose=[0 0 nominal_height];
    base_LF=base_pose+[l/2 b/2 0];
    base_LR=base_pose+[-l/2 b/2 0];
    base_RF=base_pose+[l/2 -b/2 0];
    base_RR=base_pose+[-l/2 -b/2 0];
    base_LM=base_pose+[0 b 0];
    base_RM=base_pose+[0 -b 0];

    a0=base_pose+[Lstance+0.1 b/2 0];
    a1=base_pose+[0 b 0];
    a2=base_pose+[-Lstance-0.1 b/2 0];
    a3=base_pose+[Lstance+0.1 -b/2 0];
    a4=base_pose+[0 -b 0];
    a5=base_pose+[-Lstance-0.1 -b/2 0];

    b0=a0+[-l1*sin(q0(1)) +l1*cos(q0(1)) 0 ];
    b1=a1+[-l1*sin(q1(1)) +l1*cos(q1(1)) 0 ];
    b2=a2+[-l1*sin(q2(1)) +l1*cos(q2(1)) 0 ];
    b3=a3+[-l1*sin(q3(1)) -l1*cos(q3(1)) 0 ];
    b4=a4+[-l1*sin(q4(1)) -l1*cos(q4(1)) 0 ];
    b5=a5+[-l1*sin(q5(1)) -l1*cos(q5(1)) 0 ];

    c0=b0+[-l2*sin(q0(1))*cos(q0(2)) l2*cos(q0(1))*cos(q0(2))  l2*sin(q0(2))];
    c1=b1+[-l2*sin(q1(1))*cos(q1(2)) l2*cos(q1(1))*cos(q1(2))  l2*sin(q1(2))];
    c2=b2+[-l2*sin(q2(1))*cos(q2(2)) l2*cos(q2(1))*cos(q2(2))  l2*sin(q2(2))];
    c3=b3+[-l2*sin(q3(1))*cos(q3(2)) -l2*cos(q3(1))*cos(q3(2))  l2*sin(q3(2))];
    c4=b4+[-l2*sin(q4(1))*cos(q4(2)) -l2*cos(q4(1))*cos(q4(2))  l2*sin(q4(2))];
    c5=b5+[-l2*sin(q5(1))*cos(q5(2)) -l2*cos(q5(1))*cos(q5(2))  l2*sin(q5(2))];

    d0=c0+[-l3*sin(q0(1))*cos(q0(2)+q0(3)) l3*cos(q0(1))*cos(q0(2)+q0(3))  l3*sin(q0(2)+q0(3))];
    d1=c1+[-l3*sin(q1(1))*cos(q1(2)+q1(3)) l3*cos(q1(1))*cos(q1(2)+q1(3))  l3*sin(q1(2)+q1(3))];
    d2=c2+[-l3*sin(q2(1))*cos(q2(2)+q2(3)) l3*cos(q2(1))*cos(q2(2)+q2(3))  l3*sin(q2(2)+q2(3))];
    d3=c3+[-l3*sin(q3(1))*cos(q3(2)+q3(3)) -l3*cos(q3(1))*cos(q3(2)+q3(3))  l3*sin(q3(2)+q3(3))];
    d4=c4+[-l3*sin(q4(1))*cos(q4(2)+q4(3)) -l3*cos(q4(1))*cos(q4(2)+q4(3))  l3*sin(q4(2)+q4(3))];
    d5=c5+[-l3*sin(q5(1))*cos(q5(2)+q5(3)) -l3*cos(q5(1))*cos(q5(2)+q5(3))  l3*sin(q5(2)+q5(3))];

    base_LF=RotZ(h)*base_LF'+[xi;yi;0];
    base_LR=RotZ(h)*base_LR'+[xi;yi;0];
    base_RF=RotZ(h)*base_RF'+[xi;yi;0];
    base_RR=RotZ(h)*base_RR'+[xi;yi;0];
    base_LM=RotZ(h)*base_LM'+[xi;yi;0];
    base_RM=RotZ(h)*base_RM'+[xi;yi;0];

    [base_LF(1),base_LF(2)]=Projection_per(base_LF,xc,yc,zc);
    [base_LR(1),base_LR(2)]=Projection_per(base_LR,xc,yc,zc);
    [base_RF(1),base_RF(2)]=Projection_per(base_RF,xc,yc,zc);
    [base_RR(1),base_RR(2)]=Projection_per(base_RR,xc,yc,zc);
    [base_LM(1),base_LM(2)]=Projection_per(base_LM,xc,yc,zc);
    [base_RM(1),base_RM(2)]=Projection_per(base_RM,xc,yc,zc);

    a0=RotZ(h)*a0'+[xi;yi;0];
    a1=RotZ(h)*a1'+[xi;yi;0];
    a2=RotZ(h)*a2'+[xi;yi;0];
    a3=RotZ(h)*a3'+[xi;yi;0];
    a4=RotZ(h)*a4'+[xi;yi;0];
    a5=RotZ(h)*a5'+[xi;yi;0];
    
    [a0(1),a0(2)]=Projection_per(a0,xc,yc,zc); 
    [a1(1),a1(2)]=Projection_per(a1,xc,yc,zc); 
    [a2(1),a2(2)]=Projection_per(a2,xc,yc,zc); 
    [a3(1),a3(2)]=Projection_per(a3,xc,yc,zc); 
    [a4(1),a4(2)]=Projection_per(a4,xc,yc,zc); 
    [a5(1),a5(2)]=Projection_per(a5,xc,yc,zc);

    b0=RotZ(h)*b0'+[xi;yi;0];
    b1=RotZ(h)*b1'+[xi;yi;0];
    b2=RotZ(h)*b2'+[xi;yi;0];
    b3=RotZ(h)*b3'+[xi;yi;0];
    b4=RotZ(h)*b4'+[xi;yi;0];
    b5=RotZ(h)*b5'+[xi;yi;0];
    
    [b0(1),b0(2)]=Projection_per(b0,xc,yc,zc); 
    [b1(1),b1(2)]=Projection_per(b1,xc,yc,zc); 
    [b2(1),b2(2)]=Projection_per(b2,xc,yc,zc); 
    [b3(1),b3(2)]=Projection_per(b3,xc,yc,zc); 
    [b4(1),b4(2)]=Projection_per(b4,xc,yc,zc); 
    [b5(1),b5(2)]=Projection_per(b5,xc,yc,zc);

    c0=RotZ(h)*c0'+[xi;yi;0];
    c1=RotZ(h)*c1'+[xi;yi;0];
    c2=RotZ(h)*c2'+[xi;yi;0];
    c3=RotZ(h)*c3'+[xi;yi;0];
    c4=RotZ(h)*c4'+[xi;yi;0];
    c5=RotZ(h)*c5'+[xi;yi;0];
    
    [c0(1),c0(2)]=Projection_per(c0,xc,yc,zc); 
    [c1(1),c1(2)]=Projection_per(c1,xc,yc,zc); 
    [c2(1),c2(2)]=Projection_per(c2,xc,yc,zc); 
    [c3(1),c3(2)]=Projection_per(c3,xc,yc,zc); 
    [c4(1),c4(2)]=Projection_per(c4,xc,yc,zc); 
    [c5(1),c5(2)]=Projection_per(c5,xc,yc,zc);

    d0=RotZ(h)*d0'+[xi;yi;0];
    d1=RotZ(h)*d1'+[xi;yi;0];
    d2=RotZ(h)*d2'+[xi;yi;0];
    d3=RotZ(h)*d3'+[xi;yi;0];
    d4=RotZ(h)*d4'+[xi;yi;0];
    d5=RotZ(h)*d5'+[xi;yi;0];
    
    [d0(1),d0(2)]=Projection_per(d0,xc,yc,zc); 
    [d1(1),d1(2)]=Projection_per(d1,xc,yc,zc); 
    [d2(1),d2(2)]=Projection_per(d2,xc,yc,zc); 
    [d3(1),d3(2)]=Projection_per(d3,xc,yc,zc); 
    [d4(1),d4(2)]=Projection_per(d4,xc,yc,zc); 
    [d5(1),d5(2)]=Projection_per(d5,xc,yc,zc); 


    %plotting
    %plotting base
    clf
    base_points=[base_LF base_LM base_LR base_RR base_RM base_RF];
    hold on
    patch(base_points(1,:),base_points(2,:),'db', 'facealpha',0.4)
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([xi-3 xi+3])    
    ylim([yi-3 yi+3])
    zlim([0 3])
    %plotting link1
    
    l1_0=[a0 b0];
    l1_1=[a1 b1];
    l1_2=[a2 b2];
    l1_3=[a3 b3];
    l1_4=[a4 b4];
    l1_5=[a5 b5];    


    plot(l1_0(1,:),l1_0(2,:),'LineWidth',2,'Color','k')
    plot(l1_1(1,:),l1_1(2,:),'LineWidth',2,'Color','k')
    plot(l1_2(1,:),l1_2(2,:),'LineWidth',2,'Color','k')
    plot(l1_3(1,:),l1_3(2,:),'LineWidth',2,'Color','k')
    plot(l1_4(1,:),l1_4(2,:),'LineWidth',2,'Color','k')
    plot(l1_5(1,:),l1_5(2,:),'LineWidth',2,'Color','k')

    %plotting link2
    l2_0=[b0 c0];
    l2_1=[b1 c1];
    l2_2=[b2 c2];
    l2_3=[b3 c3];
    l2_4=[b4 c4];
    l2_5=[b5 c5];
    
    plot(l2_0(1,:),l2_0(2,:),'LineWidth',2,'Color','r')
    plot(l2_1(1,:),l2_1(2,:),'LineWidth',2,'Color','r')
    plot(l2_2(1,:),l2_2(2,:),'LineWidth',2,'Color','r')
    plot(l2_3(1,:),l2_3(2,:),'LineWidth',2,'Color','r')
    plot(l2_4(1,:),l2_4(2,:),'LineWidth',2,'Color','r')
    plot(l2_5(1,:),l2_5(2,:),'LineWidth',2,'Color','r')

    %plotting link3
    l3_0=[c0 d0];
    l3_1=[c1 d1];
    l3_2=[c2 d2];
    l3_3=[c3 d3];
    l3_4=[c4 d4];
    l3_5=[c5 d5];
    
    
    plot(l3_0(1,:),l3_0(2,:),'LineWidth',2,'Color','g')
    plot(l3_1(1,:),l3_1(2,:),'LineWidth',2,'Color','g')
    plot(l3_2(1,:),l3_2(2,:),'LineWidth',2,'Color','g')
    plot(l3_3(1,:),l3_3(2,:),'LineWidth',2,'Color','g')
    plot(l3_4(1,:),l3_4(2,:),'LineWidth',3,'Color','g')
    plot(l3_5(1,:),l3_5(2,:),'LineWidth',2,'Color','g')
    
    pause(0.025)
    movieVector(j) = getframe(figh);
    j = j+1;
end
%% linear
linear_diff=sqrt((xi-xf)^2+(yi-yf)^2);
number_of_linear_point=ceil(points_per_Cycle*linear_diff/Lstance);
for i=0:1:number_of_linear_point

    % calculating the next set of cordinates for each link and base
    q0=trajec_real(rem(S(1)*10+i,points_per_Cycle)+1,:);
    q1=trajec_real(rem(S(2)*10+i,points_per_Cycle)+1,:);
    q2=trajec_real(rem(S(3)*10+i,points_per_Cycle)+1,:);
    q3=trajec_real(rem(S(4)*10+i,points_per_Cycle)+1,:);
    q4=trajec_real(rem(S(5)*10+i,points_per_Cycle)+1,:);
    q5=trajec_real(rem(S(6)*10+i,points_per_Cycle)+1,:);

    base_pose=[i*Lstance/points_per_Cycle 0 nominal_height];
    base_LF=base_pose+[l/2 b/2 0];
    base_LR=base_pose+[-l/2 b/2 0];
    base_RF=base_pose+[l/2 -b/2 0];
    base_RR=base_pose+[-l/2 -b/2 0];
    base_LM=base_pose+[0 b 0];
    base_RM=base_pose+[0 -b 0];

    a0=base_pose+[Lstance+0.1 b/2 0];
    a1=base_pose+[0 b 0];
    a2=base_pose+[-Lstance-0.1 b/2 0];
    a3=base_pose+[Lstance+0.1 -b/2 0];
    a4=base_pose+[0 -b 0];
    a5=base_pose+[-Lstance-0.1 -b/2 0];

    b0=a0+[-l1*sin(q0(1)) +l1*cos(q0(1)) 0 ];
    b1=a1+[-l1*sin(q1(1)) +l1*cos(q1(1)) 0 ];
    b2=a2+[-l1*sin(q2(1)) +l1*cos(q2(1)) 0 ];
    b3=a3+[-l1*sin(q3(1)) -l1*cos(q3(1)) 0 ];
    b4=a4+[-l1*sin(q4(1)) -l1*cos(q4(1)) 0 ];
    b5=a5+[-l1*sin(q5(1)) -l1*cos(q5(1)) 0 ];

    c0=b0+[-l2*sin(q0(1))*cos(q0(2)) l2*cos(q0(1))*cos(q0(2))  l2*sin(q0(2))];
    c1=b1+[-l2*sin(q1(1))*cos(q1(2)) l2*cos(q1(1))*cos(q1(2))  l2*sin(q1(2))];
    c2=b2+[-l2*sin(q2(1))*cos(q2(2)) l2*cos(q2(1))*cos(q2(2))  l2*sin(q2(2))];
    c3=b3+[-l2*sin(q3(1))*cos(q3(2)) -l2*cos(q3(1))*cos(q3(2))  l2*sin(q3(2))];
    c4=b4+[-l2*sin(q4(1))*cos(q4(2)) -l2*cos(q4(1))*cos(q4(2))  l2*sin(q4(2))];
    c5=b5+[-l2*sin(q5(1))*cos(q5(2)) -l2*cos(q5(1))*cos(q5(2))  l2*sin(q5(2))];

    d0=c0+[-l3*sin(q0(1))*cos(q0(2)+q0(3)) l3*cos(q0(1))*cos(q0(2)+q0(3))  l3*sin(q0(2)+q0(3))];
    d1=c1+[-l3*sin(q1(1))*cos(q1(2)+q1(3)) l3*cos(q1(1))*cos(q1(2)+q1(3))  l3*sin(q1(2)+q1(3))];
    d2=c2+[-l3*sin(q2(1))*cos(q2(2)+q2(3)) l3*cos(q2(1))*cos(q2(2)+q2(3))  l3*sin(q2(2)+q2(3))];
    d3=c3+[-l3*sin(q3(1))*cos(q3(2)+q3(3)) -l3*cos(q3(1))*cos(q3(2)+q3(3))  l3*sin(q3(2)+q3(3))];
    d4=c4+[-l3*sin(q4(1))*cos(q4(2)+q4(3)) -l3*cos(q4(1))*cos(q4(2)+q4(3))  l3*sin(q4(2)+q4(3))];
    d5=c5+[-l3*sin(q5(1))*cos(q5(2)+q5(3)) -l3*cos(q5(1))*cos(q5(2)+q5(3))  l3*sin(q5(2)+q5(3))];

    base_LF=RotZ(h)*base_LF'+[xi;yi;0];
    base_LR=RotZ(h)*base_LR'+[xi;yi;0];
    base_RF=RotZ(h)*base_RF'+[xi;yi;0];
    base_RR=RotZ(h)*base_RR'+[xi;yi;0];
    base_LM=RotZ(h)*base_LM'+[xi;yi;0];
    base_RM=RotZ(h)*base_RM'+[xi;yi;0];
    
    [base_LF(1),base_LF(2)]=Projection_per(base_LF,xc,yc,zc);
    [base_LR(1),base_LR(2)]=Projection_per(base_LR,xc,yc,zc);
    [base_RF(1),base_RF(2)]=Projection_per(base_RF,xc,yc,zc);
    [base_RR(1),base_RR(2)]=Projection_per(base_RR,xc,yc,zc);
    [base_LM(1),base_LM(2)]=Projection_per(base_LM,xc,yc,zc);
    [base_RM(1),base_RM(2)]=Projection_per(base_RM,xc,yc,zc);

    a0=RotZ(h)*a0'+[xi;yi;0];
    a1=RotZ(h)*a1'+[xi;yi;0];
    a2=RotZ(h)*a2'+[xi;yi;0];
    a3=RotZ(h)*a3'+[xi;yi;0];
    a4=RotZ(h)*a4'+[xi;yi;0];
    a5=RotZ(h)*a5'+[xi;yi;0];
    
    [a0(1),a0(2)]=Projection_per(a0,xc,yc,zc); 
    [a1(1),a1(2)]=Projection_per(a1,xc,yc,zc); 
    [a2(1),a2(2)]=Projection_per(a2,xc,yc,zc); 
    [a3(1),a3(2)]=Projection_per(a3,xc,yc,zc); 
    [a4(1),a4(2)]=Projection_per(a4,xc,yc,zc); 
    [a5(1),a5(2)]=Projection_per(a5,xc,yc,zc);

    b0=RotZ(h)*b0'+[xi;yi;0];
    b1=RotZ(h)*b1'+[xi;yi;0];
    b2=RotZ(h)*b2'+[xi;yi;0];
    b3=RotZ(h)*b3'+[xi;yi;0];
    b4=RotZ(h)*b4'+[xi;yi;0];
    b5=RotZ(h)*b5'+[xi;yi;0];
    
    [b0(1),b0(2)]=Projection_per(b0,xc,yc,zc); 
    [b1(1),b1(2)]=Projection_per(b1,xc,yc,zc); 
    [b2(1),b2(2)]=Projection_per(b2,xc,yc,zc); 
    [b3(1),b3(2)]=Projection_per(b3,xc,yc,zc); 
    [b4(1),b4(2)]=Projection_per(b4,xc,yc,zc); 
    [b5(1),b5(2)]=Projection_per(b5,xc,yc,zc);

    c0=RotZ(h)*c0'+[xi;yi;0];
    c1=RotZ(h)*c1'+[xi;yi;0];
    c2=RotZ(h)*c2'+[xi;yi;0];
    c3=RotZ(h)*c3'+[xi;yi;0];
    c4=RotZ(h)*c4'+[xi;yi;0];
    c5=RotZ(h)*c5'+[xi;yi;0];
    
    [c0(1),c0(2)]=Projection_per(c0,xc,yc,zc); 
    [c1(1),c1(2)]=Projection_per(c1,xc,yc,zc); 
    [c2(1),c2(2)]=Projection_per(c2,xc,yc,zc); 
    [c3(1),c3(2)]=Projection_per(c3,xc,yc,zc); 
    [c4(1),c4(2)]=Projection_per(c4,xc,yc,zc); 
    [c5(1),c5(2)]=Projection_per(c5,xc,yc,zc);

    d0=RotZ(h)*d0'+[xi;yi;0];
    d1=RotZ(h)*d1'+[xi;yi;0];
    d2=RotZ(h)*d2'+[xi;yi;0];
    d3=RotZ(h)*d3'+[xi;yi;0];
    d4=RotZ(h)*d4'+[xi;yi;0];
    d5=RotZ(h)*d5'+[xi;yi;0];
    
    [d0(1),d0(2)]=Projection_per(d0,xc,yc,zc); 
    [d1(1),d1(2)]=Projection_per(d1,xc,yc,zc); 
    [d2(1),d2(2)]=Projection_per(d2,xc,yc,zc); 
    [d3(1),d3(2)]=Projection_per(d3,xc,yc,zc); 
    [d4(1),d4(2)]=Projection_per(d4,xc,yc,zc); 
    [d5(1),d5(2)]=Projection_per(d5,xc,yc,zc); 


    %plotting
    %plotting base
    clf
    base_points=[base_LF base_LM base_LR base_RR base_RM base_RF];
    hold on
    patch(base_points(1,:),base_points(2,:),'db', 'facealpha',0.4);
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    xlim([xi-3 xi+3])    
    ylim([yi-3 yi+3])
    zlim([0 3])
    %plotting link1
    
    l1_0=[a0 b0];
    l1_1=[a1 b1];
    l1_2=[a2 b2];
    l1_3=[a3 b3];
    l1_4=[a4 b4];
    l1_5=[a5 b5];    


    plot(l1_0(1,:),l1_0(2,:),'LineWidth',2,'Color','k')
    plot(l1_1(1,:),l1_1(2,:),'LineWidth',2,'Color','k')
    plot(l1_2(1,:),l1_2(2,:),'LineWidth',2,'Color','k')
    plot(l1_3(1,:),l1_3(2,:),'LineWidth',2,'Color','k')
    plot(l1_4(1,:),l1_4(2,:),'LineWidth',2,'Color','k')
    plot(l1_5(1,:),l1_5(2,:),'LineWidth',2,'Color','k')

    %plotting link2
    l2_0=[b0 c0];
    l2_1=[b1 c1];
    l2_2=[b2 c2];
    l2_3=[b3 c3];
    l2_4=[b4 c4];
    l2_5=[b5 c5];
    
    plot(l2_0(1,:),l2_0(2,:),'LineWidth',2,'Color','r')
    plot(l2_1(1,:),l2_1(2,:),'LineWidth',2,'Color','r')
    plot(l2_2(1,:),l2_2(2,:),'LineWidth',2,'Color','r')
    plot(l2_3(1,:),l2_3(2,:),'LineWidth',2,'Color','r')
    plot(l2_4(1,:),l2_4(2,:),'LineWidth',2,'Color','r')
    plot(l2_5(1,:),l2_5(2,:),'LineWidth',2,'Color','r')

    %plotting link3
    l3_0=[c0 d0];
    l3_1=[c1 d1];
    l3_2=[c2 d2];
    l3_3=[c3 d3];
    l3_4=[c4 d4];
    l3_5=[c5 d5];
    
    
    plot(l3_0(1,:),l3_0(2,:),'LineWidth',2,'Color','g')
    plot(l3_1(1,:),l3_1(2,:),'LineWidth',2,'Color','g')
    plot(l3_2(1,:),l3_2(2,:),'LineWidth',2,'Color','g')
    plot(l3_3(1,:),l3_3(2,:),'LineWidth',2,'Color','g')
    plot(l3_4(1,:),l3_4(2,:),'LineWidth',3,'Color','g')
    plot(l3_5(1,:),l3_5(2,:),'LineWidth',2,'Color','g')
    
    pause(0.025)
    
    movieVector(j) = getframe(figh);
    j = j+1;
end

myWriter = VideoWriter('Hex_proj');
myWriter.FrameRate = 60;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);

disp('DONE!')
