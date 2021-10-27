q0=[0.5,1.3,-2.7,0];
q1=[0.1,1.3,-2.1,0];
q2=[0.1,1.3,-2.9,0];
q3=[0.1,1.3,-2.5,0];
q4=[0.1,1.3,-2.3,0];
q5=[0.1,1.3,-2.7,0];
%% dimensions
l=0.8;
b=0.3;

%% hexapod description
for i=0:1:1000
    % calculating the next set of cordinates for each link and base
    q0=trajec_real(rem(0*10+i,20)+1,:);
    q1=trajec_real(rem(1*10+i,20)+1,:);
    q2=trajec_real(rem(0*10+i,20)+1,:);
    q3=trajec_real(rem(1*10+i,20)+1,:);
    q4=trajec_real(rem(0*10+i,20)+1,:);
    q5=trajec_real(rem(1*10+i,20)+1,:);

	base_pose=[i*Lstance/points_per_Cycle 0 0];
	base_LF=base_pose+[l/2 b/2 0];
	base_LR=base_pose+[-l/2 b/2 0];
	base_RF=base_pose+[l/2 -b/2 0];
	base_RR=base_pose+[-l/2 -b/2 0];

	a0=base_pose+[Lstance+0.1 b/2 0];
	a1=base_pose+[0 b/2 0];
	a2=base_pose+[-Lstance-0.1 b/2 0];
	a3=base_pose+[Lstance+0.1 -b/2 0];
	a4=base_pose+[0 -b/2 0];
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


	%plotting
	%plotting base
    clf
	base_points=[base_LF' base_LR' base_RR' base_RF'];
	plot3(base_points(1,:),base_points(2,:),base_points(3,:))
	hold on
	patch(base_points(1,:),base_points(2,:),base_points(3,:),'m')
	grid on
	xlabel('x')
	ylabel('y')
	zlabel('z')
    xlim([-0.8 2])    
    ylim([-0.8 2])
    zlim([-0.3 2])
	%plotting link1
	l1_0=[a0' b0'];
	l1_1=[a1' b1'];
	l1_2=[a2' b2'];
	l1_3=[a3' b3'];
	l1_4=[a4' b4'];
	l1_5=[a5' b5'];    xlim([-0.8 2])    
    ylim([-0.8 2])
    zlim([-0.3 2])

	plot3(l1_0(1,:),l1_0(2,:),l1_0(3,:),'LineWidth',4)
	plot3(l1_1(1,:),l1_1(2,:),l1_1(3,:),'LineWidth',4)
	plot3(l1_2(1,:),l1_2(2,:),l1_2(3,:),'LineWidth',4)
	plot3(l1_3(1,:),l1_3(2,:),l1_3(3,:),'LineWidth',4)
	plot3(l1_4(1,:),l1_4(2,:),l1_4(3,:),'LineWidth',4)
	plot3(l1_5(1,:),l1_5(2,:),l1_5(3,:),'LineWidth',4)

	%plotting link2
	l2_0=[b0' c0'];
	l2_1=[b1' c1'];
	l2_2=[b2' c2'];
	l2_3=[b3' c3'];
	l2_4=[b4' c4'];
	l2_5=[b5' c5'];
	plot3(l2_0(1,:),l2_0(2,:),l2_0(3,:),'LineWidth',4)
	plot3(l2_1(1,:),l2_1(2,:),l2_1(3,:),'LineWidth',4)
	plot3(l2_2(1,:),l2_2(2,:),l2_2(3,:),'LineWidth',4)
	plot3(l2_3(1,:),l2_3(2,:),l2_3(3,:),'LineWidth',4)
	plot3(l2_4(1,:),l2_4(2,:),l2_4(3,:),'LineWidth',4)
	plot3(l2_5(1,:),l2_5(2,:),l2_5(3,:),'LineWidth',4)

	%plotting link3
	l3_0=[c0' d0'];
	l3_1=[c1' d1'];
	l3_2=[c2' d2'];
	l3_3=[c3' d3'];
	l3_4=[c4' d4'];
	l3_5=[c5' d5'];
	plot3(l3_0(1,:),l3_0(2,:),l3_0(3,:),'LineWidth',4)
	plot3(l3_1(1,:),l3_1(2,:),l3_1(3,:),'LineWidth',4)
	plot3(l3_2(1,:),l3_2(2,:),l3_2(3,:),'LineWidth',4)
	plot3(l3_3(1,:),l3_3(2,:),l3_3(3,:),'LineWidth',4)
	plot3(l3_4(1,:),l3_4(2,:),l3_4(3,:),'LineWidth',4)
	plot3(l3_5(1,:),l3_5(2,:),l3_5(3,:),'LineWidth',4)
    pause(0.05)
    
end