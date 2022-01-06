%% MOCOM LAB2 - Luca Mosetti Matteo Maragliano Emanuele Rambaldi

clc
clear
close all

addpath("./include")

%% Definition of the home-position matrices
biTri(1,1,1) = 1;  biTri(1,2,1) = 0;  biTri(1,3,1) = 0;  biTri(1,4,1) = 0;
biTri(2,1,1) = 0;  biTri(2,2,1) = 1;  biTri(2,3,1) = 0;  biTri(2,4,1) = 0;
biTri(3,1,1) = 0;  biTri(3,2,1) = 0;  biTri(3,3,1) = 1;  biTri(3,4,1) = 0.175;
biTri(4,1,1) = 0;  biTri(4,2,1) = 0;  biTri(4,3,1) = 0;  biTri(4,4,1) = 1;

biTri(1,1,2) = -1; biTri(1,2,2) = 0;  biTri(1,3,2) = 0;  biTri(1,4,2) = 0;
biTri(2,1,2) = 0;  biTri(2,2,2) = 0;  biTri(2,3,2) = 1;  biTri(2,4,2) = 0;
biTri(3,1,2) = 0;  biTri(3,2,2) = 1;  biTri(3,3,2) = 0;  biTri(3,4,2) = 0.108;
biTri(4,1,2) = 0;  biTri(4,2,2) = 0;  biTri(4,3,2) = 0;  biTri(4,4,2) = 1;

biTri(1,1,3) = 0;  biTri(1,2,3) = 0;  biTri(1,3,3) = 1;  biTri(1,4,3) = 0.105;
biTri(2,1,3) = -1; biTri(2,2,3) = 0;  biTri(2,3,3) = 0;  biTri(2,4,3) = 0;
biTri(3,1,3) = 0;  biTri(3,2,3) = -1; biTri(3,3,3) = 0;  biTri(3,4,3) = 0;
biTri(4,1,3) = 0;  biTri(4,2,3) = 0;  biTri(4,3,3) = 0;  biTri(4,4,3) = 1;

biTri(1,1,4) = 0;  biTri(1,2,4) = 1;  biTri(1,3,4) = 0;  biTri(1,4,4) = -0.1455;
biTri(2,1,4) = 0;  biTri(2,2,4) = 0;  biTri(2,3,4) = -1; biTri(2,4,4) = 0;
biTri(3,1,4) = -1; biTri(3,2,4) = 0;  biTri(3,3,4) = 0;  biTri(3,4,4) = 0.3265;
biTri(4,1,4) = 0;  biTri(4,2,4) = 0;  biTri(4,3,4) = 0;  biTri(4,4,4) = 1;

biTri(1,1,5) = 0;  biTri(1,2,5) = 0;  biTri(1,3,5) = -1; biTri(1,4,5) = 0.095;
biTri(2,1,5) = 0;  biTri(2,2,5) = -1; biTri(2,3,5) = 0;  biTri(2,4,5) = 0;
biTri(3,1,5) = -1; biTri(3,2,5) = 0;  biTri(3,3,5) = 0;  biTri(3,4,5) = 0;
biTri(4,1,5) = 0;  biTri(4,2,5) = 0;  biTri(4,3,5) = 0;  biTri(4,4,5) = 1;

biTri(1,1,6) = 0;  biTri(1,2,6) = 0;  biTri(1,3,6) = 1;  biTri(1,4,6) = 0;
biTri(2,1,6) = 0;  biTri(2,2,6) = 1;  biTri(2,3,6) = 0;  biTri(2,4,6) = 0;
biTri(3,1,6) = -1; biTri(3,2,6) = 0;  biTri(3,3,6) = 0;  biTri(3,4,6) = -0.325;
biTri(4,1,6) = 0;  biTri(4,2,6) = 0;  biTri(4,3,6) = 0;  biTri(4,4,6) = 1;

biTri(1,1,7) = 0;  biTri(1,2,7) = 0;  biTri(1,3,7) = 1;  biTri(1,4,7) = 0.132;
biTri(2,1,7) = 0;  biTri(2,2,7) = 1;  biTri(2,3,7) = 0;  biTri(2,4,7) = 0;
biTri(3,1,7) = -1; biTri(3,2,7) = 0;  biTri(3,3,7) = 0;  biTri(3,4,7) = 0;
biTri(4,1,7) = 0;  biTri(4,2,7) = 0;  biTri(4,3,7) = 0;  biTri(4,4,7) = 1;

% Constant matrix from link 7 to the end-eff.
T_7e(1,1) = 1;  T_7e(1,2) = 0;  T_7e(1,3) = 0;  T_7e(1,4) = 0;
T_7e(2,1) = 0;  T_7e(2,2) = 1;  T_7e(2,3) = 0;  T_7e(2,4) = 0;
T_7e(3,1) = 0;  T_7e(3,2) = 0;  T_7e(3,3) = 1;  T_7e(3,4) = 0.021;
T_7e(4,1) = 0;  T_7e(4,2) = 0;  T_7e(4,3) = 0;  T_7e(4,4) = 1;

% Define the type of joints and their angle bounds
numberOfJoints= 7;
jointTypes= [0 ; 0 ; 0 ; 0 ; 0 ; 0; 0];
q_min= [-pi ; 0 ; -pi ; -pi ; -pi ; -pi; -pi];
q_max= [ pi ; 2*pi ; pi ; pi ; pi ;  pi;  pi];


%% ========================================================================
%                          KINEMATIC SIMULATION
%  ========================================================================

% simulation variables
dt= 0.01;               % time step
t_i= 0.0;               % initial time
t_f= 10.0;              % final time
t= t_i:dt:t_f;          % number of iterations

% define the angular and linear gains
gamma_a= 10;    % w_e= gamma_a * angular_err
gamma_l= 10;    % v_e= gamma_l * linear_err

% initial condition: manipulator in the home-position
q= [0 ; 0 ; 0 ; 0 ; 0 ; 0; 0];

% set the final goal: for instance we can try to make the end-eff slide
% for a certain distance provided by T
T= [1     0     0     -0.1;  ...
    0     1     0       0 ;  ...
    0     0     1       0 ;  ...
    0     0     0       1 ;  ];
biTei= GetDirectGeometry(q, biTri, jointTypes);
T_goal= GetTransformationWrtBase(biTei, 7) * T_7e * T;

% set up the plot parameters
figure;
title("Manipulator's motion");
xlabel('x');
ylabel('y');
zlabel('z');
grid on
axis equal
hold on;

for i = t

    % compute the transformation matrices of link <i> w.r.t. link <i-1> for
    % the current configuration q
    biTei= GetDirectGeometry(q, biTri, jointTypes);
    % compute the transformation matrix of the end-eff. w.r.t. the base
    bTe= GetTransformationWrtBase(biTei, 7) * T_7e;

    % get the vectors that show the position of the joints <i> and the end-eff
    % w.r.t. the base in order to plot them later on
    r1= GetBasicVectorWrtBase(biTei, 1);
    r2= GetBasicVectorWrtBase(biTei, 2);
    r3= GetBasicVectorWrtBase(biTei, 3);
    r4= GetBasicVectorWrtBase(biTei, 4);
    r5= GetBasicVectorWrtBase(biTei, 5);
    r6= GetBasicVectorWrtBase(biTei, 6);
    r7= GetBasicVectorWrtBase(biTei, 7);
    re= bTe(1:3, 4);

    % compute the end-eff Jacobian for the current configuration
    J= GetJacobian(biTei, bTe, jointTypes);

    % compute the error committed at the current step
    err= ComputeError(bTe, T_goal);

    % compute the angular and linear end-eff velocities proportionally to
    % the error committed
    w_e= gamma_a * err(1:3);
    v_e= gamma_l * err(4:6);
    x_dot= [w_e; v_e];         % overall velocity

    % compute the joint velocities using the inverse kinematic formula
    q_dot= pinv(J) * x_dot;

    %% Plot the manipulator motion
    % Plot the links
    plot3([0 r1(1)], [0 r1(2)], [0 r1(3)], 'b');
    plot3([r1(1) r2(1)], [r1(2) r2(2)], [r1(3) r2(3)], 'r');
    plot3([r2(1) r3(1)], [r2(2) r3(2)], [r2(3) r3(3)], 'g');
    plot3([r3(1) r4(1)], [r3(2) r4(2)], [r3(3) r4(3)], 'c');
    plot3([r4(1) r5(1)], [r4(2) r5(2)], [r4(3) r5(3)], 'b');
    plot3([r5(1) r6(1)], [r5(2) r6(2)], [r5(3) r6(3)], 'm');
    plot3([r6(1) r7(1)], [r6(2) r7(2)], [r6(3) r7(3)], 'r');
    plot3([r7(1) re(1)], [r7(2) re(2)], [r7(3) re(3)], 'k');

    % Plot the joints
    plot3([r1(1) r2(1)], [r1(2) r2(2)], [r1(3) r2(3)], 'ko');
    plot3([r2(1) r3(1)], [r2(2) r3(2)], [r2(3) r3(3)], 'ko');
    plot3([r3(1) r4(1)], [r3(2) r4(2)], [r3(3) r4(3)], 'ko');
    plot3([r4(1) r5(1)], [r4(2) r5(2)], [r4(3) r5(3)], 'ko');
    plot3([r5(1) r6(1)], [r5(2) r6(2)], [r5(3) r6(3)], 'ko');
    plot3([r6(1) r7(1)], [r6(2) r7(2)], [r6(3) r7(3)], 'ko');

    % Plot the end-eff.
    plot3([0 re(1)], [0 re(2)], [0 re(3)], 'kx');

    %% update the robot's configuration
    q= KinematicSimulation(q, q_dot, dt, q_min, q_max);

end
