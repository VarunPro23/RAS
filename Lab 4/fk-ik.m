%% R||R||R
clear all
syms the1 the2 the3 a1 a2 a3

z_offset = 53.5
H01= [cos(the1) 0 -sin(the1)  0;...  %0H1 matrix
      sin(the1) 0 cos(the1)  0;.
      ..
       0        -1   0        a1;...
       0        0    0         1 ]

H12= [-sin(the2) -cos(the2)  0  90 + a2*sin(the2);...  %1H2 matrix
      cos(the2) -sin(the2)  0  -a2*cos(the2);...
       0           0        1   0;...
       0           0        0    1 ]

H23 = [cos(the3 - the2) -sin(the3 - the2) 0  a3*sin(the3 - the2);
       sin(the3 - the2) cos(the3 - the2)  0 -a3*cos(the3 - the2);
       0          0         1   0;
       0          0         0  1]

H03=simplify(H01*H12 *H23);%OH2=0H1*1H2

H03


Px = cos(the1)*(a3*cos(the3) + a2*sin(the2) + 90)
Py = sin(the1)*(a3*cos(the3) + a2*sin(the2) + 90) 
Pz = a1 + a2*cos(the2) - a3*sin(the3) - z_offset


t1=90/180*pi;

t2=20/180*pi

t3=30/180*pi

% Calculation Forward Kinematics

 a1_v=53.5, a2_v=150, a3_v=150

 px_new=vpa(subs(Px,[the1,the2, the3,a1,a2,a3],[t1,t2, t3,a1_v,a2_v,a3_v]))

 py_new=vpa(subs(Py,[the1,the2, the3,a1,a2,a3],[t1,t2, t3,a1_v,a2_v,a3_v]))
 
 pz_new=vpa(subs(Pz,[the1,the2, the3,a1,a2,a3],[t1,t2, t3,a1_v,a2_v,a3_v]))
    
 [t1,t2,t3] = solve(Px==px_new,Py==py_new,Pz==pz_new,the1,the2,the3)

 Theta1=vpa((t1)*180/pi)

 Theta2=vpa((t2)*180/pi)

 Theta3=vpa((t3)*180/pi)

 Solution = [Theta1 Theta2 Theta3]



%% Inverse Kinematics

clc;
clear;


a1 = 53.5;
a2 = 150;
a3 = 150;
z_offset = 53.5;


px_target = 280;   %  X target
py_target = -195;    %  Y target
pz_target = 15;    %  Z target


equations = @(angles) [
    cos(angles(1)) * (a3*cos(angles(3)) + a2*sin(angles(2)) + 90) - px_target;
    sin(angles(1)) * (a3*cos(angles(3)) + a2*sin(angles(2)) + 90) - py_target;
    (a1 + a2*cos(angles(2)) - a3*sin(angles(3)) - z_offset) - pz_target
];


initial_guess = [0.5, 0.5, 0.5];


options = optimoptions('fsolve', 'Display', 'off');
solution = fsolve(equations, initial_guess, options);


theta_deg = rad2deg(solution);


fprintf('Inverse Kinematics Solution:\n');
fprintf('Theta1 = %.4f°\n', theta_deg(1));
fprintf('Theta2 = %.4f°\n', theta_deg(2));
fprintf('Theta3 = %.4f°\n', theta_deg(3));
